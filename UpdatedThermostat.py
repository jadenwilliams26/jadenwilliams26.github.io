import Adafruit_DHT
import RPi.GPIO as GPIO
import time
from collections import deque
from pymongo import MongoClient

# -----------------------------
# Sensor and GPIO pin assignments
# -----------------------------
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
HEATER_PIN = 17
MIST_PIN = 27
RED_LED = 22         # Turns on if temperature is too low
BLUE_LED = 23        # Turns on if humidity is too low
BUTTON_MODE = 5      # Cycles through setpoint modes
BUTTON_UP = 6        # Increases current setpoint
BUTTON_DOWN = 13     # Decreases current setpoint

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(HEATER_PIN, GPIO.OUT)
GPIO.setup(MIST_PIN, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(BLUE_LED, GPIO.OUT)

GPIO.setup(BUTTON_MODE, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# -----------------------------
# Database manager for species profiles
# -----------------------------
class SpeciesProfileDB:
    def __init__(self, uri="mongodb://localhost:27017/", db_name="reptile_env", coll_name="reptiles"):
        self.client = MongoClient(uri)
        self.collection = self.client[db_name][coll_name]

    def get_profile(self, species_name):
        """
        Fetch the species profile document. Returns a dict or None if not found.
        Expected format:
          {
            "species": "...",
            "temp_min": <float>,
            "temp_max": <float>,
            "humidity_min": <float>,
            "humidity_max": <float>
          }
        """
        return self.collection.find_one({"species": species_name})

    def list_species(self):
        return [doc["species"] for doc in self.collection.find({}, {"species": 1})]

# -----------------------------
# Circular buffer for moving average
# -----------------------------
class CircularBuffer:
    def __init__(self, size):
        self.buffer = deque(maxlen=size)

    def add(self, value):
        self.buffer.append(value)

    def average(self):
        if not self.buffer:
            return None
        return sum(self.buffer) / len(self.buffer)

# -----------------------------
# Device controllers
# -----------------------------
class HeaterControl:
    def __init__(self, pin):
        self.pin = pin
        GPIO.output(self.pin, GPIO.LOW)

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)

class MistingControl:
    def __init__(self, pin):
        self.pin = pin
        GPIO.output(self.pin, GPIO.LOW)

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)

# -----------------------------
# LCD Display placeholder
# -----------------------------
class LCDDisplay:
    def display(self, temperature, humidity, species, temp_range, humid_range):
        # Replace with real LCD code. Here’s what would show:
        print(f"Species: {species}")
        print(f"Temp: {temperature:.2f}°C (Target {temp_range[0]}–{temp_range[1]})")
        print(f"Humid: {humidity:.2f}% (Target {humid_range[0]}–{humid_range[1]})")
        print("-" * 40)

# -----------------------------
# Sensor manager with smoothing
# -----------------------------
class SensorManager:
    def __init__(self, sensor_type, pin, buffer_size=5):
        self.sensor = sensor_type
        self.pin = pin
        self.temp_buffer = CircularBuffer(buffer_size)
        self.humid_buffer = CircularBuffer(buffer_size)

    def read_averaged(self):
        humidity, temperature = Adafruit_DHT.read_retry(self.sensor, self.pin)
        if humidity is not None and temperature is not None:
            self.temp_buffer.add(temperature)
            self.humid_buffer.add(humidity)
            return self.temp_buffer.average(), self.humid_buffer.average()
        else:
            print("Sensor read failed")
            return None, None

# -----------------------------
# Thermostat logic that uses species profile
# -----------------------------
class Thermostat:
    def __init__(self, sensor_mgr, heater, mister, lcd, species_db, initial_species):
        self.sensor_mgr = sensor_mgr
        self.heater = heater
        self.mister = mister
        self.lcd = lcd
        self.species_db = species_db

        # Load the initial species profile
        self.current_species = None
        self.temp_range = [0, 0]
        self.humid_range = [0, 0]
        self.load_species_profile(initial_species)

    def load_species_profile(self, species_name):
        profile = self.species_db.get_profile(species_name)
        if not profile:
            print(f"[WARNING] Species '{species_name}' not found in database.")
            return
        # Set the target ranges from database document
        self.current_species = profile["species"]
        self.temp_range = [profile["temp_min"], profile["temp_max"]]
        self.humid_range = [profile["humidity_min"], profile["humidity_max"]]
        print(f"[INFO] Loaded profile for {self.current_species}: "
              f"Temp {self.temp_range}, Humidity {self.humid_range}")

    def cycle_species(self):
        """
        Cycle through species in DB when BUTTON_MODE is pressed.
        """
        species_list = self.species_db.list_species()
        if not species_list:
            return
        if self.current_species not in species_list:
            next_species = species_list[0]
        else:
            idx = species_list.index(self.current_species)
            next_species = species_list[(idx + 1) % len(species_list)]
        self.load_species_profile(next_species)

    def update(self):
        # Check for species change request
        if not GPIO.input(BUTTON_MODE):
            self.cycle_species()
            time.sleep(0.3)  # debounce

        temp, humid = self.sensor_mgr.read_averaged()
        if temp is None or humid is None:
            return

        # Display current readings and targets
        self.lcd.display(temp, humid, self.current_species, self.temp_range, self.humid_range)

        # Temperature control: heater logic
        if temp < self.temp_range[0]:
            self.heater.on()
            GPIO.output(RED_LED, GPIO.HIGH)  # indicate heating
        elif temp > self.temp_range[1]:
            self.heater.off()
            GPIO.output(RED_LED, GPIO.LOW)
        else:
            self.heater.off()
            GPIO.output(RED_LED, GPIO.LOW)

        # Humidity control: mister logic
        if humid < self.humid_range[0]:
            self.mister.on()
            GPIO.output(BLUE_LED, GPIO.HIGH)  # indicate misting
        elif humid > self.humid_range[1]:
            self.mister.off()
            GPIO.output(BLUE_LED, GPIO.LOW)
        else:
            self.mister.off()
            GPIO.output(BLUE_LED, GPIO.LOW)

# -----------------------------
# Main program
# -----------------------------
def main():
    # Initialize components
    species_db = SpeciesProfileDB()
    # Choose a default species to start with; will be cycled with BUTTON_MODE
    initial_species = "Leopard Gecko"

    sensor_mgr = SensorManager(DHT_SENSOR, DHT_PIN, buffer_size=5)
    heater = HeaterControl(HEATER_PIN)
    mister = MistingControl(MIST_PIN)
    lcd = LCDDisplay()
    thermostat = Thermostat(sensor_mgr, heater, mister, lcd, species_db, initial_species)

    try:
        while True:
            thermostat.update()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting gracefully...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()