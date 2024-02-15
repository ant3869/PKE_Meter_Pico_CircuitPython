"""

Gain: Higher ranges have less precision but can measure larger movements!

Example:

button_manager.check_and_handle_buttons()

while True:
    button_manager.check_and_handle_buttons()
    time.sleep(0.1)
    
"""

import time
import board
import analogio
import digitalio
import adafruit_lsm9ds1

class ButtonManager:
    
# --- constants   --------------------------------------------------------

    BUTTON_GAIN_CYCLE           = board.GP26
    BUTTON_SENSITIVITY_DECREASE = board.GP27
    BUTTON_SENSITIVITY_INCREASE = board.GP28   
    LED_INCREASED   = board.GP14
    LED_DECREASED   = board.GP15
    SW_THRESHOLD    = 61000
    LED_ON_DURATION = 1.0  

# --- constructors   ------------------------------------------------------

    def __init__(self, sensor, debounce_delay=0.500):
        self.sensor = sensor
        self.debounce_delay = debounce_delay
        self.last_debounce_time = 0

        # Initialize LED pins
        self.led_increase = self.setup_led(self.LED_INCREASED)
        self.led_decrease = self.setup_led(self.LED_DECREASED)
        
        # Initialize button pins
        self.button_cycle_gain = analogio.AnalogIn(self.BUTTON_GAIN_CYCLE)
        self.button_increase_sensitivity = analogio.AnalogIn(self.BUTTON_SENSITIVITY_INCREASE)
        self.button_decrease_sensitivity = analogio.AnalogIn(self.BUTTON_SENSITIVITY_DECREASE)

        # LED timing variables
        self.led_increased_start = 0
        self.led_decreased_start = 0

        # Sensitivity and gain settings
        self.init_sensitivity_and_gain_settings()

    def setup_led(self, pin):
        led = digitalio.DigitalInOut(pin)
        led.direction = digitalio.Direction.OUTPUT
        return led

# --- variables   --------------------------------------------------------

    def init_sensitivity_and_gain_settings(self):
        # Initialize settings for sensitivity and gain
        self.sensitivity        = 0.5
        self.sensitivity_min    = 0.1
        self.sensitivity_max    = 1.0
        self.sensitivity_step   = 0.1
        self.current_gain_index = 0
        self.mag_gains = [adafruit_lsm9ds1.MAGGAIN_4GAUSS, adafruit_lsm9ds1.MAGGAIN_8GAUSS,
                          adafruit_lsm9ds1.MAGGAIN_12GAUSS, adafruit_lsm9ds1.MAGGAIN_16GAUSS]

# --- functions   --------------------------------------------------------

    def check_and_handle_buttons(self):
        current_time = time.monotonic()
        self.check_buttons(current_time)
        self.update_led_states_based_on_timing(current_time)

    def check_buttons(self, current_time):
        # Check each button and handle state changes
        if self.button_decrease_sensitivity.value > self.SW_THRESHOLD:
            self.adjust_sensitivity(False, current_time)
        elif self.button_increase_sensitivity.value > self.SW_THRESHOLD:
            self.adjust_sensitivity(True, current_time)
        elif self.button_cycle_gain.value > self.SW_THRESHOLD:
            self.cycle_mag_gain(current_time)

    def adjust_sensitivity(self, is_increase, current_time):
        # Adjust sensitivity and update LEDs
        if is_increase:
            self.sensitivity = min(self.sensitivity + self.sensitivity_step, self.sensitivity_max)
            self.led_on(self.led_increase, current_time)
        else:
            self.sensitivity = max(self.sensitivity - self.sensitivity_step, self.sensitivity_min)
            self.led_on(self.led_decrease, current_time)
        print(f"Sensitivity adjusted to {self.sensitivity}")

    def cycle_mag_gain(self, current_time):
        # Cycle through magnetometer gain settings
        self.current_gain_index = (self.current_gain_index + 1) % len(self.mag_gains)
        new_gain = self.mag_gains[self.current_gain_index]
        self.sensor.mag_gain = new_gain
        print(f"Magnetometer gain set to {self.sensor.mag_gain}")
        # Optionally, add LED feedback for cycling gain

    def led_on(self, led, current_time):
        led.value = True
        if led == self.led_increase:
            self.led_increased_start = current_time
        else:
            self.led_decreased_start = current_time

    def update_led_states_based_on_timing(self, current_time):
        if self.led_increase.value and (current_time - self.led_increased_start > self.LED_ON_DURATION):
            self.led_increase.value = False
        if self.led_decrease.value and (current_time - self.led_decreased_start > self.LED_ON_DURATION):
            self.led_decrease.value = False
