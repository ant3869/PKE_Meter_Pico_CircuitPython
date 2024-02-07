"""
    A class to manage button inputs for adjusting sensitivity and cycling through
    magnetometer gain settings. It handles debouncing and distinguishes between
    increase/decrease actions for both sensitivity and gain.

    Attributes:
        button_increase_sensitivity: DigitalInOut object for the increase sensitivity button.
        button_decrease_sensitivity: DigitalInOut object for the decrease sensitivity button.
        button_increase_gain: DigitalInOut object for the increase gain button.
        button_decrease_gain: DigitalInOut object for the decrease gain button.
        debounce_delay: Time in seconds to debounce the buttons.
        last_debounce_time: Timestamp of the last button press to manage debounce.
        sensitivity: Placeholder attribute for sensitivity level.
        current_gain_index: Index to track the current magnetometer gain setting.
        mag_gains: List of available magnetometer gain settings.
    
    Methods:
        setup_button(pin): Initializes a button on the specified pin.
        check_and_handle_buttons(): Checks all managed buttons and performs associated actions.
        check_button_pressed(button, current_time): Checks if a button is pressed, considering debounce.
        adjust_sensitivity(is_increase): Adjusts sensitivity up or down.
        cycle_mag_gain(is_increase): Cycles through magnetometer gain settings.
        
    Reference:
        button_manager = ButtonManager()

        while True:
            button_manager.check_and_handle_buttons()
            # Additional application logic...
            time.sleep(0.01)  # Short delay in the loop to reduce CPU load
"""

import time
import board

class ButtonManager():
    
# --- constants   --------------------------------------------------------

  BUTTON_SENSITIVITY_DECREASE  = board.GP10  # Sensitivity Variable Decrease Button
  BUTTON_SENSITIVITY_INCREASE  = board.GP11  # Sensitivity Variable Increase Button
  BUTTON_GAIN_DECREASE         = board.GP12  # Gain Variable Decrease Button
  BUTTON_GAIN_INCREASE         = board.GP13  # Gain Variable Decrease Button

  LED_INCREASED                = board.GP14  # Increased Variable Indicator LED
  LED_DECREASED                = board.GP15  # Decreased Variable Indicator LED

# --- constructor   ------------------------------------------------------

  def __init__(self, increase_gain_pin  = None,
    decrease_gain_pin        = None,
    increase_sensitivity_pin = None,
    decrease_sensitivity_pin = None,
    led_increase_pin         = None,
    led_decrease_pin         = None,
    debounce_delay           = 0.500):
     
    # Initialize sensitivity button pins
    self.button_increase_sensitivity = self.setup_button(increase_sensitivity_pin or self.BUTTON_SENSITIVITY_INCREASE)
    self.button_decrease_sensitivity = self.setup_button(decrease_sensitivity_pin or self.BUTTON_SENSITIVITY_DECREASE)

    # Initialize gain button pins
    self.button_increase_gain = self.setup_button(increase_gain_pin or self.BUTTON_GAIN_INCREASE)
    self.button_decrease_gain = self.setup_button(decrease_gain_pin or self.BUTTON_GAIN_DECREASE)

    # Setup LEDs
    self.led_increase = self.setup_led(led_increase_pin if led_increase_pin else self.LED_INCREASED)
    self.led_decrease = self.setup_led(led_decrease_pin if led_decrease_pin else self.LED_DECREASED)
        
# --- variables   --------------------------------------------------------
        
    # Debounce handling
    self.debounce_delay     = debounce_delay
    self.last_debounce_time = 0

    # Sensitivity variables
    self.sensitivity      = 0.5  # Starting sensitivity, adjust as needed
    self.sensitivity_min  = 0.1  # Minimum sensitivity
    self.sensitivity_max  = 1.0  # Maximum sensitivity
    self.sensitivity_step = 0.1  # Sensitivity adjustment step

    # Gain variables
    self.current_gain_index = 0
    self.mag_gains          = ["MAGGAIN_4GAUSS", "MAGGAIN_8GAUSS", "MAGGAIN_12GAUSS", "MAGGAIN_16GAUSS"]
     
# --- functions   --------------------------------------------------------

  def setup_button(self, pin):
    """Initializes and returns a DigitalInOut object for a button on the specified pin."""
    button = digitalio.DigitalInOut(pin)
    button.direction = digitalio.Direction.INPUT
    button.pull = digitalio.Pull.UP  # Use internal pull-up resistor
    return button

  def setup_led(self, pin):
    """Initializes and returns a DigitalInOut object for an LED on the specified pin."""
    led = digitalio.DigitalInOut(pin)
    led.direction = digitalio.Direction.OUTPUT
    return led

  def check_and_handle_buttons(self):
    """Check each button and adjust sensitivity or gain accordingly"""
    self.handle_button_press(self.button_increase_sensitivity, True, self.adjust_sensitivity)
    self.handle_button_press(self.button_decrease_sensitivity, False, self.adjust_sensitivity)
    self.handle_button_press(self.button_increase_gain, True, self.cycle_mag_gain)
    self.handle_button_press(self.button_decrease_gain, False, self.cycle_mag_gain)

  def handle_button_press(self, button, is_increase, action_method):
    if self.check_button_pressed(button):
        action_method(is_increase)
        # Toggle corresponding LED based on the action
        self.toggle_led(self.led_increase if is_increase else self.led_decrease)

  def check_button_pressed(self, button):
    current_time = time.monotonic()
    if button.value and current_time - self.last_debounce_time > self.debounce_delay:
        self.last_debounce_time = current_time
        return True
    return False

  def toggle_led(self, led, duration=0.2):
    led.value = True
    time.sleep(duration)  # Provide visual feedback for a short duration
    led.value = False

  def adjust_sensitivity(self, is_increase):
    # Adjust sensitivity within limits and provide feedback
    new_sensitivity = self.sensitivity + (self.SENSITIVITY_STEP if is_increase else -self.SENSITIVITY_STEP)
    self.sensitivity = max(self.SENSITIVITY_MIN, min(new_sensitivity, self.SENSITIVITY_MAX))
    print(f"Sensitivity adjusted to {self.sensitivity}")

  def cycle_mag_gain(self, is_increase):
    # Cycle through the gain settings and wrap around as needed
    self.current_gain_index += 1 if is_increase else -1
    self.current_gain_index %= len(self.MAG_GAINS)
        
  def check_and_handle_buttons(self):
    current_time = time.monotonic()
    # Handle sensitivity buttons
    if self.check_button_pressed(self.button_increase_sensitivity, current_time):
        self.adjust_sensitivity(True)
        self.led_increase.value = True  # Turn on LED as feedback
        print("Sensitivity Increase button pressed")
    if self.check_button_pressed(self.button_decrease_sensitivity, current_time):
        self.adjust_sensitivity(False)
        self.led_decrease.value = True  # Turn on LED as feedback
        print("Sensitivity Decrease button pressed")
    
    # Handle gain buttons
    if self.check_button_pressed(self.button_increase_gain, current_time):
        self.cycle_mag_gain(True)
        self.led_increase.value = True  # Turn on LED as feedback
        print("Gain Increase button pressed")
    if self.check_button_pressed(self.button_decrease_gain, current_time):
        self.cycle_mag_gain(False)
        self.led_decrease.value = True  # Turn on LED as feedback
        print("Gain Decrease button pressed") 

    # Debounce delay handling
    time.sleep(self.debounce_delay)

  def check_button_pressed(self, button, current_time):
    """Simple debounce logic."""
    if button.value and current_time - self.last_debounce_time > self.debounce_delay:
        self.last_debounce_time = current_time
        return True
    return False

 def adjust_sensitivity(self, is_increase):
    """Adjusts sensitivity within defined limits."""
    self.sensitivity += self.sensitivity_step * (1 if is_increase else -1)
    self.sensitivity = max(self.sensitivity_min, min(self.sensitivity_max, self.sensitivity))
    print(f"Sensitivity adjusted to {self.sensitivity}")
    
    print(f"Sensitivity adjusted to {self.sensitivity}")
    # apply the sensitivity adjustment to your application

  def cycle_mag_gain(self, is_increase):
    """Cycles through magnetometer gain settings."""
    self.current_gain_index += 1 if is_increase else -1
    self.current_gain_index %= len(self.mag_gains)  # Wrap index
    print(f"Magnetometer gain set to {self.mag_gains[self.current_gain_index]}")

  def cycle_mag_gain(self, is_increase):
    """Cycles through magnetometer gain settings."""
    self.current_gain_index += 1 if is_increase else -1
    self.current_gain_index %= len(self.mag_gains)  # Wrap index
    print(f"Magnetometer gain set to {self.mag_gains[self.current_gain_index]}")
