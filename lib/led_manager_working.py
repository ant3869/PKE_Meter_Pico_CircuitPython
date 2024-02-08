"""
EMF_MIN_ADC = 0  # Minimum EMF reading
EMF_MAX_ADC = 1100  # Maximum EMF reading
led_slow = 1.0  # Slow blinking interval in seconds
led_fast = 0.1  # Fast blinking interval in seconds
led_pins = [board.D2, board.D3, board.D4]  # Example LED pins

# Initialize LED manager
led_manager = LEDManager(led_pins)

# Main loop
while True:
    emf_reading = 550  # Example EMF reading, replace with real data
    led_manager.led_control(emf_reading, EMF_MIN_ADC, EMF_MAX_ADC, led_slow, led_fast)
    time.sleep(0.01)  # Short delay to avoid hogging CPU
"""

import time
import board
import digitalio

class LEDManager:
    def __init__(self, led_pins):
        self.led_objects = [self._setup_led(pin) for pin in led_pins]
        self.current_led = 0  # Tracks the current LED for blinking
        self.previous_millis = time.monotonic()
        self.led_states = [False] * len(led_pins)  # Initial state of LEDs
        self.led_timestamps = [0] * len(led_pins)  # Timestamps for state changes

    def _setup_led(self, pin):
        """Setup an individual LED."""
        led = digitalio.DigitalInOut(pin)
        led.direction = digitalio.Direction.OUTPUT
        return led

    def update_led_state(self, led_index, state):
        """Update the state of a specific LED."""
        current_time = time.monotonic()
        if state != self.led_states[led_index]:  # Only update if the state has changed
            self.led_objects[led_index].value = state
            self.led_states[led_index] = state
            self.led_timestamps[led_index] = current_time  # Record the time of this change

    def blink_leds(self, interval):
        """Blink LEDs in sequence based on a specified interval."""
        current_time = time.monotonic()
        if current_time - self.previous_millis >= interval:
            self.led_objects[self.current_led].value = False  # Turn off the current LED
            self.current_led = (self.current_led + 1) % len(self.led_objects)
            self.led_objects[self.current_led].value = True   # Turn on the next LED
            self.previous_millis = current_time

    def led_control(self, value, in_min, in_max, led_slow, led_fast):
        """Control LEDs based on an input value."""
        # Map value to interval
        interval_factor = self.map_range(value, in_min, in_max, led_slow, led_fast)
        self.blink_leds(interval_factor)

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        """Maps a value from one range to another, ensuring the result stays within the target range."""
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min