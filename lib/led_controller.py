"""

# Example usage
led_pin = board.D13  # Use the board's built-in LED if available

# For a basic LED
basic_led = LEDController(led_pin, pwm=False)
basic_led.blink(on_time=1, off_time=1, cycles=3)

# For a PWM-controlled LED (brightness control)
pwm_led = LEDController(led_pin, pwm=True)
pwm_led.set_brightness(0.5)  # Set to 50% brightness
time.sleep(2)
pwm_led.blink(on_time=1, off_time=1, cycles=3)
pwm_led.set_brightness(0.1)  # Dim to 10% brightness
time.sleep(2)
pwm_led.off()

# Assuming EMF reading logic is elsewhere and provides current_emf_reading
current_emf_reading = 512  # Example EMF reading

# Define LED pins (adjust according to your setup)
LED_PINS = [board.D2, board.D3, board.D4]  # Example pins for digital LEDs

# Initialize the LED controller
led_controller = LEDController(LED_PINS, pwm=False, emf_min=0, emf_max=1023, led_slow=1.0, led_fast=0.1)

# Map the current EMF reading to LED blink speed
led_controller.map_emf_to_speed(current_emf_reading)

# Use a loop to control LEDs based on the mapped speed
while True:
    led_controller.control_leds()
    time.sleep(0.01)  # Small delay to prevent hogging the CPU

led_objects
current_led
interval
previous_millis
led_states
led_timestamps 

"""

import board
import digitalio
import pwmio
import time

class LEDController:
    def __init__(self, pins, pwm=False):
        if not isinstance(pins, (list, tuple)):
            raise ValueError("Pins must be a list or tuple")
        self.led_objects = []
        self.pwm = pwm
        self.emf_min = 0
        self.emf_max = 1100
        self.led_slow = 1.25
        self.led_fast = 0.025
        self.interval = self.led_slow
        self.current_led = 0
        self.previous_millis = time.monotonic()
        
    # intialize LED array 
    def setup_leds(self, pins):
        self.led_objects = []  # This will hold the DigitalInOut objects for the LEDs
        for pin in pins:
            led = digitalio.DigitalInOut(pin)
            led.direction = digitalio.Direction.OUTPUT
            led_objects.append(led)
        return self.led_objects

    # blink LEDs function
    def blink_leds(self, led_objects, interval):
        global current_led, previous_millis
        current_time = time.monotonic()
        if current_time - previous_millis >= interval:
            self.led_objects[self.current_led].value = False  # Turn off the current LED
            self.current_led = (self.current_led + 1) % len(self.led_objects)
            self.led_objects[self.current_led].value = True   # Turn on the next LED
            previous_millis = current_time
            
    # control LEDs based on EMF reading
    def led_control(self, emf_reading, led_objects):
        emf_range = EMF_MAX_ADC - EMF_MIN_ADC
        interval_factor = ((emf_reading - EMF_MIN_ADC) / emf_range) ** 0.2  # Square root for non-linear mapping
        speed_range = led_slow - led_fast
        interval = led_slow - (interval_factor * speed_range)
        self.blink_leds(self.led_objects, self.interval)

    # Update the state of a specific LED and record the time of this update.
    def update_led_state(self, led_index, state):
        global led_states, led_timestamps
        current_time = time.monotonic()
        if state != led_states[led_index]:  # Only update if the state has changed
            self.led_objects[led_index].value = state
            led_states[led_index] = state
            led_timestamps[led_index] = current_time  # Record the time of this change

"""

    for pin in pins:
        if pwm:
            led = pwmio.PWMOut(pin, frequency=5000, duty_cycle=0)
        else:
            led = digitalio.DigitalInOut(pin)
            led.direction = digitalio.Direction.OUTPUT
        self.leds.append(led)

    def map_emf_to_speed(self, emf_reading):
        emf_reading = max(self.emf_min, min(self.emf_max, emf_reading))  # Clamp EMF reading
        emf_range = self.emf_max - self.emf_min
        speed_range = self.led_slow - self.led_fast
        self.interval = (((emf_reading - self.emf_min) * speed_range) / emf_range) + self.led_fast


    def control_leds(self, emf_reading):
        current_time = time.monotonic()
        if current_time - self.previous_millis >= self.interval:
            self.toggle_led(self.leds[self.current_led], False)  # Turn off the current LED
            self.current_led = (self.current_led + 1) % len(self.leds)
            self.toggle_led(self.leds[self.current_led], True)  # Turn on the next LED
            self.previous_millis = current_time

    def toggle_led(self, led, state):
        if self.pwm:
            led.duty_cycle = 65535 if state else 0
        else:
            led.value = state

    def on(self):
        for led in self.leds:
            self.toggle_led(led, True)

    def off(self):
        for led in self.leds:
            self.toggle_led(led, False)
            
    def blink(self, on_time=0.5, off_time=0.5, cycles=1):
        for _ in range(cycles):
            self.on()
            time.sleep(on_time)
            self.off()
            time.sleep(off_time)
      
    def set_brightness(self, brightness):
        if not self.pwm:
            print("PWM not enabled. Brightness control is not available.")
            return
        brightness = max(0, min(1, brightness))  # Clamp brightness value
        for led in self.leds:
            led.duty_cycle = int(brightness * 65535)
            
"""
