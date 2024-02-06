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

    
"""

import board
import digitalio
import pwmio
import time

class LEDController:
    def __init__(self, pins, pwm=False):
        if not isinstance(pins, (list, tuple)):
            raise ValueError("Pins must be a list or tuple")
        self.leds = []
        self.pwm = pwm
        self.emf_min = 0
        self.emf_max = 1100
        self.led_slow = 1.0
        self.led_fast = 0.1
        self.interval = self.led_slow
        self.current_led = 0
        self.previous_millis = time.monotonic()

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
