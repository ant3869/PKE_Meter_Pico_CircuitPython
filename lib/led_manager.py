import time
import board
import digitalio

class LEDManager:
    def __init__(self, led_pins):
        self.led_objects = [self._setup_led(pin) for pin in led_pins]
        self.current_led = -1
        self.EMF_MIN_ADC = 0
        self.EMF_MAX_ADC = 1100
        self.led_slow = 1.25
        self.led_fast = 0.00001
        self.interval_factors = self.precompute_intervals()

    def _setup_led(self, pin):
        led = digitalio.DigitalInOut(pin)
        led.direction = digitalio.Direction.OUTPUT
        return led

    def precompute_intervals(self):
        intervals = []
        for value in range(self.EMF_MIN_ADC, self.EMF_MAX_ADC+1):
            intervals.append(self.map_range(value, self.EMF_MIN_ADC, self.EMF_MAX_ADC, self.led_slow, self.led_fast))
        return intervals

    def led_control(self, value):
        # Ensure value is within the expected range
        value = max(0, min(int(value), self.EMF_MAX_ADC))  # Convert value to int and ensure it's within range
        interval_factor = self.interval_factors[value]
        current_time = time.monotonic()
        if 'last_update_time' not in self.__dict__:
            self.last_update_time = current_time
        if current_time - self.last_update_time >= interval_factor:
            self.current_led = (self.current_led + 1) % len(self.led_objects)
            for i, led in enumerate(self.led_objects):
                led.value = (i == self.current_led)
            self.last_update_time = current_time

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
