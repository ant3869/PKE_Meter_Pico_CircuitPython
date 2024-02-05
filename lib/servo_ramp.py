"""

import board
import time
from servo_ramp import ServoRamp

# Initialize the ServoRamp
SERVO_PIN = board.D9  # Adjust based on your setup
servo_ramp = ServoRamp(servo_pin=SERVO_PIN, min_pulse=750, max_pulse=1800, default_movement_duration=2)

# Example: Move the servo to 90 degrees (midpoint) over 2 seconds
servo_ramp.go(target_angle=90, movement_duration=2)

# Continuously update the servo position until the movement is complete
while servo_ramp.is_running():
    servo_ramp.update()
    time.sleep(0.01)  # Small delay to avoid hogging the CPU
    
    
# Inside SensorManager's calibration-related method
current_calibration_time = # Obtain current calibration time somehow
servo_ramp.calibration_to_servo_pos(current_calibration_time, cal_max=total_calibration_duration)

    
"""

import time
import board
import pwmio
from adafruit_motor import servo

class ServoRamp:
    def __init__(self, servo_pin, min_pulse=750, max_pulse=1800, default_movement_duration=1):
        self.servo_pwm = pwmio.PWMOut(servo_pin, frequency=50)
        self.servo = servo.Servo(self.servo_pwm, min_pulse=min_pulse, max_pulse=max_pulse)
        self.default_movement_duration = default_movement_duration
        self.reset_movement()

    def reset_movement(self):
        self.target_pos = None
        self.start_time = None
        self.current_pos = 0
        self.movement_duration = None

    def go(self, target_angle, movement_duration=None):
        self.target_pos = target_angle
        self.start_time = time.monotonic()
        self.movement_duration = movement_duration or self.default_movement_duration
        self.current_pos = self.servo.angle if self.servo.angle is not None else 0

    def update(self):
        if not self.is_running():
            return
        elapsed_time = time.monotonic() - self.start_time
        if elapsed_time >= self.movement_duration:
            self.servo.angle = self.target_pos
            self.reset_movement()
        else:
            fraction_complete = elapsed_time / self.movement_duration
            new_angle = self.current_pos + (self.target_pos - self.current_pos) * fraction_complete
            self.servo.angle = new_angle

    def is_running(self):
        return self.start_time is not None and (time.monotonic() - self.start_time) < self.movement_duration

    def map_value_to_angle(self, value, in_min, in_max):
        out_min, out_max = 0, 180
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

  def calibration_to_servo_pos(self, cal_time, cal_max=10):
        self.apply_mapped_value(cal_time, 0, cal_max, self.default_movement_duration)

    def apply_mapped_value(self, value, in_min, in_max, duration=None):
        angle = self.map_value_to_angle(value, in_min, in_max)
        self.go(target_angle=angle, movement_duration=duration or self.default_movement_duration)
        
    def park_servo(self):
        self.apply_mapped_value(0, 0, 180)

    def initialize_servo(self):
        for angle in [0, 45, 90, 135, 180, 0]:
            self.go(target_angle=angle, movement_duration=0.5)
            while self.is_running():
                self.update()
                time.sleep(0.01)
            time.sleep(2)