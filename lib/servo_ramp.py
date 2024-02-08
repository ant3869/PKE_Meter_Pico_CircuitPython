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
    def __init__(self, servo_pin):
        self.servo_pwm = pwmio.PWMOut(servo_pin, frequency=50)
        self.servo = servo.Servo(self.servo_pwm, min_pulse=750, max_pulse=1800)
        self.default_movement_duration = 1
        self.movement_duration = self.default_movement_duration
        self.EMF_MIN     = 0
        self.EMF_MAX     = 1100
        self.min_angle   = 0
        self.max_angle   = 180
        self.start_time  = None
        self.target_pos  = self.min_angle
        self.current_pos = self.min_angle      
        self.servo.angle = self.current_pos

    def go(self, target_angle, movement_duration=None):
        """Initiates movement to a target angle over a specified duration."""
        self.target_pos = target_angle
        self.movement_duration = movement_duration if movement_duration else self.default_movement_duration
        if self.start_time is None:
            self.start_time = time.monotonic()
                
    def map_value_to_angle(self, value, in_min, in_max):
        """Maps a value from one range to another."""
        out_min, out_max = self.min_angle, self.max_angle
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min   

#     @staticmethod
#     def map_range(value, in_min, in_max, out_min, out_max):
#         """Maps a value from one range to another."""
#         value = max(min(value, in_max), in_min)
#         return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def is_running(self):
        """Check if the servo is currently moving."""
        return self.current_pos is not self.target_pos

    def update(self):
        """Updates the servo position based on the ramp calculation."""
        if self.target_pos is None or self.start_time is None:
            return
        
        elapsed_time = time.monotonic() - self.start_time
        if elapsed_time <= self.movement_duration:
            fraction_complete = elapsed_time / self.movement_duration
            new_angle = (self.target_pos - self.current_pos) * fraction_complete + self.current_pos
            self.servo.angle = new_angle
        else:
            self.servo.angle = self.target_pos
            self.current_pos = self.target_pos  # Ensure current position is updated
            self.start_time = None  # Reset for the next movement
            
    def wait_for_movement_completion(self):
        """Waits for the current servo movement to complete before continuing."""
        while self.is_running():
            self.update()
            time.sleep(0.01)  # Small delay to avoid hogging the CPU
            
    def park_servo(self):
        """Move the servo to the parked position."""        
        self.start_time = time.monotonic()
        self.go(target_angle=self.min_angle)
        self.wait_for_movement_completion()

    def calibration_to_servo_pos(self, progress):
        """Maps an EMF reading to a servo position and moves the servo."""
        mapped_angle = self.map_value_to_angle(progress, 0, 10)
        servo_angle = max(self.min_angle, min(self.max_angle, mapped_angle))
        self.go(target_angle=servo_angle, movement_duration=0.1)
        
#     def calibration_to_servo_pos_(self, progress, total_calibration_time=10):
#         """Maps calibration progress to a servo position and moves the servo."""
#         mapped_angle = self.map_value_to_angle(progress, 0, total_calibration_time)
#         remaining_time = total_calibration_time - progress
#         movement_duration = max(0.1, remaining_time)  # Ensure at least a minimal movement duration
#         self.go(target_angle=mapped_angle, movement_duration=movement_duration)
        
    def emf_to_servo_pos(self, emf_reading):
        """Maps an EMF reading to a servo position and moves the servo."""
        clamped_emf = max(self.EMF_MIN, min(self.EMF_MAX, emf_reading))
        mapped = self.map_value_to_angle(clamped_emf, self.EMF_MIN, self.EMF_MAX)
        clamped = max(self.min_angle, min(self.max_angle, mapped))
        self.start_time = time.monotonic()
        self.go(target_angle=clamped, movement_duration=1)
        self.wait_for_movement_completion()

    def initialize_servo(self):
        for angle in [self.min_angle, 45, 90, 135, self.max_angle]:
            self.go(target_angle=angle, movement_duration=0.5)
            while self.is_running():
                self.update()
                time.sleep(0.01)
            self.park_servo()
            time.sleep(2)
