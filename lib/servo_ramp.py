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
        self.min_angle = 0
        self.max_angle = 180
        self.current_pos = 0
        self.target_pos = None
        self.start_time = None
        self.EMF_MIN = 0
        self.EMF_MAX = 1100
        self.reset_movement()     

    def reset_movement(self):
        self.target_pos = self.min_angle
        self.start_time = time.monotonic()
        self.movement_duration = self.default_movement_duration
        self.current_pos = self.min_angle
        self.servo.angle = self.current_pos
        
    def go(self, target_angle, movement_duration=None):
        """Initiates movement to a target angle over a specified duration."""
        if movement_duration is None:
            movement_duration = self.default_movement_duration
        self.target_pos = target_angle
        self.start_time = time.monotonic()
        self.movement_duration = movement_duration
        self.current_pos = self.servo.angle if self.servo.angle is not None else self.min_angle
        
    def update(self):
        """Updates the servo's position if a movement is in progress."""
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
        """Checks if there is a movement in progress."""
        return self.start_time is not None and (time.monotonic() - self.start_time) < self.movement_duration

    def calibration_to_servo_pos(self, cal_time, cal_max=10):
        """Maps calibration progress to a servo position."""
        angle = self.map_value_to_angle(cal_time, 0, cal_max)
        self.go(target_angle=angle, movement_duration=None)  # Use default duration
        
    def emf_to_servo_pos(self, emf_reading):
        """Maps an EMF reading to a servo position and moves the servo."""
        clamped_emf = max(self.EMF_MIN, min(self.EMF_MAX, emf_reading))
        target_angle = self.map_value_to_angle(clamped_emf, self.EMF_MIN, self.EMF_MAX)
        self.go(target_angle=target_angle, movement_duration=None)  # Use default duration
        
    def map_value_to_angle(self, value, in_min, in_max):
        """Maps a value from one range to another."""
        out_min, out_max = self.min_angle, self.max_angle
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def park_servo(self):
        """Move the servo to the parked position after calibration."""
        self.go(self.min_angle)  # Or any other specific parked position

#     def go(self, target_angle, movement_duration=None):
#         if movement_duration is None:
#             movement_duration = self.default_movement_duration
#         self.target_pos = target_angle
#         self.start_time = time.monotonic()
#         self.current_pos = self.servo.angle if self.servo.angle is not None else self.min_angle
# 
#     def update(self):
#         if not self.is_running():
#             return
#         elapsed_time = time.monotonic() - self.start_time
#         if elapsed_time >= self.movement_duration:
#             self.servo.angle = self.target_pos
#             self.reset_movement()
#         else:
#             fraction_complete = elapsed_time / self.movement_duration
#             new_angle = self.current_pos + (self.target_pos - self.current_pos) * fraction_complete
#             self.servo.angle = new_angle
# 
#     def is_running(self):
#         return self.start_time is not None and (time.monotonic() - self.start_time) < self.movement_duration
# 
#     def map_value_to_angle(self, value, in_min, in_max):
#         out_min, out_max = self.min_angle, self.max_angle
#         value = max(min(value, in_max), in_min)
#         return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
# 
#     def calibration_to_servo_pos(self, cal_time, cal_max):
#         self.apply_mapped_value(cal_time, 0, cal_max, self.default_movement_duration)
# 
#     def apply_mapped_value(self, value, in_min, in_max, duration=None):
#         angle = self.map_value_to_angle(value, in_min, in_max)
#         self.go(target_angle=angle, movement_duration=duration or self.default_movement_duration)
#         
#     def park_servo(self):
#         self.apply_mapped_value(0, self.min_angle, self.max_angle)
        
#     def calibration_to_servo_pos(self, elapsed_time, total_calibration_time):
#         """Interpolate servo position based on calibration progress."""
#         progress = elapsed_time / total_calibration_time
#         target_angle = self.min_angle + progress * (self.max_angle - self.min_angle)
#         self.move_servo_to(target_angle)
        
#     def move_servo_to(self, target_angle):
#         """Move the servo to a specified angle smoothly."""
#         # Here you can implement logic to move the servo smoothly
#         # For simplicity, we'll just set the servo to the target angle directly
#         self.servo.angle = target_angle
#         self.current_pos = target_angle

    def initialize_servo(self):
        for angle in [self.min_angle, 45, 90, 135, self.max_angle]:
            self.go(target_angle=angle, movement_duration=0.5)
            while self.is_running():
                self.update()
                time.sleep(0.01)
            self.park_servo()
            time.sleep(2)