# sensor_manager.py

"""

from sensor_manager import SensorManager
sensor_manager = SensorManager(sensor, oled, servo_ramp, calibration_time=10)

# Start calibration
sensor_manager.start_calibration()

while not sensor_manager.is_calibration_complete():
    sensor_manager.update_calibration()
    # Optionally update a visual indicator (e.g., servo position) here
    time.sleep(0.1)  # Adjust based on desired calibration update frequency

# Calibration is complete, proceed with normal operation
while True:
    smoothed_data, anomalies = sensor_manager.process_data()
    # Use the processed data
        
"""

import time
import math
from emf_processor import EMFProcessor

class SensorManager:
    def __init__(self, sensor, oled, servo_ramp, calibration_time=10):
        self.sensor = sensor
        self.oled = oled
        self.servo_ramp = servo_ramp         
        self.calibration_time = calibration_time
        self.emf_processor = EMFProcessor()
        self.raw_readings = { 'x': [], 'y': [], 'z': [] }
        self.calibration_complete = False
        self.start_time = 0

    def start_calibration(self):
        """Initialize calibration process."""        
        self.display_message("Calibrating ...")    
        self.start_time = time.monotonic()
        self.servo_ramp.park_servo()
                
    def time_elapsed(self):      
        elapsed_time = time.monotonic() - self.start_time
        return elapsed_time >= self.calibration_time

    def update_calibration(self):
        """Update calibration with new sensor reading."""
        if not self.calibration_complete:
            sensor_data = self.read_sensor_data()
            if sensor_data is None:  
                return
            
            self.raw_readings['x'].append(sensor_data['x'])
            self.raw_readings['y'].append(sensor_data['y'])
            self.raw_readings['z'].append(sensor_data['z'])
            
            self.update_servo_calibration_progress()
            
            if self.time_elapsed():
                self.finalize_calibration()
    
    def finalize_calibration(self):
        """Finalize the calibration process and compute baseline."""       
        scale_x, scale_y, scale_z = 1, 1, 1
        
        offset_x = sum(self.raw_readings['x']) / len(self.raw_readings['x'])
        offset_y = sum(self.raw_readings['y']) / len(self.raw_readings['y'])
        offset_z = sum(self.raw_readings['z']) / len(self.raw_readings['z'])
                
        self.emf_processor.calibration_params = {
        'offset_x': -offset_x, 'scale_x': 1,
        'offset_y': -offset_y, 'scale_y': 1,
        'offset_z': -offset_z, 'scale_z': 1,
        }
                        
        self.servo_ramp.park_servo()
        self.calibration_complete = True
        self.display_message("Calibration Complete", pause=2)
        
    def update_servo_calibration_progress(self):
        """Update servo position based on calibration progress."""
        cal_val = time.monotonic() - self.start_time
        self.servo_ramp.calibration_to_servo_pos(cal_val, self.calibration_time)

    def adjust_sensitivity(self, increase=True):    # Adjust sensitivity or gain based on user input
        if increase:
            self.emf_processor.sensitivity += 0.1
        else:
            self.emf_processor.sensitivity -= 0.1
        self.emf_processor.sensitivity = max(0.1, min(self.emf_processor.sensitivity, 2.0))  # Clamping

    def display_message(self, message, pause=0):
        """Display a message on the OLED."""
        self.oled.fill(0)
        self.oled.text(message, 0, 0, 1)
        self.oled.show()
        if pause > 0:
            time.sleep(pause)

    def read_sensor_data(self):
        """Reads sensor data and returns it as a list of magnitudes."""
        mx, my, mz = self.sensor.magnetic
        if None in [mx, my, mz]:  # Check if any value is None
            print("Failed to read sensor data.")
            return None
        return {'x': mx, 'y': my, 'z': mz}  # Return a list of a list to keep data structure consistent

    def process_sensor_data(self):
        """Process sensor data through EMFProcessor."""
        raw_data = self.read_sensor_data()
        if not raw_data:  # Additional check to ensure we have data
            return None
        processed_data = self.emf_processor.process_emf_data(raw_data)
        return processed_data
