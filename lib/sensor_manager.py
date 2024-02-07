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
    def __init__(self, sensor, oled, servo_ramp):
        self.sensor = sensor
        self.oled = oled
        self.servo_ramp = servo_ramp         
        self.calibration_time = 10
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
        """Check if calibration time has elapsed."""
        return time.monotonic() - self.start_time >= self.calibration_time

    def update_calibration(self):
        """Update calibration with new sensor reading."""
        if not self.calibration_complete:
            sensor_data = self.read_sensor_data()
            if sensor_data is None:
                return
            
            if all(isinstance(value, (int, float)) for value in sensor_data.values()):
                self.raw_readings['x'].append(sensor_data['x'])
                self.raw_readings['y'].append(sensor_data['y'])
                self.raw_readings['z'].append(sensor_data['z'])

            self.update_servo_calibration_progress()

            if self.time_elapsed():
                self.finalize_calibration()
                
    def finalize_calibration(self):
        """Finalize the calibration process and compute baseline."""
        if not self.raw_readings['x']:
            self.display_message("Calibration Failed: No Data", pause=2)
            return

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
        
    def perform_calibration(self):
        """Perform the entire calibration process."""
        self.start_calibration()
        while not self.calibration_complete:
            self.update_calibration()
            time.sleep(0.1)  # Regular interval check for completion    
    
    def update_servo_calibration_progress(self):
        """Update servo position based on calibration progress."""
        cal_val = time.monotonic() - self.start_time
        self.servo_ramp.calibration_to_servo_pos(cal_val, self.calibration_time)

    def display_message(self, message, pause=0):
        """Display a message on the OLED."""
        self.oled.fill(0)
        self.oled.text(message, 0, 0, 1)
        self.oled.show()
        if pause > 0:
            time.sleep(pause)

    def read_sensor_data(self):
        """Reads sensor data and returns it as a dictionary."""
        try:
            mx, my, mz = self.sensor.magnetic
            if None in [mx, my, mz]:
                raise ValueError("Failed to read sensor data.")
        except Exception as e:
            print(e)
            return None
        return {'x': mx, 'y': my, 'z': mz}

    def process_sensor_data(self):
        """Process sensor data through EMFProcessor."""
        raw_data = [self.read_sensor_data()]
        if not raw_data or raw_data[0] is None:
            print("No data to process.")
            return None
        
        formatted_data = [[d['x'], d['y'], d['z']] for d in raw_data if isinstance(d, dict) and None not in d.values()]
        
        if not formatted_data:
            print("Data format error or empty data.")
            return None

        processed_data = self.emf_processor.prepare_data(formatted_data)
        return processed_data
    
    def process_and_evaluate_emf_data(self):
        """Processes sensor data and evaluates it to get a composite EMF score."""
        processed_data = self.process_sensor_data()
        if processed_data is None:
            print("No data to evaluate.")
            return 0  # Return the lowest possible score as a fallback

        composite_score = self.emf_processor.get_composite_emf_score(processed_data)
        return composite_score