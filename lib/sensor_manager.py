import time
import math
from emf_processor import EMFProcessor

class SensorManager:
    def __init__(self, sensor, oled, servo_ramp, calibration_time=10):
        self.sensor = sensor
        self.oled = oled
        self.servo_ramp = servo_ramp
        self.emf_processor = EMFProcessor()
        self.calibration_time = calibration_time
        self.readings = []
        self.cal_val = 0
        self.baseline = None

    def start_calibration(self):
        """Initialize calibration process."""
        self.display_message("Calibrating...")
        self.readings = []
        self.baseline = None
        self.servo_ramp.park_servo()
    
    def update_calibration(self):
        """Update calibration with new sensor reading."""
        if not self.is_calibration_complete():
            mag_x, mag_y, mag_z = self.sensor.magnetic
            self.readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
            self.update_servo_position_based_on_calibration_progress()
            
            elapsed_time = time.monotonic() - self.start_time
            if elapsed_time >= self.calibration_time:
                self.finalize_calibration()
    
    def update_servo_position_based_on_calibration_progress(self):
	"""Update servo_ramp.calibration_to_servo_pos with calibration status."""
        self.cal_val = time.monotonic() - self.start_time
        self.servo_ramp.calibration_to_servo_pos(self.cal_val, self.calibration_time)

    def finalize_calibration(self):
    """Finalize the calibration process and compute baseline."""
        self.baseline = sum(self.readings) / len(self.readings)
        self.display_message("Calibration Complete", pause=2)
        self.servo_ramp.park_servo()

    def display_message(self, message, pause=0):
    """Display a message on the OLED."""
        self.oled.fill(0)
        self.oled.text(message, 0, 0, 1)
        self.oled.show()
        if pause > 0:
            time.sleep(pause)

    def is_calibration_complete(self):
    """Check if calibration process is complete."""
        return self.baseline is not None

    def read_sensor_data(self):
    """Reads sensor data and returns it as a list of magnitudes."""
        mx, my, mz = self.sensor.magnetic
        return [[mx, my, mz]]

    def process_sensor_data(self):
    """Process sensor data through EMFProcessor."""
        raw_data = self.read_sensor_data()
        if not raw_data:
            print("No sensor data available.")
            return None, None, None
        return self.emf_processor.process_emf_data(raw_data)