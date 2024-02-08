"""

calibration_system = CalibrationSystem(sensor, dfplayer, oled, servo_ramp)
BAR_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT = 0, 0, 128, 32  # Example OLED dimensions for the progress bar
calibration_system.calibrate_sensor(BAR_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT)

# Process magnetometer data after calibration
processed_value = calibration_system.process_mag_data()
print(f"Processed EMF Value: {processed_value}")

"""

import time
import math

class CalibrationSystem:
    def __init__(self, sensor, dfplayer, oled, servo_ramp, calibration_time=10, emf_min_µT=0, emf_max_µT=1000, emf_min_adc=0, emf_max_adc=1100):
        self.sensor = sensor
        self.dfplayer = dfplayer
        self.oled = oled
        self.servo_ramp = servo_ramp
        self.calibration_time = calibration_time
        self.emf_min_µT = emf_min_µT
        self.emf_max_µT = emf_max_µT
        self.emf_min_adc = emf_min_adc
        self.emf_max_adc = emf_max_adc
        self.baseline = 0

    def calibrate_sensor(self, bar_x, bar_y, bar_width, bar_height):
        self.dfplayer.set_volume(25)
        self.dfplayer.play(track=9)
        
        self.oled.fill(0)
        self.oled.text("Calibrating...", 0, 0, 1)
        self.oled.show()
            
        self.baseline = self.calibrate_baseline(bar_x, bar_y, bar_width, bar_height)
        
        self.oled.fill(0)
        self.oled.text("Calibration Complete", 0, 0, 1)
        self.oled.show()
        
        self.dfplayer.stop()
        self.servo_ramp.park_servo()
        time.sleep(2)  # Pause to show message

    def calibrate_baseline(self, bar_x, bar_y, bar_width, bar_height):
        start_time = time.monotonic()
        readings = []
        
        while time.monotonic() - start_time < self.calibration_time:
            mag_x, mag_y, mag_z = self.sensor.magnetic
            readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
            
            elapsed_time = time.monotonic() - start_time           
            progress = elapsed_time / self.calibration_time  # Progress as a fraction of 0 to 1   
            
            fill_width = int(progress * bar_width)
            servo_progress = progress * 10
            
            self.oled.fill_rect(bar_x, bar_y, fill_width, bar_height, 1)
            self.servo_ramp.calibration_to_servo_pos(servo_progress)
            
            self.servo_ramp.update()
            self.oled.show()
            
            time.sleep(0.1)
            
        return sum(readings) / len(readings)

    def process_mag_data(self):
        mag_x, mag_y, mag_z = self.sensor.magnetic
        emf_strength = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2) - self.baseline
        mapped_emf = self.map_value(emf_strength, self.emf_min_µT, self.emf_max_µT, self.emf_min_adc, self.emf_max_adc)
        processed_value = (mapped_emf ** 3)
        return max(0, min(1100, processed_value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Maps a value from one range to another."""
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min