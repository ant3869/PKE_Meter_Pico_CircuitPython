"""
    GP0:  PLAYER_RX (DFPlayer Mini RX)
    GP1:  PLAYER_TX (DFPlayer Mini TX)
    GP2:  SERVO_PIN (Wing Servo)
    GP4:  I2C_SDA (I2C for LSM9DS1)
    GP5:  I2C_SCL (I2C for LSM9DS1) 
    GP10: BUTTON_A_DECREASE (Variable A Decrease) (14)
    GP11: BUTTON_A_INCREASE (Variable A Increase) (15)
    GP12: BUTTON_B_DECREASE (Variable B Decrease) (16)
    GP13: BUTTON_B_INCREASE (Variable B Increase) (17)
    GP14: LED_INCREASE (Variable increased indicator) (19)
    GP15: LED_DECREASE (Variable decreased ibdicator) (20)
    GP16: LED_WING_1 (Wing/Display LED 1) GREEN
    GP17: LED_WING_2 (Wing/Display LED 2) BLUE
    GP18: LED_WING_3 (Wing/Display LED 3) WHITE
    GP19: LED_WING_4 (Wing/Display LED 4) RED
    GP20: LED_WING_5 (Wing/Display LED 5) YELLOW (R)
    GP21: LED_WING_6 (Wing/Display LED 6) BLACK
    GP22: LED_WING_7 (Wing/Display LED 7) YELLOW
    
"""

import math
import time
import board
import busio
import pwmio
import digitalio
import adafruit_lsm9ds1
from math import floor
from dfplayer import DFPlayer
from lib.servo_ramp import ServoRamp
from lib.led_controller import LEDController
from lib.sensor_manager import SensorManager
from adafruit_ssd1306 import SSD1306_I2C
from digitalio import DigitalInOut, Direction

# --- constants   ----------------------------------------------------------

SERVO_PIN        = board.GP2
SERVO_MIN_POS    = 750
SERVO_MAX_POS    = 1800 

PLAYER_TX        = board.GP0  # board.TX
PLAYER_RX        = board.GP1  # board.RX
PLAYER_BAUD      = 9600
PLAYER_VOL       = 80

I2C_SDA          = board.GP4
I2C_SCL          = board.GP5

LED_WING_1       = board.GP16
LED_WING_2       = board.GP17
LED_WING_3       = board.GP18
LED_WING_4       = board.GP19
LED_WING_5       = board.GP20
LED_WING_6       = board.GP21
LED_WING_7       = board.GP22
LED_PINS    = [LED_WING_1,LED_WING_2, LED_WING_3, LED_WING_4, LED_WING_5, LED_WING_6, LED_WING_7]

LED_INCREASED    = board.GP14    # Window Variable Increased Indicator LED
LED_DECREASED    = board.GP15    # Window Variable Decreased Indicator LED

BUTTON_INCREASE  = board.GP10    # Window Variable Increase Button
BUTTON_DECREASE  = board.GP11    # Window Variable Decrease Button

# --- variables   ----------------------------------------------------------
EMF_MIN_µT       = 0.0           # Minimum expected EMF reading
EMF_MAX_µT       = 100.0         # Maximum expected EMF reading
EMF_MIN_ADC      = 0
EMF_MAX_ADC      = 1100

LED_INTENSITY_MIN = 0.2
LED_INTENSITY_MAX = 1
LED_SLOW          = 0.9           # Slowest blink speed in seconds
LED_FAST          = 0.1           # Fastest blink speed in seconds

SCALE_FACTOR      = 0.1           # Adjust based on your needs
MAX_GRAPH_VALUE   = 1100          # Maximum value after scaling 

OLED_WIDTH        = 128
OLED_HEIGHT       = 64
GRAPH_BASELINE_Y  = OLED_HEIGHT-6 # Baseline for the graph (e.g., half the OLED height)
window_size       = 10            # Default window size for smoothing
current_track     = None

# Global variables to track the debounced button states
increase_pressed = False
decrease_pressed = False

# Additional variables for debounce logic
last_increase_state = False
increase_stable_count = 0
increase_stable_threshold = 5  # Adjust based on your testing

last_decrease_state = False
decrease_stable_count = 0
decrease_stable_threshold = 5  # Adjust based on your testing
# button_pressed = False
# last_button_state = False
# button_press_count = 0
# button_press_threshold = 5  # Number of consecutive checks to confirm button press
# button_press_detected = False  # Flag to mark if a button press was handled

# --- objects   -----------------------------------------------------------
i2c       = busio.I2C(I2C_SCL, I2C_SDA)
uart      = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX, baudrate=PLAYER_BAUD) # Using UART0 on default pins GP0 (TX)and GP1 (RX)

led_increased   = digitalio.DigitalInOut(LED_INCREASED)
led_decreased   = digitalio.DigitalInOut(LED_DECREASED)
led_controller = LEDController(LED_PINS, pwm=False, emf_min=0, emf_max=1023, led_slow=1.0, led_fast=0.1)

button_increase = digitalio.DigitalInOut(BUTTON_INCREASE)      # Direct use as touch-button             
button_decrease = digitalio.DigitalInOut(BUTTON_DECREASE)      # Example pin for decrease

servo_ramp = ServoRamp(servo_pin=SERVO_PIN, min_pulse=750, max_pulse=1800, default_movement_duration=2)

# --- initialization   ----------------------------------------------------
oled           = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor         = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
sensor_manager = SensorManager(sensor, oled, servo_ramp, calibration_time=10)
y_history      = [OLED_HEIGHT // 6] * OLED_WIDTH    # Mid-screen baseline

led_increased.direction   = digitalio.Direction.OUTPUT
led_decreased.direction   = digitalio.Direction.OUTPUT

button_decrease.direction = digitalio.Direction.INPUT
button_decrease.pull      = digitalio.Pull.UP
button_increase.direction = digitalio.Direction.INPUT
button_increase.pull      = digitalio.Pull.UP

sensor_manager.start_calibration()
# start_time = time.monotonic()  # Capture the start time for progress calculation
# 
# while not sensor_manager.is_calibration_com plete():
#     sensor_manager.update_calibration()
#     
#     # Update the servo position as a visual indicator of progress
#     servo_ramp.update_progress(start_time, sensor_manager.calibration_time)
#     
#     # Ensure the servo position is updated smoothly
#     if servo_ramp.is_running():
#         servo_ramp.update()
#     
#     time.sleep(0.1)

try:
    uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
    time.sleep(2)
    dfplayer = DFPlayer(uart=uart)
    dfplayer.set_eq(DFPlayer.EQ_NORMAL)
    dfplayer.set_volume(PLAYER_VOL)
    print("DFPlayer initialized successfully.")
except Exception as e:
    print("Failed to initialize DFPlayer:", e)

time.sleep(1)
dfplayer.play(track=8) # Play startup confirmation sound effect

# --- functions   ----------------------------------------------------
# def calibrate_sensor(sensor, oled, calibration_time=10):
#     oled.fill(0)
#     oled.text("Calibrating...", 0, 0, 1)
#     oled.show()
# 
#     start_time = time.time()
#     readings = []
#     while time.time() - start_time < calibration_time:
#         mag_x, mag_y, mag_z = sensor.magnetic
#         readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
#         time.sleep(0.1)
#     
#     baseline = sum(readings) / len(readings)
#     oled.fill(0)
#     oled.text("Calibration", 0, 0, 1)
#     oled.text("Complete", 0, 10, 1)
#     oled.show()
#     time.sleep(2)  # Pause to show message
#     return baseline

# # Function to playt sound with error handling
# def play_sound_effect(val):
#     try:
#         time.sleep(1)
#         dfplayer.play(track=val)             
#     except Exception as e:
#         print("Failed to play track. Error: ", e)

# Function to update history buffer
def update_history(new_value):
    y_history.append(new_value)
    del y_history[0]

# Function to map from one range to another
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function for a simple moving average
def simple_moving_average(history, window_size=10):
    return [sum(history[max(i - window_size, 0):i+1]) / min(i + 1, window_size) for i in range(len(history))]

# Function to process magnetometer data
def process_mag_data(mx, my, mz, baseline):
    emf_strength = math.sqrt(mx**2 + my**2 + mz**2) - baseline
    mapped_emf = map_value(emf_strength, EMF_MIN_µT, EMF_MAX_µT, EMF_MIN_ADC, EMF_MAX_ADC)
    processed_value = (mapped_emf ** 3)
    return max(0, min(1100, processed_value))

# Function to process and smooth magnetometer data
def smooth_and_process_data(mx, my, mz, baseline, history, window_size=10):
    emf_strength = math.sqrt(mx**2 + my**2 + mz**2)
    relative_strength = emf_strength - baseline
    mapped_emf = map_value(relative_strength, EMF_MIN_µT, EMF_MAX_µT, EMF_MIN_ADC, EMF_MAX_ADC)
    history.append(mapped_emf)
    
    if len(history) > window_size:
        history.pop(0)
    
    smoothed_value = sum(history[-window_size:]) / len(history[-window_size:])
    return max(0, min(1100, smoothed_value))

# def adjust_window_size(inc_button, dec_button, current_window_size, min_size=5, max_size=20):
#     """Adjusts the window size for data smoothing based on button presses."""
#     if inc_button.value:   # Increase window size
#         new_window_size = min(current_window_size + 1, max_size)
#         time.sleep(0.2)    # Debounce delay
#         return new_window_size
#     elif dec_button.value:  # Decrease window size
#         new_window_size = max(current_window_size - 1, min_size)
#         time.sleep(0.2)    # Debounce delay
#         return new_window_size
#     return current_window_size  # No change

# Function to control LEDs
def dynamic_led_control(emf_level):
    intensity = map_value(emf_level, EMF_MIN_ADC, EMF_MAX_ADC, LED_INTENSITY_MIN, LED_INTENSITY_MAX)
    intensity = max(LED_INTENSITY_MIN, min(LED_INTENSITY_MAX, intensity)) # Clamp intensity to [LED_INTENSITY_MIN, LED_INTENSITY_MAX] range
    blink_speed = map_value(intensity, 0, 1, LED_SLOW, LED_FAST)
#    blink_speed = map_value(emf_level, EMF_MIN_ADC, EMF_MAX_ADC, LED_SLOW, LED_FAST)
#    blink_speed = max(LED_SLOW, min(LED_FAST, blink_speed)) # Clamp intensity to [LED_SLOW, LED_FAST] range
    
    duty_cycle = floor(intensity * 65535)        # Calculate duty_cycle within 0-65535 range
    duty_cycle = max(0, min(65535, duty_cycle))  # Additional clamping for safety
    led_wing1_pwm.duty_cycle = duty_cycle

    # Keep the LED on for a fraction of the blink speed, then turn off
    time.sleep(blink_speed / 5)  # On time
    led_wing1_pwm.duty_cycle = 0
    time.sleep(blink_speed / 5)  # Off time

# def play_sound_effect(track_number):
#     global current_track
#     if track_number != current_track:
#         time.sleep(1)
#         dfplayer.play(track=track_number) # The requested track is different from the currently playing track
#         current_track = track_number      # Update the current track

def play_sound_effect(track_number):
    global current_track
    if track_number != current_track:
        try:
            time.sleep(1)  # Evaluate necessity
            dfplayer.play(track=track_number)
            current_track = track_number
            print(f"Playing track {track_number}")
        except Exception as e:
            print(f"Failed to play track {track_number}: {e}")

# Function to decide which track to play based on EMF reading
def play_track_based_on_emf(emf_reading):
    if emf_reading < (EMF_MAX_ADC / 3):
        play_sound_effect(1)  # Attempt to play track 1
    elif emf_reading < (2 * EMF_MAX_ADC / 3):
        play_sound_effect(2)  # Attempt to play track 2
    else:
        play_sound_effect(3)  # Attempt to play track 3

# def classify_emf_reading(reading, baseline):
#     offset = reading - baseline
#     if offset <= 40:
#         return "MINIMUM"
#     elif offset <= 300:
#         return "LOW"
#     elif offset <= 700:
#         return "MODERATE"
#     elif offset <= 1000:
#         return "ELEVATED"
#     else:
#         return "EXTREME"
    
def check_buttons_and_set_flags():
    global increase_pressed, decrease_pressed
    global last_increase_state, increase_stable_count, last_decrease_state, decrease_stable_count

    # Read the current state of the buttons
    current_increase_state = button_increase.value
    current_decrease_state = button_decrease.value  # Assuming you have defined button_decrease

    # Check for increase button
    if current_increase_state == last_increase_state:
        increase_stable_count += 1
    else:
        increase_stable_count = 0  # Reset if the state changed

    if increase_stable_count >= increase_stable_threshold:
        if current_increase_state:  # Assuming True means pressed
            increase_pressed = True
        increase_stable_count = 0  # Reset to avoid repeated triggers

    # Check for decrease button
    if current_decrease_state == last_decrease_state:
        decrease_stable_count += 1
    else:
        decrease_stable_count = 0  # Reset if the state changed

    if decrease_stable_count >= decrease_stable_threshold:
        if current_decrease_state:  # Assuming True means pressed
            decrease_pressed = True
        decrease_stable_count = 0  # Reset to avoid repeated triggers

    # Update the last known states
    last_increase_state = current_increase_state
    last_decrease_state = current_decrease_state

    # Reset flags after they've been used
    if increase_pressed:
        # Perform actions for increase
        increase_pressed = False  # Reset flag after action

    if decrease_pressed:
        # Perform actions for decrease
        decrease_pressed = False  # Reset flag after action

def button_logic_check():
    global button_press_detected
    button_was_pressed = check_button_pressed(button_increase)
    if button_was_pressed:
        print("Button pressed")
        led_increased.value = True  # Turn on LED for feedback
        # Handle button press logic here
        button_press_detected = False  # Reset for next press
        time.sleep(0.2)  # Debounce delay
    else:
        led_increased.value = False

# def adjust_window_size(inc_button, dec_button, current_window_size, min_size=5, max_size=20):
#     """Adjusts the window size for data smoothing based on button presses."""
#     if inc_button.value:    # Increase window size
#         led_increased.value = True
#         led_decreased.value = False
#         new_window_size = min(current_window_size + 1, max_size)
#         time.sleep(0.2)     # Debounce delay
#         return new_window_size
#     elif dec_button.value:  # Decrease window size
#         led_increased.value = False
#         led_decreased.value = True
#         new_window_size = max(current_window_size - 1, min_size)
#         time.sleep(0.2)     # Debounce delay
#         return new_window_size
#     return current_window_size  # No change

def adjust_window_size(current_window_size, min_size=5, max_size=20):
    global increase_pressed, decrease_pressed
    new_window_size = current_window_size
    
    if increase_pressed:
        led_increased.value = True
        led_decreased.value = False
        new_window_size = min(current_window_size + 1, max_size)
        increase_pressed = False  # Reset flag
    elif decrease_pressed:
        led_increased.value = False
        led_decreased.value = True
        new_window_size = max(current_window_size - 1, min_size)
        decrease_pressed = False  # Reset flag
    
    time.sleep(0.2)  # Debounce delay is applied globally here; adjust as needed
    return new_window_size

# Function to draw a line
def draw_line(oled, x0, y0, x1, y1, color):
    x0, y0, x1, y1 = map(int, (x0, y0, x1, y1))
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        oled.pixel(x0, y0, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy

# def draw_graph(oled, latest_reading, processed_value, description, smoothed_history):
def draw_graph(oled, smoothed_value, history, window_size):
    oled.fill(0)
    
    oled.text('EMF: {:.2f}'.format(smoothed_value), 0, 0, 1)
    oled.text('Window: {:.2f}'.format(window_size), 0, 10, 1)
#     description = classify_emf_reading(processed_value, baseline_emf_strength)
#     oled.text('EMF: {:.2f}'.format(processed_value), 0, 0, 1)
#     oled.text('SH: {:.2f}'.format(smoothed_history[-1] if smoothed_history else 0), 0, 10, 1)

    if history:
        # Assume SCALE_FACTOR is defined; adjust it based on your data's range
        for x in range(1, len(history)):
            # Subtract the scaled value from the baseline to make the wave go up
            y0 = GRAPH_BASELINE_Y - (history[x - 1] * SCALE_FACTOR)
            y1 = GRAPH_BASELINE_Y - (history[x] * SCALE_FACTOR)
            
            # Flip the graph direction by adjusting y-coordinates
            y0 = max(0, min(oled.height, y0))
            y1 = max(0, min(oled.height, y1))

            # Ensure coordinates are integers and within bounds
            y0 = int(max(0, min(oled.height - 1, y0)))
            y1 = int(max(0, min(oled.height - 1, y1)))
            
            draw_line(oled, x - 1, y0, x, y1, 1)

    oled.show()

# Main loop


# baseline = calibrate_sensor(sensor, oled, calibration_time=10)
history = []
increase_pressed = False
decrease_pressed = False

while True:
#     check_buttons_and_set_flags()
    window_size = 5
    mag_x, mag_y, mag_z = sensor.magnetic
    smoothed_value = smooth_and_process_data(mag_x, mag_y, mag_z, sensor_manager.baseline, history)
    
#     processed_data, trend, anomalies = sensor_manager.process_sensor_data()
#     # Use smoothed_value for servo control and other actions
#     print(f"EMF Reading: {processed_data}, Type: {type(processed_data)}")
#     
#     if isinstance(processed_data, list) and len(processed_data) == 1:
#          emf_reading = processed_data[0]  # Extract the single value from the list
#      else
#         emf_reading = processed_data
#          
#      if emf_reading < 0
#          emf_reading = 0
#      
    led_controller.adjust_and_control_leds(smoothed_value)
    servo_ramp.emf_to_servo_pos(smoothed_value)
    play_track_based_on_emf(smoothed_value)
    draw_graph(oled, smoothed_value, history, window_size)
# 
# # Ensure processed_data is not None and handle it accordingly
#     if processed_data is not None:
#         # Correctly handle single value or list containing a single value
#         if isinstance(processed_data, list) and len(processed_data) == 1:
#             emf_reading = processed_data[0]  # Extract the single value from the list
#         else:
#             emf_reading = processed_data
# 
#         # Ensure emf_reading is non-negative
#         emf_reading = max(emf_reading, 0)
# 
#         led_controller.adjust_and_control_leds(emf_reading)
#         servo_ramp.emf_to_servo_pos(emf_reading)
#         play_track_based_on_emf(emf_reading)
#         draw_graph(oled, emf_reading, history, window_size)
#     else:
#         print("No processed data available.")
#     
    time.sleep(0.2)  # Adjust delay as needed