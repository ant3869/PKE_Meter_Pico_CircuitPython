"""

  Project Name: Ghostbusters "Functional" P.K.E Meter
  Author: Ant3869
 
  Description:
    This project is a functional replica of the P.K.E. Meter from the movie Ghostbusters. 
    It not only mimics the iconic design but also serves as a real Electromagnetic Field (EMF) detector.

    Features
        Functioning EMF detection
        Ability to increase/decrease sensitivity/range (using the 4 Direct Contact Touch Switches)
        Wings that raise or lower (reflecting detection: Higher the reading, higher the raise, and vice versa)
        7 LEDs on each wing blink in sequence (same as in the film Ghostbusters)
        LEDs speed increase and decrease (reflecting detection: Higher the reading, faster the speed, and vice versa)
        3 unique SFX while detecting (low, medium, and high readings)

    Components:       
        Raspberry Pi Pico with rp2040 (using Adafruit CircuitPython 8.2.9)
        LSM9DS1 D0F 3-axis Accel/Mag/Gyro
        DFPlayer Mini
        5V 2A Recharge/Boost Converter Power Module
        SURPASS 9g S0009M Servo Metal Gear Motor
        iPhone 5 Loudspeaker
        33 LEDs (7 on each 'wing', 14 on display screen, and 5 indicators on front panel)
        16mm Variable Resistor Potentiometer
        4 Direct Contact Touch Switches
        3.7v Lithium Rechargeable Battery
        
    Hardware Connections (pin configurations):
         GP0: PLAYER_RX (DFPlayer Mini RX) (1)
         GP1: PLAYER_TX (DFPlayer Mini TX) (2)
         GP2: SERVO_PIN (Wing Servo)       (4)
         GP4: I2C_SDA (I2C SDA connection for LSM9DS1) (6)
         GP5: I2C_SCL (I2C SCL connection for LSM9DS1) (7)
        GP10: BUTTON_GAIN_DECREASE (Decrease signal gain value) (14)
        GP11: BUTTON_GAIN_INCREASE (Increase signal gain value) (15)
        GP12: BUTTON_SENSITIVITY_DECREASE (Decrease signal detetection sensitivity value) (16)
        GP13: BUTTON_SENSITIVITY_INCREASE (Increase signal detetection sensitivity value) (17)
        GP14: LED_INCREASE (Signal adjustment increased indicator LED) (19)
        GP15: LED_DECREASE (Signal adjustment decreased indicator LED) (20)
        GP16: LED_WING_1 (Wing/Display LED 1) GREEN      (21)
        GP17: LED_WING_2 (Wing/Display LED 2) BLUE       (22)
        GP18: LED_WING_3 (Wing/Display LED 3) WHITE      (24)
        GP19: LED_WING_4 (Wing/Display LED 4) RED        (25)
        GP20: LED_WING_5 (Wing/Display LED 5) YELLOW (R) (26)
        GP21: LED_WING_6 (Wing/Display LED 6) BLACK      (27)
        GP22: LED_WING_7 (Wing/Display LED 7) YELLOW     (29)
    
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
from lib.button_manager import ButtonManager
from adafruit_ssd1306 import SSD1306_I2C
from digitalio import DigitalInOut, Direction

# --- pin decleration   ----------------------------------------------------------

SERVO_PIN        = board.GP2

PLAYER_TX        = board.GP0  # board.TX
PLAYER_RX        = board.GP1  # board.RX

I2C_SDA          = board.GP4
I2C_SCL          = board.GP5

LED_WING_1       = board.GP16
LED_WING_2       = board.GP17
LED_WING_3       = board.GP18
LED_WING_4       = board.GP19
LED_WING_5       = board.GP20
LED_WING_6       = board.GP21
LED_WING_7       = board.GP22

# --- constants/variables   ----------------------------------------------------------

player_vol       = 100
current_track     = None

emf_min_µt       = 0.0           # Minimum expected EMF reading
emf_max_µt       = 100.0         # Maximum expected EMF reading
EMF_MIN_ADC      = 0
EMF_MAX_ADC      = 1100

LED_INTENSITY_MIN = 0.2
LED_INTENSITY_MAX = 1
LED_SLOW          = 0.9           # Slowest blink speed in seconds
LED_FAST          = 0.1           # Fastest blink speed in seconds
LED_PINS          = [LED_WING_1,LED_WING_2, LED_WING_3, LED_WING_4, LED_WING_5, LED_WING_6, LED_WING_7]

SCALE_FACTOR      = 0.1           # Adjust based on your needs
MAX_GRAPH_VALUE   = 1100          # Maximum value after scaling 

OLED_WIDTH        = 128
OLED_HEIGHT       = 64
GRAPH_BASELINE_Y  = OLED_HEIGHT-6 # Baseline for the graph (e.g., half the OLED height)

calibration_time  = 10
window_size       = 10            # Default window size for smoothing

# --- objects   -----------------------------------------------------------

i2c            = busio.I2C(I2C_SCL, I2C_SDA)
uart           = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX) # Using UART0 on default pins GP0 (TX)and GP1 (RX)
oled           = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor         = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
led_controller = LEDController(LED_PINS, pwm=False)
servo_ramp     = ServoRamp(servo_pin=SERVO_PIN)
sensor_manager = SensorManager(sensor, oled, servo_ramp)
button_manager = ButtonManager()

# --- initialization   ----------------------------------------------------

y_history = [OLED_HEIGHT // 6] * OLED_WIDTH    

try:
    uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
    time.sleep(2)
    dfplayer = DFPlayer(uart=uart)
    print("DFPlayer initialized successfully.")
except Exception as e:
    print("Failed to initialize DFPlayer:", e)

# --- functions   ----------------------------------------------------

# Function to update history buffer
def update_history(new_value):
    y_history.append(new_value)
    del y_history[0]

# Function to map from one range to another
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function error checking track playing
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

def main_loop():
    global history  # If history is managed globally
    while True:
        check_and_handle_buttons()
        emf_reading = sensor_manager.process_and_evaluate_emf_data()

        led_controller.map_emf_to_speed(emf_reading)
        servo_ramp.emf_to_servo_pos(emf_reading)
        play_track_based_on_emf(emf_reading)
       
        if len(history) > window_size:
            history.pop(0)
            
        draw_graph(oled, emf_reading, history, window_size)
        time.sleep(0.2)

if __name__ == "__main__":
    history = []
    window_size = 10
    val = dfplayer.num_files(folder=None,media=DFPlayer.MEDIA_SD)
    print(val)
    sensor_manager.perform_calibration()
    dfplayer.play(track=8)
    time.sleep(2)
    main_loop()