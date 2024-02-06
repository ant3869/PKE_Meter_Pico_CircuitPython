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
        Raspberry Pi Pico (3.3V)
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
        GP0:  PLAYER_RX (DFPlayer Mini RX)
        GP1:  PLAYER_TX (DFPlayer Mini TX)
        GP2:  SERVO_PIN (Wing Servo)
        GP4:  I2C_SDA (I2C SDA connection for LSM9DS1)
        GP5:  I2C_SCL (I2C SCL connection for LSM9DS1) 
        GP10: BUTTON_SENSITIVITY_DECREASE (Decrease signal detetection sensitivity value) (14)
        GP11: BUTTON_SENSITIVITY_INCREASE (Increase signal detetection sensitivity value) (15)
        GP12: BUTTON_GAIN_DECREASE (Decrease signal gain value) (16)
        GP13: BUTTON_GAIN_INCREASE (Increase signal gain value) (17)
        GP14: LED_INCREASE (Signal adjustment increased indicator LED) (19)
        GP15: LED_DECREASE (Signal adjustment decreased indicator LED) (20)
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

LED_INCREASED    = board.GP14  # Button Variable Increased Indicator LED
LED_DECREASED    = board.GP15  # Button Variable Decreased Indicator LED

BUTTON_SENSITIVITY_DECREASE  = board.GP10  # Window Variable Increase Button
BUTTON_SENSITIVITY_INCREASE  = board.GP11  # Window Variable Decrease Button
BUTTON_GAIN_DECREASE  = board.GP12    
BUTTON_GAIN_INCREASE  = board.GP13    

# --- constants/variables   ----------------------------------------------------------
player_baud      = 9600
player_vol       = 80
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

# sensitivity
sensitivity = 0.5  # Starting sensitivity, adjust as needed
sensitivity_min = 0.1  # Minimum sensitivity
sensitivity_max = 1.0  # Maximum sensitivity
sensitivity_step = 0.1  # Sensitivity adjustment step

# gain
gain = 0.5  # Starting sensitivity, adjust as needed
gain_min = 0.1  # Minimum sensitivity
gain_max = 1.0  # Maximum sensitivity
gain_step = 0.1  # Sensitivity adjustment step

# button states
last_increase_state = True
last_decrease_state = True
last_debounce_time = 0
debounce_delay = 0.05  # 50 milliseconds for debounce

# --- objects   -----------------------------------------------------------
i2c       = busio.I2C(I2C_SCL, I2C_SDA)
uart      = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX, baudrate=player_baud) # Using UART0 on default pins GP0 (TX)and GP1 (RX)

led_increased   = digitalio.DigitalInOut(LED_INCREASED)
led_decreased   = digitalio.DigitalInOut(LED_DECREASED)

button_increase = digitalio.DigitalInOut(BUTTON_SENSITIVITY_INCREASE)      # Direct use as touch-button             
button_decrease = digitalio.DigitalInOut(BUTTON_SENSITIVITY_DECREASE)      # Example pin for decrease

oled           = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor         = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
led_controller = LEDController(LED_PINS, pwm=False)
servo_ramp     = ServoRamp(servo_pin=SERVO_PIN)
sensor_manager = SensorManager(sensor, oled, servo_ramp)

# --- initialization   ----------------------------------------------------
y_history = [OLED_HEIGHT // 6] * OLED_WIDTH    

led_increased.direction   = digitalio.Direction.OUTPUT
led_decreased.direction   = digitalio.Direction.OUTPUT

button_decrease.direction = digitalio.Direction.INPUT
button_decrease.pull      = digitalio.Pull.UP
button_increase.direction = digitalio.Direction.INPUT
button_increase.pull      = digitalio.Pull.UP

try:
    uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
    time.sleep(2)
    dfplayer = DFPlayer(uart=uart)
    dfplayer.set_eq(DFPlayer.EQ_NORMAL)
    dfplayer.set_volume(player_vol)
    print("DFPlayer initialized successfully.")
except Exception as e:
    print("Failed to initialize DFPlayer:", e)

time.sleep(1)
dfplayer.play(track=8) # Play startup confirmation sound effect

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
        
def check_and_handle_buttons():
    global last_increase_state, last_decrease_state, last_debounce_time
    current_increase_state = button_increase.value
    current_decrease_state = button_decrease.value
    current_time = time.monotonic()

    # Check if button states have changed to initiate a debounce period
    if current_increase_state != last_increase_state:
        last_debounce_time = current_time

    if current_decrease_state != last_decrease_state:
        last_debounce_time = current_time

    # After the debounce delay, process the button state if it has changed
    if (current_time - last_debounce_time) > debounce_delay:
        # If button state is stable and has changed, handle the button action
        if current_increase_state != last_increase_state and not current_increase_state:
            adjust_sensitivity(True)

        if current_decrease_state != last_decrease_state and not current_decrease_state:
            adjust_sensitivity(False)

    # Update the last button states
    last_increase_state = current_increase_state
    last_decrease_state = current_decrease_state

def adjust_sensitivity(is_increase):
    global sensitivity
    if is_increase:
        sensitivity = min(sensitivity + sensitivity_step, sensitivity_max)
        led_increased.value = True
        time.sleep(0.2)  # Short feedback duration
        led_increased.value = False
    else:
        sensitivity = max(sensitivity - sensitivity_step, sensitivity_min)
        led_decreased.value = True
        time.sleep(0.2)  # Short feedback duration
        led_decreased.value = False

    print(f"Sensitivity adjusted to {sensitivity}")
    led_increased.value = False
    led_decreased.value = False

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
    sensor_manager.perform_calibration()
    main_loop()
