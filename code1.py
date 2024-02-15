"""

GP0: PLAYER_RX (DFPlayer Mini RX) (1)
GP1: PLAYER_TX (DFPlayer Mini TX) (2)
GP2: SERVO_PIN (Wing Servo) (4)
GP4: I2C_SDA (I2C SDA connection for LSM9DS1) (6)
GP5: I2C_SCL (I2C SCL connection for LSM9DS1) (7)
GP26: BUTTON_CYCLE_GAIN (Cycle through gain settings [0, 32, 64, 90]) (15)
GP27: BUTTON_SENSITIVITY_DECREASE (Decrease signal detection sensitivity value) (16)
GP28: BUTTON_SENSITIVITY_INCREASE (Increase signal detection sensitivity value) (17)
GP14: LED_INCREASE (Signal adjustment increased indicator LED) (19)
GP15: LED_DECREASE (Signal adjustment decreased indicator LED) (20)
GP16: LED_WING_1 (Wing/Display LED 1) RED    (21)
GP17: LED_WING_2 (Wing/Display LED 2) GREEN  (22)
GP18: LED_WING_3 (Wing/Display LED 3) BLUE   (24)
GP19: LED_WING_4 (Wing/Display LED 4) WHITE  (25)
GP20: LED_WING_5 (Wing/Display LED 5) YELLOW (26)
GP21: LED_WING_6 (Wing/Display LED 6) PURPLE (27)
GP22: LED_WING_7 (Wing/Display LED 7) BLACK  (29)

"""

import math
import time
import board
import busio
import pwmio
import analogio
import digitalio
import adafruit_lsm9ds1
from math import floor
from adafruit_ssd1306 import SSD1306_I2C
from digitalio import DigitalInOut, Direction
from servo_ramp import ServoRamp
from sound_manager import SoundManager
from led_manager import LEDManager
from button_manager import ButtonManager

# --- constants   ----------------------------------------------------------

SERVO_PIN        = board.GP2
PLAYER_TX, PLAYER_RX    = board.GP0, board.GP1 
I2C_SDA, I2C_SCL        = board.GP4, board.GP5

LED_WING_1       = board.GP16
LED_WING_2       = board.GP17
LED_WING_3       = board.GP18
LED_WING_4       = board.GP19
LED_WING_5       = board.GP20
LED_WING_6       = board.GP21
LED_WING_7       = board.GP22

# --- variables   ----------------------------------------------------------

PLAYER_BAUD, PLAYER_VOL = 9600, 100
EMF_MIN_µT, EMF_MAX_µT     = 0.0, 1100.0    # Minimum expected EMF reading     
EMF_MIN_ADC, EMF_MAX_ADC   = 0,   1100      # Maximum expected EMF reading

OLED_WIDTH, OLED_HEIGHT, SCALE_FACTOR = 128, 64, 0.5
GRAPH_BASELINE_Y, MAX_GRAPH_VALUE     = OLED_HEIGHT-6, OLED_HEIGHT-62 
BAR_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT   = 10, 30, 100, 5

# --- objects   -----------------------------------------------------------

i2c        = busio.I2C(I2C_SCL, I2C_SDA)
leds       = [LED_WING_1,LED_WING_2, LED_WING_3, LED_WING_4, LED_WING_5, LED_WING_6, LED_WING_7]
oled       = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor     = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
servo_ramp     = ServoRamp(servo_pin=SERVO_PIN)
sound_manager  = SoundManager()
led_manager    = LEDManager(leds)
button_manager = ButtonManager(sensor)

# --- initialization   ----------------------------------------------------

calibration_time = 20
baseline         = 0
y_history        = [OLED_HEIGHT // 6] * OLED_WIDTH

# --- functions   ----------------------------------------------------

# calibration
def calibrate_sensor():
    sound_manager.play_track(09)
    
    oled.fill(0)
    oled.text("Calibrating...", 0, 0, 1)
    oled.show()
        
    baseline = calibrate_baseline(BAR_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT)
    
    oled.fill(0)
    oled.text("Calibration", 0, 0, 1)
    oled.text("Complete", 0, 10, 1)
    oled.show()
    
    sound_manager.stop_playback()
    servo_ramp.park_servo()
    time.sleep(2)

def calibrate_baseline(bar_x, bar_y, bar_width, bar_height):
    start_time = time.time()
    readings = []
    
    while time.time() - start_time < calibration_time:
        mag_x, mag_y, mag_z = sensor.magnetic
        readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
        elapsed_time = time.time() - start_time       
        progress = elapsed_time / calibration_time    
        fill_width = int(progress * bar_width)
        servo_progress = int(progress * 10)
        oled.fill_rect(bar_x, bar_y, fill_width, bar_height, 1)
        servo_ramp.calibration_to_servo_pos(servo_progress)
        
        servo_ramp.update()
        oled.show()
        
        time.sleep(0.1)
        
    return sum(readings) / len(readings)

# update history buffer
def update_history(new_value):
    y_history.append(new_value)
    del y_history[0]

# map from one range to another
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# simple moving average
def simple_moving_average(history, window_size=10):
    return [sum(history[max(i - window_size, 0):i+1]) / min(i + 1, window_size) for i in range(len(history))]

# process magnetometer data
def process_mag_data():
    mag_x, mag_y, mag_z = sensor.magnetic
    emf_strength = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2) - baseline
    mapped_emf = map_value(emf_strength, EMF_MIN_µT, EMF_MAX_µT, EMF_MIN_ADC, EMF_MAX_ADC)
    processed_value = (mapped_emf ** 3)
    return max(0, min(1100, processed_value))

# --- display  ----------------------------------------------------

# # Setup LEDs
# leds = [digitalio.DigitalInOut(pin) for pin in led_pins]
# for led in leds:
#     led.direction = digitalio.Direction.OUTPUT
# 
# def blink_leds(leds, duration, min_interval=0.0001):
#     end_time = time.monotonic() + duration
#     while time.monotonic() < end_time:
#         for led in leds:
#             led.value = True
#             time.sleep(min_interval)  # Sleep for a very short duration
#             led.value = False

# draw a line
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

def draw_graph(value):
    oled.fill(0)  
    oled.text('EMF: {:.2f}'.format(value), 0, 0, 1)
    oled.text('Base: {:.2f}'.format(baseline), 0, 10, 1)

    if y_history and len(y_history) > 1:
        for x in range(1, len(y_history)):
            y0 = GRAPH_BASELINE_Y - (y_history[x - 1] * SCALE_FACTOR)
            y1 = GRAPH_BASELINE_Y - (y_history[x] * SCALE_FACTOR)            
            y0 = max(0, min(OLED_HEIGHT, y0))
            y1 = max(0, min(OLED_HEIGHT, y1))
            y0 = int(max(0, min(OLED_HEIGHT - 1, y0)))
            y1 = int(max(0, min(OLED_HEIGHT - 1, y1)))            
            draw_line(oled, x - 1, y0, x, y1, 1)

    oled.show()

def update():
    button_manager.check_and_handle_buttons()
    value = process_mag_data()
    led_manager.led_control(value)
    sound_manager.sound_control(value)
    servo_ramp.emf_to_servo_pos(value)
    update_history(value)
    draw_graph(value)
    
# --- main   ----------------------------------------------------

calibrate_sensor()
update_history(0)
servo_ramp.park_servo()
time.sleep(1)

# --- loop   ----------------------------------------------------

while True:
    update()
    #time.sleep(0.001)
