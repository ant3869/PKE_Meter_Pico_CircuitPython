import math
import time
import board
import busio
import pwmio
import analogio
import digitalio
import adafruit_lsm9ds1
from math import floor
from dfplayer import DFPlayer
from adafruit_ssd1306 import SSD1306_I2C
from digitalio import DigitalInOut, Direction
from servo_ramp import ServoRamp

# --- constants   ----------------------------------------------------------

SERVO_PIN        = board.GP2

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

LED_INCREASED    = board.GP14  # Button Variable Increased Indicator LED
LED_DECREASED    = board.GP15  # Button Variable Decreased Indicator LED

BUTTON_SENSITIVITY_DECREASE  = board.GP26  # Window Variable Increase Button
# BUTTON_SENSITIVITY_INCREASE  = board.GP11  # Window Variable Decrease Button
BUTTON_GAIN_DECREASE         = board.GP27    
BUTTON_GAIN_INCREASE         = board.GP28   

EMF_MIN_µT       = 0.0           # Minimum expected EMF reading
EMF_MAX_µT       = 1100.0        # Maximum expected EMF reading
EMF_MIN_ADC      = 0
EMF_MAX_ADC      = 1100

OLED_WIDTH       = 128
OLED_HEIGHT      = 64
GRAPH_BASELINE_Y = OLED_HEIGHT-6  # Baseline for the graph (e.g., half the OLED height)
MAX_GRAPH_VALUE  = OLED_HEIGHT-62 # Maximum value after scaling
SCALE_FACTOR     = 0.5            # Adjust based on your needs
BAR_WIDTH        = 100            # Adjust the width as appropriate for your display
BAR_HEIGHT       = 5              # Adjust the height as needed
BAR_X            = 10             # X position of the progress bar
BAR_Y            = 30             # Y position of the progress bar

SW_THRESHOLD     = 65000          # Threshold value for analog reading
LED_ON_DURATION  = 1.0            # Duration to keep the LED on in seconds

# --- objects   -----------------------------------------------------------

i2c        = busio.I2C(I2C_SCL, I2C_SDA)
uart       = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX, baudrate=PLAYER_BAUD) # Using UART0 on default pins GP0 (TX) and GP1 (RX)
leds       = [LED_WING_1,LED_WING_2, LED_WING_3, LED_WING_4, LED_WING_5, LED_WING_6, LED_WING_7]
oled       = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor     = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
servo_ramp = ServoRamp(servo_pin=SERVO_PIN)
dfplayer   = DFPlayer(uart=uart)

# --- initialization   ----------------------------------------------------

led_increased               = digitalio.DigitalInOut(LED_INCREASED)
led_decreased               = digitalio.DigitalInOut(LED_DECREASED)
led_increased.direction     = digitalio.Direction.OUTPUT
led_decreased.direction     = digitalio.Direction.OUTPUT
button_gain_increase        = analogio.AnalogIn(BUTTON_GAIN_INCREASE)
button_gain_decrease        = analogio.AnalogIn(BUTTON_GAIN_DECREASE)
# button_sensitivity_increase = analogio.AnalogIn(BUTTON_SENSITIVITY_INCREASE) TO DO: not enough analog pins
button_sensitivity_decrease = analogio.AnalogIn(BUTTON_SENSITIVITY_DECREASE)
led_increased_start = 0
led_decreased_start = 0

led_objects = []
led_slow    = 1.0
led_fast    = 0.02
interval    = 0
current_led = 0
led_states      = [False] * len(led_objects)  # Initialize all LED states to False (off)
led_timestamps  = [0] * len(led_objects)  # Track when each LED was last toggled
previous_millis = time.monotonic()

calibration_time = 10
baseline         = 0
y_history        = [OLED_HEIGHT // 6] * OLED_WIDTH

# uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
# time.sleep(1)

dfplayer.set_volume(PLAYER_VOL)     
last_played_track = None

# --- functions   ----------------------------------------------------

# calibration
def calibrate_sensor():
    dfplayer.set_volume(25)
    dfplayer.play(track=09)
    
    oled.fill(0)
    oled.text("Calibrating...", 0, 0, 1)
    oled.show()
        
    baseline = calibrate_baseline(BAR_X, BAR_Y, BAR_WIDTH, BAR_HEIGHT)
    
    oled.fill(0)
    oled.text("Calibration", 0, 0, 1)
    oled.text("Complete", 0, 10, 1)
    oled.show()
    
    dfplayer.stop()
    servo_ramp.park_servo()
    time.sleep(2)  # Pause to show message

def calibrate_baseline(bar_x, bar_y, bar_width, bar_height):
    start_time = time.time()
    readings = []
    
    while time.time() - start_time < calibration_time:
        mag_x, mag_y, mag_z = sensor.magnetic
        readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
        
        elapsed_time = time.time() - start_time           
        progress = elapsed_time / calibration_time  # Progress as a fraction of 0 to 1   
        
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

# --- lights  ---------------------------------------------------- 

def setup_leds(led_pins):
    led_objects = []  # This will hold the DigitalInOut objects for the LEDs
    for pin in led_pins:
        led = digitalio.DigitalInOut(pin)
        led.direction = digitalio.Direction.OUTPUT
        led_objects.append(led)
    return led_objects

def update_led_state(led_index, state):
    """Update the state of a specific LED and record the time of this update."""
    global led_states, led_timestamps
    current_time = time.monotonic()
    if state != led_states[led_index]:  # Only update if the state has changed
        led_objects[led_index].value = state
        led_states[led_index] = state
        led_timestamps[led_index] = current_time  # Record the time of this change

# blink LEDs function
def blink_leds(led_objects, interval):
    global current_led, previous_millis
    current_time = time.monotonic()
    if current_time - previous_millis >= interval:
        led_objects[current_led].value = False  # Turn off the current LED
        current_led = (current_led + 1) % len(led_objects)
        led_objects[current_led].value = True   # Turn on the next LED
        previous_millis = current_time

# control LEDs based on EMF reading
def led_control(emf_reading, led_objects):
    emf_range = EMF_MAX_ADC - EMF_MIN_ADC
    interval_factor = ((emf_reading - EMF_MIN_ADC) / emf_range) ** 0.2  # Square root for non-linear mapping
    speed_range = led_slow - led_fast
    interval = led_slow - (interval_factor * speed_range)
    blink_leds(led_objects, interval)

# --- sound  ---------------------------------------------------- 

# play a specific track based on EMF reading
def sound_control(emf_reading):
    global last_played_track
    track_to_play = None

    if emf_reading < (EMF_MAX_ADC / 3):
        track_to_play = 01  # Track number for low EMF reading
    elif emf_reading < (2 * EMF_MAX_ADC / 3):
        track_to_play = 03  # Track number for medium EMF reading
    else:
        track_to_play = 05  # Track number for high EMF reading

    # only play if not last played track
    if track_to_play != last_played_track:
        if dfplayer.get_volume() < 75:
            dfplayer.set_volume(75)
        
#         dfplayer.play(track=track_to_play)
        last_played_track = track_to_play
        dfplayer.loop()
  
# --- buttons  ---------------------------------------------------- 
  
def update_led_states_based_on_timing(current_time):
    global led_increased_start, led_decreased_start

    # Turn off led_increased if it has been on longer than LED_ON_DURATION
    if led_increased.value and (current_time - led_increased_start > LED_ON_DURATION):
        led_increased.value = False

def check_buttons():
    global led_increased_start, led_decreased_start     
    current_time = time.monotonic()
    any_button_pressed = False

    # Check each button
    if button_gain_increase.value > SW_THRESHOLD:
        print("Gain Increase Button Pressed")
        led_increased.value = True
        led_decreased.value = False  # Ensure the other LED is off
        any_button_pressed = True
    if button_gain_decrease.value > SW_THRESHOLD:
        print("Gain Decrease Button Pressed")
        led_decreased.value = True
        led_increased.value = False  # Ensure the other LED is off
        any_button_pressed = True
    if button_sensitivity_decrease.value > SW_THRESHOLD:
        print("Sensitivity Decrease Button Pressed")
        led_decreased.value = True
        led_increased.value = False  # Ensure the other LED is off
        any_button_pressed = True
#     if button_sensitivity_increase.value > SW_THRESHOLD:
#         print("Sensitivity Increase Button Pressed")
#         led_increased.value = True
#         led_decreased.value = False  # Ensure the other LED is off
#         any_button_pressed = True

    if led_increased.value:
        led_increased_start = current_time
        update_led_states_based_on_timing(current_time)
    elif led_decreased.value:
        led_decreased_start = current_time
        update_led_states_based_on_timing(current_time)
    else:
        led_increased.value = False
        led_decreased.value = False

    time.sleep(0.1)

# --- display  ----------------------------------------------------

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
    check_buttons()
    value = process_mag_data()
    led_control(value, led_objects)
    sound_control(value)
    servo_ramp.emf_to_servo_pos(value)
    update_history(value)
    draw_graph(value)
    
# --- main   ----------------------------------------------------

led_objects = setup_leds(leds)
calibrate_sensor()
update_history(0)
servo_ramp.park_servo()
time.sleep(1)

# --- loop   ----------------------------------------------------

while True:
    update()
    time.sleep(0.1)
