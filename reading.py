import math
import time
import board
import busio
import pwmio
import digitalio
import adafruit_lsm9ds1
from math import floor
from dfplayer import DFPlayer
from adafruit_ssd1306 import SSD1306_I2C
from digitalio import DigitalInOut, Direction

# --- constants   ----------------------------------------------------------
SERVO_PIN        = board.GP2

PLAYER_TX        = board.GP0  # board.TX
PLAYER_RX        = board.GP1  # board.RX
PLAYER_BAUD      = 9600
PLAYER_VOL       = 100

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

EMF_MIN_µT       = 0.0           # Minimum expected EMF reading
EMF_MAX_µT       = 100.0         # Maximum expected EMF reading
EMF_MIN_ADC      = 0
EMF_MAX_ADC      = 1100

SCALE_FACTOR     = 0.5           # Adjust based on your needs
MAX_GRAPH_VALUE  = 1100          # Maximum value after scaling 

OLED_WIDTH       = 128
OLED_HEIGHT      = 64
GRAPH_BASELINE_Y = OLED_HEIGHT-6 # Baseline for the graph (e.g., half the OLED height)

# --- objects   -----------------------------------------------------------
i2c   = busio.I2C(I2C_SCL, I2C_SDA)
uart  = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX, baudrate=PLAYER_BAUD) # Using UART0 on default pins GP0 (TX) and GP1 (RX)
button_increase           = digitalio.DigitalInOut(BUTTON_GAIN_INCREASE)  # Direct use as touch-button
button_increase.direction = digitalio.Direction.INPUT
# button_increase.pull      = digitalio.Pull.DOWN  # Use Pull.UP if logic is inverted
led_increased             = digitalio.DigitalInOut(LED_INCREASED)
led_increased.direction   = digitalio.Direction.OUTPUT
leds          = [LED_WING_1,LED_WING_2, LED_WING_3, LED_WING_4, LED_WING_5, LED_WING_6, LED_WING_7]
led_slow    = 1.0
led_fast    = 0.1
interval    = 0
current_led = 0
previous_millis = time.monotonic()

# --- initialization   ----------------------------------------------------
oled           = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor         = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
led_wing1_pwm  = pwmio.PWMOut(LED_WING_1, frequency=5000, duty_cycle=0) # Initialize PWM for the LED on LED_WING_1
y_history      = [OLED_HEIGHT // 6] * OLED_WIDTH    # Mid-screen baseline

uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
time.sleep(1)
dfplayer = DFPlayer(uart=uart)                # Creates uart internally
dfplayer.set_volume(PLAYER_VOL)                # Set volume to PLAYER_VOL
time.sleep(1)
dfplayer.play(track=8) # Play startup confirmation sound effect
setup_leds()

# --- functions   ----------------------------------------------------
def calibrate_sensor(sensor, oled, calibration_time=10):
    oled.fill(0)
    oled.text("Calibrating...", 0, 0, 1)
    oled.show()

    start_time = time.time()
    readings = []
    while time.time() - start_time < calibration_time:
        mag_x, mag_y, mag_z = sensor.magnetic
        readings.append(math.sqrt(mag_x**2 + mag_y**2 + mag_z**2))
        time.sleep(0.1)
    
    baseline = sum(readings) / len(readings)
    oled.fill(0)
    oled.text("Calibration", 0, 0, 1)
    oled.text("Complete", 0, 10, 1)
    oled.show()
    time.sleep(2)  # Pause to show message
    return baseline

def get_baseline(readings):
    return sum(readings) / len(readings)

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
def process_mag_data(baseline):
    emf_strength = math.sqrt(mx**2 + my**2 + mz**2) - baseline
    mapped_emf = map_value(emf_strength, EMF_MIN_µT, EMF_MAX_µT, EMF_MIN_ADC, EMF_MAX_ADC)
    processed_value = (mapped_emf ** 2)
    return max(0, min(1100, processed_value))

def setup_leds(leds):
    for led in leds:
        led = digitalio.DigitalInOut(pin)
        led.direction = digitalio.Direction.OUTPUT

def blink_leds(interval):
    current_time = time.monotonic()
    if current_time - previous_millis >= interval:
        leds[current_led].value = False   # Turn off the current LED
        current_led = (current_led + 1) % len(leds)
        leds[current_led].value = True  # Turn on the next LED
        previous_millis = current_time
        
# Function to control LEDs
def led_control(emf_reading):
    emf_range = EMF_MAX_ADC - EMF_MIN_ADC
    speed_range = LED_SLOW - LED_FAST
    interval = (((emf_reading - EMF_MIN_ADC) * speed_range) / emf_range) + LED_FAST
    blink_leds(interval)
    
# Function to play a specific track based on EMF reading
def play_track_based_on_emf(emf_reading):
    if emf_reading < (EMF_MAX_ADC / 3):
        dfplayer.play(track=8)  # Play track 1
    elif emf_reading < (2 * EMF_MAX_ADC / 3):
        dfplayer.play(track=7)  # Play track 2
    else:
        dfplayer.play(track=6)  # Play track 3
    
def check_touch_and_toggle_led(touch_sensor, led):
    if touch_sensor.value: led.value = True else led.value = False
    time.sleep(0.3)        # Debounce delay

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

def draw_graph(emf_reading):
    oled.fill(0)  # Clear the display    
    oled.text('EMF: {:.2f}'.format(emf_reading), 0, 0, 1)
    oled.text('SH: {:.2f}'.format(y_history[-1] if y_history else 0), 0, 10, 1)

    if smoothed_history:
        for x in range(1, len(emf_reading)):
            y0 = GRAPH_BASELINE_Y - (y_history[x - 1] * SCALE_FACTOR)
            y1 = GRAPH_BASELINE_Y - (y_history[x] * SCALE_FACTOR)            
            y0 = max(0, min(oled.height, y0))
            y1 = max(0, min(oled.height, y1))
            y0 = int(max(0, min(oled.height - 1, y0)))
            y1 = int(max(0, min(oled.height - 1, y1)))            
            draw_line(oled, x - 1, y0, x, y1, 1)

    oled.show()

def update():
    mag_x, mag_y, mag_z = sensor.magnetic                  # Collect raw magnetometer data reading   
    value               = process_mag_data(mag_x, mag_y, mag_z, baseline_emf_strength) # Process magnetometer data
    check_touch_and_toggle_led(button_increase, led_increased)   
    # processed_value     = simple_moving_average(value)     # Refine reading
    update_history(value)   
    led_control(value)                   # Integrate dynamic LED control
    play_track_based_on_emf(value)               # Control the DFPlayer Mini based on adjusted sensor data
    draw_graph(value)    # Draw Graph for debuging
    
# Main execution
baseline_emf_strength = calibrate_sensor(sensor, oled)
update_history(0)

while True:
    update()
    time.sleep(0.1)
