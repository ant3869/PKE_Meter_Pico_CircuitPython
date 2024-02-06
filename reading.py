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
PLAYER_TX        = board.GP0  # board.TX
PLAYER_RX        = board.GP1  # board.RX
PLAYER_BAUD      = 9600
PLAYER_VOL       = 50

I2C_SDA          = board.GP4
I2C_SCL          = board.GP5

LED_WING_1       = board.GP9
LED_INCREASED    = board.GP13    # Alpha Variable Increased Indicator LED

BUTTON_INCREASE  = board.GP14    # Alpha Variable Increase

EMF_MIN_µT       = 0.0           # Minimum expected EMF reading
EMF_MAX_µT       = 100.0         # Maximum expected EMF reading
EMF_MIN_ADC      = 0
EMF_MAX_ADC      = 1100

LED_INTENSITY_MIN = 0.2
LED_INTENSITY_MAX = 1
LED_SLOW         = 0.9           # Slowest blink speed in seconds
LED_FAST         = 0.1           # Fastest blink speed in seconds

SCALE_FACTOR     = 0.1           # Adjust based on your needs
MAX_GRAPH_VALUE  = 1100          # Maximum value after scaling 

OLED_WIDTH       = 128
OLED_HEIGHT      = 64
GRAPH_BASELINE_Y = OLED_HEIGHT-6 # Baseline for the graph (e.g., half the OLED height)

# --- objects   -----------------------------------------------------------
i2c   = busio.I2C(I2C_SCL, I2C_SDA)
# uart  = busio.UART(tx=PLAYER_TX, rx=PLAYER_RX, baudrate=PLAYER_BAUD) # Using UART0 on default pins GP0 (TX) and GP1 (RX)
button_increase           = digitalio.DigitalInOut(BUTTON_INCREASE)  # Direct use as touch-button
button_increase.direction = digitalio.Direction.INPUT
button_increase.pull      = digitalio.Pull.DOWN  # Use Pull.UP if logic is inverted
led_increased             = digitalio.DigitalInOut(LED_INCREASED)
led_increased.direction   = digitalio.Direction.OUTPUT

# --- initialization   ----------------------------------------------------
oled           = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
sensor         = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

led_wing1_pwm  = pwmio.PWMOut(LED_WING_1, frequency=5000, duty_cycle=0) # Initialize PWM for the LED on LED_WING_1
y_history      = [OLED_HEIGHT // 6] * OLED_WIDTH    # Mid-screen baseline

# uart.write(b'\x7E\xFF\x06\x0C\x00\x00\x00\x00\xEF') #DFPlayer Mini to reset
time.sleep(1)
# dfplayer       = DFPlayer(uart=uart)                # Creates uart internally
# dfplayer.set_volume(PLAYER_VOL)                     # Set volume to PLAYER_VOL
time.sleep(1)
# dfplayer.play(8) # Play startup confirmation sound effect

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
    processed_value = (mapped_emf ** 2)
    return max(0, min(1100, processed_value))

# Function to control LEDs
def dynamic_led_control(emf_level):
    intensity = map_value(emf_level, EMF_MIN_ADC, EMF_MAX_ADC, LED_INTENSITY_MIN, LED_INTENSITY_MAX)
    intensity = max(LED_INTENSITY_MIN, min(LED_INTENSITY_MAX, intensity))        # Clamp intensity to [LED_INTENSITY_MIN, LED_INTENSITY_MAX] range
    blink_speed = map_value(intensity, 0, 1, LED_SLOW, LED_FAST)
#    blink_speed = map_value(emf_level, EMF_MIN_ADC, EMF_MAX_ADC, LED_SLOW, LED_FAST)
#    blink_speed = max(LED_SLOW, min(LED_FAST, blink_speed)) # Clamp intensity to [LED_SLOW, LED_FAST] range
    
    duty_cycle = floor(intensity * 65535)        # Calculate duty_cycle within 0-65535 range
    duty_cycle = max(0, min(65535, duty_cycle))  # Additional clamping for safety
    led_wing1_pwm.duty_cycle = duty_cycle

    # Keep the LED on for a fraction of the blink speed, then turn off
    time.sleep(blink_speed / 4)  # On time
    led_wing1_pwm.duty_cycle = 0
    time.sleep(blink_speed / 4)  # Off time

# Function to play a specific track based on EMF reading
def play_track_based_on_emf(emf_reading):
    if emf_reading < (EMF_MAX_ADC / 3):
        dfplayer.play(1)  # Play track 1
    elif emf_reading < (2 * EMF_MAX_ADC / 3):
        dfplayer.play(2)  # Play track 2
    else:
        dfplayer.play(3)  # Play track 3

def classify_emf_reading(reading, baseline):
    offset = reading - baseline
    if offset <= 40:
        return "MINIMUM"
    elif offset <= 300:
        return "LOW"
    elif offset <= 700:
        return "MODERATE"
    elif offset <= 1000:
        return "ELEVATED"
    else:
        return "EXTREME"
    
def check_touch_and_toggle_led(touch_sensor, led):
    if touch_sensor.value: # Adjust 'button_increase.value' based on your sensor/library
        led.value = True   # Turn on LED
    else:
        led.value = False  # Turn off LED
    time.sleep(0.1)        # Debounce delay

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

def draw_graph(oled, latest_reading, processed_value, description, smoothed_history):
    oled.fill(0)  # Clear the display
    
    oled.text('EMF: {:.2f}'.format(latest_reading), 0, 0, 1)
    oled.text(description, 0, 10, 1)
#     description = classify_emf_reading(processed_value, baseline_emf_strength)
#     oled.text('EMF: {:.2f}'.format(processed_value), 0, 0, 1)
#     oled.text('SH: {:.2f}'.format(smoothed_history[-1] if smoothed_history else 0), 0, 10, 1)

    if smoothed_history:
        # Assume SCALE_FACTOR is defined; adjust it based on your data's range
        for x in range(1, len(smoothed_history)):
            # Subtract the scaled value from the baseline to make the wave go up
            y0 = GRAPH_BASELINE_Y - (smoothed_history[x - 1] * SCALE_FACTOR)
            y1 = GRAPH_BASELINE_Y - (smoothed_history[x] * SCALE_FACTOR)
            
            # Flip the graph direction by adjusting y-coordinates
            y0 = max(0, min(oled.height, y0))
            y1 = max(0, min(oled.height, y1))

            # Ensure coordinates are integers and within bounds
            y0 = int(max(0, min(oled.height - 1, y0)))
            y1 = int(max(0, min(oled.height - 1, y1)))
            
            draw_line(oled, x - 1, y0, x, y1, 1)

    oled.show()

# Main execution
baseline_emf_strength = calibrate_sensor(sensor, oled)
update_history(0)

while True:
    check_touch_and_toggle_led(button_increase, led_increased)
    mag_x, mag_y, mag_z = sensor.magnetic                  # Collect raw magnetometer data reading   
    processed_value     = process_mag_data(mag_x, mag_y, mag_z, baseline_emf_strength) # Process magnetometer data
    update_history(processed_value)                        # Track recorded data
#     smoothed_history = simple_moving_average(y_history) # Refine reading
#     processed_value = simple_moving_average(processed_value)
    update_history(processed_value)
    smoothed_history = simple_moving_average(y_history)
#     update_history(processed_value) 
    latest_reading = smoothed_history[-1] if smoothed_history else baseline_emf_strength # Use the latest reading for adjustments and classification
#     update_history(latest_reading)
    description = classify_emf_reading(latest_reading, baseline_emf_strength)
    
    dynamic_led_control(latest_reading)      # Now correctly pass the latest reading for dynamic LED control and audio playback
# #     play_track_based_on_emf(latest_reading)  # Ensure this function is defined to map descriptions to track numbers
    
    draw_graph(oled, latest_reading, processed_value, description, smoothed_history)
    time.sleep(0.1)                               # Consider adding a small delay for efficiency
#     play_track_based_on_emf(processed_value)               # Control the DFPlayer Mini based on adjusted sensor data
#     dynamic_led_control(processed_value)                   # Integrate dynamic LED control
#     draw_graph(oled, smoothed_history, processed_value)    # Draw Graph for debuging
#     time.sleep(0.1)