# Ghostbusters "Functional" P.K.E Meter
**Author:** Ant3869

## Description
This project is a functional replica of the P.K.E. Meter from the movie Ghostbusters. It not only mimics the iconic design but also serves as a real Electromagnetic Field (EMF) detector.

### Features
- Functioning EMF detection.
- Ability to increase/decrease sensitivity/range using the 4 Direct Contact Touch Switches.
- Wings that raise or lower reflecting detection: Higher the reading, higher the raise, and vice versa.
- 7 LEDs on each wing blink in sequence, same as in the film Ghostbusters.
- LEDs speed increase and decrease reflecting detection: Higher the reading, faster the speed, and vice versa.
- 3 unique SFX while detecting (low, medium, and high readings).

### Components
- Raspberry Pi Pico with rp2040 (using Adafruit CircuitPython 8.2.9)
- LSM9DS1 D0F 3-axis Accel/Mag/Gyro
- DFPlayer Mini
- 5V 2A Recharge/Boost Converter Power Module
- SURPASS 9g S0009M Servo Metal Gear Motor
- iPhone 5 Loudspeaker
- 33 LEDs (7 on each 'wing', 14 on display screen, and 5 indicators on front panel)
- 16mm Variable Resistor Potentiometer
- 4 Direct Contact Touch Switches
- 3.7v Lithium Rechargeable Battery

### Hardware Connections (pin configurations)
- `GP0`: PLAYER_RX (DFPlayer Mini RX) (1)
- `GP1`: PLAYER_TX (DFPlayer Mini TX) (2)
- `GP2`: SERVO_PIN (Wing Servo) (4)
- `GP4`: I2C_SDA (I2C SDA connection for LSM9DS1) (6)
- `GP5`: I2C_SCL (I2C SCL connection for LSM9DS1) (7)
- `GP10`: BUTTON_GAIN_DECREASE (Decrease signal gain value) (14)
- `GP11`: BUTTON_GAIN_INCREASE (Increase signal gain value) (15)
- `GP12`: BUTTON_SENSITIVITY_DECREASE (Decrease signal detection sensitivity value) (16)
- `GP13`: BUTTON_SENSITIVITY_INCREASE (Increase signal detection sensitivity value) (17)
- `GP14`: LED_INCREASE (Signal adjustment increased indicator LED) (19)
- `GP15`: LED_DECREASE (Signal adjustment decreased indicator LED) (20)
- `GP16`: LED_WING_1 (Wing/Display LED 1) GREEN (21)
- `GP17`: LED_WING_2 (Wing/Display LED 2) BLUE (22)
- `GP18`: LED_WING_3 (Wing/Display LED 3) WHITE (24)
- `GP19`: LED_WING_4 (Wing/Display LED 4) RED (25)
- `GP20`: LED_WING_5 (Wing/Display LED 5) YELLOW (R) (26)
- `GP21`: LED_WING_6 (Wing/Display LED 6) BLACK (27)
- `GP22`: LED_WING_7 (Wing/Display LED 7) YELLOW (29)
