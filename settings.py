# GPIO Pin Definitions for RGB LEDs
LEFT_R_PIN = 22
LEFT_G_PIN = 23
LEFT_B_PIN = 24

RIGHT_R_PIN = 10
RIGHT_G_PIN = 9
RIGHT_B_PIN = 25

# GPIO Pin Definitions for Ultrasonic Sensor
ULTRASONIC_TR_PIN = 11
ULTRASONIC_EC_PIN = 8

# GPIO Pin Definitions for NeoPixel LEDs
NEOPIXEL_PIN = 12
NEOPIXEL_LED_COUNT = 16
NEOPIXEL_FREQ_HZ = 800000
NEOPIXEL_DMA = 10
NEOPIXEL_BRIGHTNESS = 255
NEOPIXEL_INVERT = False
NEOPIXEL_CHANNEL = 0

# General GPIO Pin Definitions for Front and Head Lights
FRONT_LIGHT_PIN_1 = 5
FRONT_LIGHT_PIN_2 = 6
FRONT_LIGHT_PIN_3 = 13

# GPIO Pin Definitions for Robot Motors
MOTOR_A_EN = 4
MOTOR_B_EN = 17
MOTOR_A_PIN1 = 14
MOTOR_A_PIN2 = 15
MOTOR_B_PIN1 = 27
MOTOR_B_PIN2 = 18
PWM_FREQUENCY_ENGINE = 1000

# GPIO States
GPIO_ON = False  # GPIO.LOW
GPIO_OFF = True  # GPIO.HIGH

# System Paths for Temperature Sensors
CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"
GPU_TEMP_COMMAND = "/opt/vc/bin/vcgencmd measure_temp"

# PCA9685 PWM Frequency
PWM_FREQUENCY = 50

# Servo Configuration
SERVO_INIT_POS = 285
SERVO_GOAL_POS = 285
SERVO_MAX_POS = 500
SERVO_MIN_POS = 100
SERVO_RANGE = 150

SERVO_HEAD = 0
SERVO_MID_HEAD = 1
SERVO_WHEEL = 2


# Settings for Robot Functions
LINE_PIN_LEFT = 19
LINE_PIN_MIDDLE = 16
LINE_PIN_RIGHT = 20

# Motor settings
MOTOR_SPEED = 50

# Servo settings
ANGLE_RATE = 1

# Line following color selection (0 for white line, 1 for black line)
COLOR_SELECT = 1


# server settings 
HOST = ''
PORT = 10223  # Define port serial 
BUFSIZ = 1024  # Define buffer size
ADDR = (HOST, PORT)