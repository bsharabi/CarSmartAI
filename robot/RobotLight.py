import time
import RPi.GPIO as GPIO  # type: ignore
from rpi_ws281x import *  # type: ignore
import threading
import settings

class RobotLight(threading.Thread):
    """
    A singleton class to control the lights of a robot using GPIO pins.
    This class extends threading.Thread to allow asynchronous control.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(RobotLight, cls).__new__(cls)
        return cls._instance

    def __init__(self, *args, **kwargs):
        if not hasattr(self, 'initialized'):  # To prevent reinitialization
            super(RobotLight, self).__init__(*args, **kwargs)
            self.LED_COUNT = settings.NEOPIXEL_LED_COUNT
            self.LED_PIN = settings.NEOPIXEL_PIN
            self.LED_FREQ_HZ = settings.NEOPIXEL_FREQ_HZ
            self.LED_DMA = settings.NEOPIXEL_DMA
            self.LED_BRIGHTNESS = settings.NEOPIXEL_BRIGHTNESS
            self.LED_INVERT = settings.NEOPIXEL_INVERT
            self.LED_CHANNEL = settings.NEOPIXEL_CHANNEL

            # Color breath settings
            self.colorBreathR = 0
            self.colorBreathG = 0
            self.colorBreathB = 0
            self.breathSteps = 10

            # GPIO pin definitions for left and right RGB LEDs
            self.left_R = settings.LEFT_R_PIN
            self.left_G = settings.LEFT_G_PIN
            self.left_B = settings.LEFT_B_PIN

            self.right_R = settings.RIGHT_R_PIN
            self.right_G = settings.RIGHT_G_PIN
            self.right_B = settings.RIGHT_B_PIN
            
            self.pin_led_1 = settings.FRONT_LIGHT_PIN_1
            self.pin_led_2 = settings.FRONT_LIGHT_PIN_2
            self.pin_led_3 = settings.FRONT_LIGHT_PIN_3

            # GPIO output states
            self.on = GPIO.LOW
            self.off = GPIO.HIGH

            # Current light mode: 'none', 'police', 'breath'
            self.lightMode = 'none'

            self.setup()

            # Create NeoPixel object with appropriate configuration.
            self.strip = Adafruit_NeoPixel(self.LED_COUNT, self.LED_PIN, self.LED_FREQ_HZ, self.LED_DMA, self.LED_INVERT, self.LED_BRIGHTNESS, self.LED_CHANNEL)  # type: ignore
            # Initialize the library (must be called once before other functions).
            self.strip.begin()

            self.__flag = threading.Event()
            self.__flag.clear()
            
            self.initialized = True

    def setup(self):
        """
        Set up the GPIO pins and turn off both RGB LEDs.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_led_1, GPIO.OUT)
        GPIO.setup(self.pin_led_2, GPIO.OUT)
        GPIO.setup(self.pin_led_3, GPIO.OUT)

        GPIO.setup(self.left_R, GPIO.OUT)
        GPIO.setup(self.left_G, GPIO.OUT)
        GPIO.setup(self.left_B, GPIO.OUT)
        GPIO.setup(self.right_R, GPIO.OUT)
        GPIO.setup(self.right_G, GPIO.OUT)
        GPIO.setup(self.right_B, GPIO.OUT)
        self.both_off()

    def both_off(self):
        """
        Turn off both left and right RGB LEDs.
        """
        GPIO.output(self.left_R, self.off)
        GPIO.output(self.left_G, self.off)
        GPIO.output(self.left_B, self.off)
        GPIO.output(self.right_R, self.off)
        GPIO.output(self.right_G, self.off)
        GPIO.output(self.right_B, self.off)

    def both_on(self):
        """
        Turn on both left and right RGB LEDs.
        """
        GPIO.output(self.left_R, self.on)
        GPIO.output(self.left_G, self.on)
        GPIO.output(self.left_B, self.on)
        GPIO.output(self.right_R, self.on)
        GPIO.output(self.right_G, self.on)
        GPIO.output(self.right_B, self.on)

    def side_on(self, side_X):
        """
        Turn on a specific side RGB LED.
        
        :param side_X: The GPIO pin of the side to turn on.
        """
        GPIO.output(side_X, self.on)

    def side_off(self, side_X):
        """
        Turn off a specific side RGB LED.
        
        :param side_X: The GPIO pin of the side to turn off.
        """
        GPIO.output(side_X, self.off)

    def red(self):
        """
        Turn on both red LEDs.
        """
        self.side_on(self.right_R)
        self.side_on(self.left_R)

    def green(self):
        """
        Turn on both green LEDs.
        """
        self.side_on(self.right_G)
        self.side_on(self.left_G)

    def blue(self):
        """
        Turn on both blue LEDs.
        """
        self.side_on(self.right_B)
        self.side_on(self.left_B)

    def yellow(self):
        """
        Turn on both red and green LEDs to produce yellow light.
        """
        self.red()
        self.green()

    def pink(self):
        """
        Turn on both red and blue LEDs to produce pink light.
        """
        self.red()
        self.blue()

    def cyan(self):
        """
        Turn on both blue and green LEDs to produce cyan light.
        """
        self.blue()
        self.green()

    def turn_left(self):
        """
        Simulate turning left indicator.
        """
        GPIO.output(self.left_G, self.on)
        GPIO.output(self.left_R, self.on)

    def turn_right(self):
        """
        Simulate turning right indicator.
        """
        GPIO.output(self.right_G, self.on)
        GPIO.output(self.right_R, self.on)

    def setColor(self, R, G, B):
        """
        Set the color of the NeoPixel LEDs.
        
        :param R: Red component (0-255)
        :param G: Green component (0-255)
        :param B: Blue component (0-255)
        """
        color = Color(int(R), int(G), int(B))  # type: ignore
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
        self.strip.show()

    def setSomeColor(self, R, G, B, IDs):
        """
        Set the color of specific NeoPixel LEDs.
        
        :param R: Red component (0-255)
        :param G: Green component (0-255)
        :param B: Blue component (0-255)
        :param IDs: List of LED IDs to set the color
        """
        color = Color(int(R), int(G), int(B))  # type: ignore
        for i in IDs:
            self.strip.setPixelColor(i, color)
        self.strip.show()

    def pause(self):
        """
        Pause the current light mode.
        """
        self.lightMode = 'none'
        self.setColor(0, 0, 0)
        self.__flag.clear()

    def resume(self):
        """
        Resume the current light mode.
        """
        self.__flag.set()

    def police(self):
        """
        Activate police light mode.
        """
        self.lightMode = 'police'
        self.resume()

    def policeProcessing(self):
        """
        Process the police light mode.
        """
        while self.lightMode == 'police':
            for _ in range(3):
                self.setSomeColor(0, 0, 255, list(range(12)))
                self.blue()
                time.sleep(0.05)
                self.setSomeColor(0, 0, 0, list(range(12)))
                self.both_off()
                time.sleep(0.05)
            if self.lightMode != 'police':
                break
            time.sleep(0.1)
            for _ in range(3):
                self.setSomeColor(255, 0, 0, list(range(12)))
                self.red()
                time.sleep(0.05)
                self.setSomeColor(0, 0, 0, list(range(12)))
                self.both_off()
                time.sleep(0.05)
            time.sleep(0.1)

    def breath(self, R_input, G_input, B_input):
        """
        Activate breath light mode with specified color.
        
        :param R_input: Red component (0-255)
        :param G_input: Green component (0-255)
        :param B_input: Blue component (0-255)
        """
        self.lightMode = 'breath'
        self.colorBreathR = R_input
        self.colorBreathG = G_input
        self.colorBreathB = B_input
        self.resume()

    def breathProcessing(self):
        """
        Process the breath light mode.
        """
        while self.lightMode == 'breath':
            for i in range(self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(self.colorBreathR * i / self.breathSteps, self.colorBreathG * i / self.breathSteps, self.colorBreathB * i / self.breathSteps)
                time.sleep(0.03)
            for i in range(self.breathSteps):
                if self.lightMode != 'breath':
                    break
                self.setColor(self.colorBreathR - (self.colorBreathR * i / self.breathSteps), self.colorBreathG - (self.colorBreathG * i / self.breathSteps), self.colorBreathB - (self.colorBreathB * i / self.breathSteps))
                time.sleep(0.03)

    def frontLight(self, switch):
        """
        Control the front light.
        
        :param switch: 'on' to turn on, 'off' to turn off
        """
        if switch == 'on':
            GPIO.output(6, GPIO.HIGH)
            GPIO.output(13, GPIO.HIGH)
        elif switch == 'off':
            GPIO.output(5, GPIO.LOW)
            GPIO.output(13, GPIO.LOW)

    def switch(self, port, status):
        """
        Control a specific port switch.
        
        :param port: Port number (1, 2, or 3)
        :param status: 1 to turn on, 0 to turn off
        """
        if port == 1:
            GPIO.output(self.pin_led_1, GPIO.HIGH if status == 1 else GPIO.LOW)
        elif port == 2:
            GPIO.output(self.pin_led_2, GPIO.HIGH if status == 1 else GPIO.LOW)
        elif port == 3:
            GPIO.output(self.pin_led_3, GPIO.HIGH if status == 1 else GPIO.LOW)
        else:
            print('Wrong Command: Example--switch(3, 1)->to switch on port3')

    def set_all_switch_off(self):
        """
        Turn off all switches.
        """
        self.switch(1, 0)
        self.switch(2, 0)
        self.switch(3, 0)
        
    def headLight(self, switch):
        """
        Control the head light.
        
        :param switch: 'on' to turn on, 'off' to turn off
        """
        if switch == 'on':
            GPIO.output(5, GPIO.HIGH)
        elif switch == 'off':
            GPIO.output(5, GPIO.LOW)

    def lightChange(self):
        """
        Change the light mode based on the current lightMode setting.
        """
        if self.lightMode == 'none':
            self.pause()
        elif self.lightMode == 'police':
            self.policeProcessing()
        elif self.lightMode == 'breath':
            self.breathProcessing()

    def run(self):
        """
        Run the thread to handle the light change process.
        """
        while True:
            self.__flag.wait()
            self.lightChange()


if __name__ == '__main__':
    RL = RobotLight()
    RL.start()
    RL.breath(70, 70, 255)
    time.sleep(15)
    RL.pause()
    RL.frontLight('off')
    time.sleep(2)
    RL.police()
