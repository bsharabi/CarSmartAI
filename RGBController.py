import RPi.GPIO as GPIO
import time

class RGBController:
    def __init__(self):
        self.left_R = 22
        self.left_G = 23
        self.left_B = 24

        self.right_R = 10
        self.right_G = 9
        self.right_B = 25

        self.on = GPIO.LOW
        self.off = GPIO.HIGH

        self.setup()

    def setup(self):
        """
        Set up the GPIO pins and turn off both RGB LEDs.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_R, GPIO.OUT)
        GPIO.setup(self.left_G, GPIO.OUT)
        GPIO.setup(self.left_B, GPIO.OUT)
        GPIO.setup(self.right_R, GPIO.OUT)
        GPIO.setup(self.right_G, GPIO.OUT)
        GPIO.setup(self.right_B, GPIO.OUT)
        self.both_off()

    def both_on(self):
        """
        Turn on both RGB LEDs.
        """
        GPIO.output(self.left_R, self.on)
        GPIO.output(self.left_G, self.on)
        GPIO.output(self.left_B, self.on)
        GPIO.output(self.right_R, self.on)
        GPIO.output(self.right_G, self.on)
        GPIO.output(self.right_B, self.on)

    def both_off(self):
        """
        Turn off both RGB LEDs.
        """
        GPIO.output(self.left_R, self.off)
        GPIO.output(self.left_G, self.off)
        GPIO.output(self.left_B, self.off)
        GPIO.output(self.right_R, self.off)
        GPIO.output(self.right_G, self.off)
        GPIO.output(self.right_B, self.off)

    def side_on(self, pin):
        """
        Turn on the specified RGB LED pin.
        """
        GPIO.output(pin, self.on)

    def side_off(self, pin):
        """
        Turn off the specified RGB LED pin.
        """
        GPIO.output(pin, self.off)

    def police(self, police_time):
        """
        Simulate police lights for the specified duration.
        """
        for _ in range(police_time):
            for _ in range(3):
                self.side_on(self.left_R)
                self.side_on(self.right_B)
                time.sleep(0.1)
                self.both_off()
                self.side_on(self.left_B)
                self.side_on(self.right_R)
                time.sleep(0.1)
                self.both_off()
            for _ in range(5):
                self.side_on(self.left_R)
                self.side_on(self.right_B)
                time.sleep(0.3)
                self.both_off()
                self.side_on(self.left_B)
                self.side_on(self.right_R)
                time.sleep(0.3)
                self.both_off()

    def red(self):
        """
        Turn on both red LEDs.
        """
        self.side_on(self.left_R)
        self.side_on(self.right_R)

    def green(self):
        """
        Turn on both green LEDs.
        """
        self.side_on(self.left_G)
        self.side_on(self.right_G)

    def blue(self):
        """
        Turn on both blue LEDs.
        """
        self.side_on(self.left_B)
        self.side_on(self.right_B)

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

    def turn_left(self, times):
        """
        Simulate turning left indicator for the specified number of times.
        """
        for _ in range(times):
            self.both_off()
            self.side_on(self.left_G)
            self.side_on(self.left_R)
            time.sleep(0.5)
            self.both_off()
            time.sleep(0.5)

    def turn_right(self, times):
        """
        Simulate turning right indicator for the specified number of times.
        """
        for _ in range(times):
            self.both_off()
            self.side_on(self.right_G)
            self.side_on(self.right_R)
            time.sleep(0.5)
            self.both_off()
            time.sleep(0.5)

def main():
    rgb = RGBController()

    # Run tests
    rgb.police(4)
    rgb.both_on()
    time.sleep(1)
    rgb.both_off()
    rgb.yellow()
    time.sleep(5)
    rgb.both_off()
    rgb.pink()
    time.sleep(5)
    rgb.both_off()
    rgb.cyan()
    time.sleep(5)
    rgb.both_off()

if __name__ == '__main__':
    main()
