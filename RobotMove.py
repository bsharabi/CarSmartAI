import time
import RPi.GPIO as GPIO  # type: ignore
import settings
import threading


class RobotMove(threading.Thread):
    """
    A singleton class to control the movement of a robot using GPIO pins.
    This class extends threading.Thread to allow asynchronous control.
    """
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(RobotMove, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):  # To prevent reinitialization
            super().__init__()
            self.Motor_A_EN = settings.MOTOR_A_EN
            self.Motor_B_EN = settings.MOTOR_B_EN
            self.Motor_A_Pin1 = settings.MOTOR_A_PIN1
            self.Motor_A_Pin2 = settings.MOTOR_A_PIN2
            self.Motor_B_Pin1 = settings.MOTOR_B_PIN1
            self.Motor_B_Pin2 = settings.MOTOR_B_PIN2

            self.Dir_forward = 0
            self.Dir_backward = 1

            self.pwm_freq = settings.PWM_FREQUENCY_ENGINE
            self.pwm_A = None
            self.pwm_B = None

            self.setup()

            # Thread control variables
            self.__flag = threading.Event()
            self.__flag.clear()
            self.mode = 'none'
            self.speed = 0
            self.mc = False

            self.initialized = True

    def pause(self):
        """
        Pause the current movement.
        """
        self.mode = 'none'
        self.mc = False
        self.motor_stop()
        self.__flag.clear()

    def resume(self):
        """
        Resume the current movement.
        """
        self.__flag.set()
        self.mc = True
        self.log_state_change()

    def setup(self):
        """
        Setup the GPIO pins and initialize PWM for the motors.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)

        self.motor_stop()

        self.pwm_A = GPIO.PWM(self.Motor_A_EN, self.pwm_freq)
        self.pwm_B = GPIO.PWM(self.Motor_B_EN, self.pwm_freq)

    def motor_stop(self):
        """
        Stop both motors.
        """
        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)

    def __motor_A(self, direction, speed):
        """
        Control motor A.

        :param direction: Direction of the motor (forward or backward).
        :param speed: Speed of the motor (0-100).
        """
        if direction == self.Dir_backward:
            GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        elif direction == self.Dir_forward:
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)

        self.pwm_A.start(100)
        self.pwm_A.ChangeDutyCycle(speed)

    def __motor_B(self, direction, speed):
        """
        Control motor B.

        :param direction: Direction of the motor (forward or backward).
        :param speed: Speed of the motor (0-100).
        """
        if direction == self.Dir_forward:
            GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        elif direction == self.Dir_backward:
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)

        self.pwm_B.start(100)
        self.pwm_B.ChangeDutyCycle(speed)
    
    def log_state_change(self):
        """
        Log and print the current state change of the robot.
        """
        print(f"Robot state changed: Speed={self.speed}, Mode={self.mode}")

    def move(self, speed, direction):
        """
        Move the robot in the specified direction.

        :param speed: Speed of the motors (0-100).
        :param direction: Direction to move ('forward', 'backward', 'none').
        """
        
        self.speed = speed
        self.mode = direction
        self.mc = False
        self.resume()

    def forward_processing(self):
        """
        Processing logic for forward movement.
        """
        self.__motor_A(self.Dir_forward, self.speed)
        self.__motor_B(self.Dir_forward, self.speed)
        while self.mc:
            time.sleep(0.01)
        self.motor_stop()

    def backward_processing(self):
        """
        Processing logic for backward movement.
        """
        self.__motor_A(self.Dir_backward, self.speed)
        self.__motor_B(self.Dir_backward, self.speed)
        while self.mc:
            time.sleep(0.01)
        self.motor_stop()

    def mode_change(self):
        """
        Change the movement mode based on the current mode setting.
        """
        if self.mode == 'none':
            self.pause()
        elif self.mode == 'forward':
            self.forward_processing()
        elif self.mode == 'backward':
            self.backward_processing()

    def cleanup(self):
        """
        Cleanup the GPIO settings.
        """
        self.motor_stop()
        GPIO.cleanup()

    def run(self):
        """
        Run the thread to handle the movement process.
        """
        print("start")
        while True:
            self.__flag.wait()
            self.mode_change()


def main():
    robot = RobotMove()
    robot.start()  # Start the thread

    try:
        for i in range(0,100,10):
            print("Moving forward speed "+i)
            robot.move(i, 'forward')
            time.sleep(1)
        
        for i in range(0,100,10):
            print("Moving backward speed "+i)
            robot.move(i, 'backward')
            time.sleep(1)

        # Additional tests
        print("Stopping motors")
        robot.move(0, 'none')
        time.sleep(1)

        print("Test complete")

    except KeyboardInterrupt:
        print("Measurement stopped by user")
    finally:
        robot.cleanup()
        robot.join()  # Ensure the thread is properly terminated


if __name__ == '__main__':
    main()
