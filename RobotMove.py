import time
import RPi.GPIO as GPIO # type: ignore
import settings

class RobotMove:
    
    def __init__(self):
        """
        Initialize the RobotMove class with motor pins and PWM frequency.

        :param pwm_freq: PWM frequency for motor speed control.
        """
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

    def motor_A(self, direction, speed):
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

    def motor_B(self, direction, speed):
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

    def move(self, speed, direction, turn=None, radius=0.6):
        """
        Move the robot in the specified direction with optional turning.

        :param speed: Speed of the motors (0-100).
        :param direction: Direction to move ('forward', 'backward', 'no').
        :param turn: Optional turning direction ('left', 'right').
        :param radius: Turning radius (0 < radius <= 1).
        """
        if direction == 'forward':
            if turn == 'right':
                self.motor_B(self.Dir_backward, int(speed * radius))
                self.motor_A(self.Dir_forward, speed)
            elif turn == 'left':
                self.motor_B(self.Dir_forward, speed)
                self.motor_A(self.Dir_backward, int(speed * radius))
            else:
                self.motor_A(self.Dir_forward, speed)
                self.motor_B(self.Dir_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                self.motor_B(self.Dir_forward, int(speed * radius))
                self.motor_A(self.Dir_backward, speed)
            elif turn == 'left':
                self.motor_B(self.Dir_backward, speed)
                self.motor_A(self.Dir_forward, int(speed * radius))
            else:
                self.motor_A(self.Dir_backward, speed)
                self.motor_B(self.Dir_backward, speed)
        elif direction == 'no':
            self.motor_stop()

    def cleanup(self):
        """
        Cleanup the GPIO settings.
        """
        self.motor_stop()
        GPIO.cleanup()


def main():
    robot = RobotMove()

    try:
        print("Moving forward")
        robot.motor_A(1,50)
        # robot.move(100, 'forward')
        # time.sleep(1.3)
        # robot.motor_stop()
        
        # print("Moving Backward")
        # robot.move(100, 'backward')
        time.sleep(1.3)
        robot.motor_stop()
       

    except KeyboardInterrupt:
        print("Measurement stopped by user")
    finally:
        robot.cleanup()


if __name__ == '__main__':
    main()

