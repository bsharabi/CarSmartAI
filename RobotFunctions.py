import RPi.GPIO as GPIO
import time
from ServoCtrl import ServoCtrl
from RobotMove import RobotMove
from RobotLight import RobotLight
from UltrasonicSensor import UltrasonicSensor
import settings

class RobotFunctions:
    """
    A class to encapsulate the line following logic for the robot.
    """

    def __init__(self):
        self.line_pin_left = settings.LINE_PIN_LEFT
        self.line_pin_middle = settings.LINE_PIN_MIDDLE
        self.line_pin_right = settings.LINE_PIN_RIGHT
        
        self.distance_threshold = 70  # cm, distance threshold to consider an obstacle
        self.speed = 100  # initial speed
        self.scan_delay = 0.5  # delay between scans

        self.servo_ctrl = ServoCtrl()
        self.robot_move = RobotMove()
        self.robot_light = RobotLight()
        self.ultrasonic_sensor = UltrasonicSensor()

        self.turn_status = 0
        self.speed = settings.MOTOR_SPEED
        self.angle_rate = settings.ANGLE_RATE
        self.color_select = settings.COLOR_SELECT
        self.check_true_out = 0
        self.backing = 0
        self.last_turn = 0

        self.setup()

    def setup(self):
        """
        Setup the GPIO pins and start necessary components.
        """
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.line_pin_right, GPIO.IN)
        GPIO.setup(self.line_pin_middle, GPIO.IN)
        GPIO.setup(self.line_pin_left, GPIO.IN)

        self.robot_move.start()
        self.servo_ctrl.start()
        self.robot_light.start()
        self.ultrasonic_sensor.start()


    def find_line(self):
        """
        Core logic for the line following robot. Adjusts movement based on sensor input.
        """
        status_right = GPIO.input(self.line_pin_right)
        status_middle = GPIO.input(self.line_pin_middle)
        status_left = GPIO.input(self.line_pin_left)
        print(f'R{status_right}   M{status_middle}   L{status_left}')

        if status_right == self.color_select:
            self.__handle_right()
        elif status_left == self.color_select:
            self.__handle_left()
        elif status_middle == self.color_select:
            self.__handle_middle()
        else:
            self.__handle_none()

    def __handle_right(self):
        """
        Handle logic when the right sensor detects the line.
        """
        self.check_true_out = 0
        self.backing = 0
        print('left')

        self.robot_light.breath(0, 255, 0)
        self.turn_status = -1
        self.last_turn = -1
        self.servo_ctrl.turnLeft(self.angle_rate)
        self.robot_move.move(self.speed, 'forward')

    def __handle_left(self):
        """
        Handle logic when the left sensor detects the line.
        """
        self.check_true_out = 0
        self.backing = 0
        print('right')

        self.turn_status = 1
        self.last_turn = 1
        self.robot_light.breath(0, 0, 255)
        self.servo_ctrl.turnRight(self.angle_rate)
        self.robot_move.move(self.speed, 'forward')

    def __handle_middle(self):
        """
        Handle logic when the middle sensor detects the line.
        """
        self.check_true_out = 0
        self.backing = 0
        print('middle')

        self.robot_light.breath(255, 255, 255)
        self.turn_status = 0
        self.servo_ctrl.turnMiddle()
        self.robot_move.move(self.speed, 'forward')
        time.sleep(0.2)

    def __handle_none(self):
        """
        Handle logic when none of the sensors detect the line.
        """
        print('none')

        self.robot_light.breath(255, 0, 0)
        if not self.backing == 1:
            if self.check_true_out == 1:
                self.check_true_out = 0
                if self.turn_status == 0:
                    if self.last_turn == 1:
                        self.servo_ctrl.turnRight(self.angle_rate)
                    else:
                        self.servo_ctrl.turnLeft(self.angle_rate)
                    self.robot_move.move(self.speed, 'backward')
                    time.sleep(0.3)
                elif self.turn_status == 1:
                    time.sleep(0.3)
                    self.servo_ctrl.turnLeft(self.angle_rate)
                else:
                    time.sleep(0.3)
                    self.servo_ctrl.turnRight(self.angle_rate)
                self.robot_move.move(self.speed, 'backward')
                self.backing = 1
                time.sleep(0.2)
            else:
                time.sleep(0.1)
                self.check_true_out = 1

    def cleanup(self):
        """
        Clean up the GPIO settings and stop the robot.
        """
        self.robot_move.terminate()
        self.robot_light.pause()
        self.servo_ctrl.terminate()
        self.ultrasonic_sensor.terminate()
        
        self.robot_move.join()
        self.servo_ctrl.join()
        self.ultrasonic_sensor.join()
        
        GPIO.cleanup()

def main():
    """
    Main function to create the LineFollower object and run the line following logic.
    """
    robot = RobotFunctions()
    try:
        while True:
            robot.find_line()
    except KeyboardInterrupt:
        print("Line following stopped by user.")
    finally:
        robot.cleanup()

if __name__ == '__main__':
    main()
