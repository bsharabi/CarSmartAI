import RPi.GPIO as GPIO
import time
from ServoCtrl import ServoCtrl
from RobotMove import RobotMove

line_pin_left = 19
line_pin_middle = 16
line_pin_right = 20
servo_ctrl = ServoCtrl()
robot_move = RobotMove()
robot_move.start()
servo_ctrl.start()
def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)
    #motor.setup()

# led = LED.LED()
turn_status = 0
speed = 75
angle_rate = 1
color_select = 1 # 0 --> white line / 1 --> black line
check_true_out = 0
backing = 0
last_turn = 0
def run():
    global turn_status, speed, angle_rate, color_select, check_true_out, backing, last_turn
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    print('R%d   M%d   L%d'%(status_right,status_middle,status_left))
    
    if status_right == color_select:
        check_true_out = 0
        backing = 0
        print('left')
        # led.colorWipe(0,255,0)
        turn_status = -1
        last_turn = -1
        servo_ctrl.turnLeft(angle_rate)
        robot_move.move(speed, 'forward')
    elif status_left == color_select:
        check_true_out = 0
        backing = 0
        print('right')
        turn_status = 1
        last_turn = 1
        # led.colorWipe(0,0,255)
        servo_ctrl.turnRight(angle_rate)
        robot_move.move(speed, 'forward')

    elif status_middle == color_select:
        check_true_out = 0
        backing = 0
        print('middle')
        # led.colorWipe(255,255,255)
        turn_status = 0
        servo_ctrl.turnMiddle()
        robot_move.move(speed, 'forward')
        # time.sleep(0.2)
    
    else:
        print('none')
        # led.colorWipe(255,0,0)
        if not backing == 1:
            if check_true_out == 1:
                check_true_out = 0
                if turn_status == 0:
                    if last_turn == 1:
                        servo_ctrl.turnRight(angle_rate)
                    else:
                        servo_ctrl.turnLeft(angle_rate)
                    robot_move.move(speed, 'backward')
                    time.sleep(0.3)
                elif turn_status == 1:
                    time.sleep(0.3)
                    servo_ctrl.turnLeft(angle_rate)
                else:
                    time.sleep(0.3)
                    servo_ctrl.turnRight(angle_rate)
                robot_move.move(speed, 'backward')
                backing = 1
                # time.sleep(0.2)
            else:
                time.sleep(0.1)
                check_true_out = 1

if __name__ == '__main__':
    try:
        setup()
        # move.setup()
        
        while 1:
            run()
            
        pass
    except KeyboardInterrupt:
        # move.destroy()
        GPIO.cleanup()
        pass
