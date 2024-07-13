
import keyboard
from RobotMove import RobotMove
from ServoCtrl import ServoCtrl
import settings
robot = RobotMove()
robot.start()
sc = ServoCtrl()
sc.start()
speed = 50
angel = 50
wheel=0
def print_pressed_key(event):
    global speed, angel,wheel
    print("speed: ", speed)
    print("angel: ", angel)
    print("wheel: ", wheel)

    if event.name == '+':
        if speed<100:
            speed+=10
        else:
            speed =100
    if event.name == '-':
        if speed>0:
            speed-=10
        else:
            speed =0
    if event.name == '0':
        if angel<100:
            angel+=10
        else:
            angel =100
    if event.name == '1':
        if angel>0:
            angel-=10
        else:
            angel =0

    if event.name == '7':
        if wheel<100:
            wheel+=10
        else:
            wheel =100
    if event.name == '8':
        if wheel>0:
            wheel-=10
        else:
            wheel =0
    if event.name == 'w':
        robot.move(speed, 'forward')
    if event.name == 's':
        robot.move(speed, 'backward')
    if event.name == 'a':
        sc.moveAngle(settings.SERVO_WHEEL, -wheel)
    if event.name == 'd':
        sc.moveAngle(settings.SERVO_WHEEL, wheel)
    if event.name == 'm':
        sc.moveInit()
        #sc.moveAngle(settings.SERVO_WHEEL, 50)
        #sc.moveAngle(settings.SERVO_HEAD, 50)
        #sc.moveAngle(settings.SERVO_MID_HEAD, 50)

    if event.name == '4':
        robot.move(0, 'none')

    if event.name == 'i':
        sc.moveAngle(settings.SERVO_HEAD, angel)
    if event.name == 'k':
        sc.moveAngle(settings.SERVO_HEAD, -angel)
    if event.name == 'l':
        sc.moveAngle(settings.SERVO_MID_HEAD, angel)
    if event.name == 'j':
        sc.moveAngle(settings.SERVO_MID_HEAD, -angel)
    if event.name == 't':
        print("pwmToAngle ",sc.pwmGenOut(angel))
        print("-pwmToAngle ",sc.pwmGenOut(-angel))


    print(f'Key {event.name} pressed')
keyboard.on_press(print_pressed_key)

print("Press any key (Press ESC to exit)")

# Keep the program running to capture keyboard events
keyboard.wait('esc')