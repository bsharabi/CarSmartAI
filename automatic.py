from UltrasonicSensor import UltrasonicSensor
import move 
import time
import Adafruit_PCA9685
import RPIservo
import os


curpath = os.path.relpath(__file__)
thisPath = os.path.dirname(curpath)

def num_import_int(initial):        
    global r
    with open( os.path.join( thisPath,"RPIservo.py")) as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                r=line
    begin=len(list(initial))
    snum=r[begin:]
    n=int(snum)
    return n


pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

pwm0_direction = 1
pwm0_init = num_import_int('init_pwm0 = ')
pwm0_max  = 520
pwm0_min  = 100
pwm0_pos  = pwm0_init

pwm1_direction = 1
pwm1_init = num_import_int('init_pwm1 = ')
pwm1_max  = 520
pwm1_min  = 100
pwm1_pos  = pwm1_init

pwm2_direction = 1
pwm2_init = num_import_int('init_pwm2 = ')
pwm2_max  = 520
pwm2_min  = 100
pwm2_pos  = pwm2_init

line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19


scGear = RPIservo.ServoCtrl()
scanNum = 3
scanPos = 1
scanDir = 1
scanList = [0,0,0]
scanServo = 1
scanRange = 100
rangeKeep = 0.7

sensor = UltrasonicSensor(trigger_pin=11, echo_pin=8)
move.setup()
try:
    while True:
        if scanPos == 1:
            pwm.set_pwm(scanServo, 0, pwm1_init+scanRange)
            time.sleep(0.3)
            scanList[0] = sensor.get_distance()
        elif scanPos == 2:
            pwm.set_pwm(scanServo, 0, pwm1_init)
            time.sleep(0.3)
            scanList[1] = sensor.get_distance()
        elif scanPos == 3:
            pwm.set_pwm(scanServo, 0, pwm1_init-scanRange)
            time.sleep(0.3)
            scanList[2] = sensor.get_distance()

        scanPos += scanDir

        if scanPos > scanNum or scanPos < 1:
            if scanDir == 1:scanDir = -1
            elif scanDir == -1:scanDir = 1
            scanPos += scanDir*2
        print(scanList)

        if min(scanList) < rangeKeep:
            if scanList.index(min(scanList)) == 0:
                scGear.moveAngle(2, -30)
            elif scanList.index(min(scanList)) == 1:
                if scanList[0] < scanList[2]:
                    scGear.moveAngle(2, -45)
                else:
                    scGear.moveAngle(2, 45)
            elif scanList.index(min(scanList)) == 2:
                scGear.moveAngle(2, 30)
            if max(scanList) < rangeKeep or min(scanList) < rangeKeep/3:
                move.motor_left(1, 1, 80)
                move.motor_right(1, 1, 80)
        else:
            #move along
            move.motor_left(1, 0, 80)
            move.motor_right(1, 0, 80)
            pass
except KeyboardInterrupt: 
    move.destroy()
        