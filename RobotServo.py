from __future__ import division
import time
import RPi.GPIO as GPIO  # type: ignore
import Adafruit_PCA9685
import threading
from UltrasonicSensor import UltrasonicSensor
from RobotLight import RobotLight
import settings

class ServoCtrl(threading.Thread):
    """
    A singleton class to control multiple servos using Adafruit PCA9685 PWM driver.
    This class extends threading.Thread to allow asynchronous control.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(ServoCtrl, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):  # To prevent reinitialization
            super(ServoCtrl, self).__init__()

            self.pwm = Adafruit_PCA9685.PCA9685()
            self.pwm.set_pwm_freq(settings.PWM_FREQUENCY)

            # Servo configuration
            self.initPos = [settings.SERVO_INIT_POS] * 3
            self.goalPos = [settings.SERVO_INIT_POS] * 3
            self.nowPos = [settings.SERVO_INIT_POS] * 3
            self.bufferPos = [float(settings.SERVO_INIT_POS)] * 3
            self.lastPos = [settings.SERVO_INIT_POS] * 3
            self.ingGoal = [settings.SERVO_INIT_POS] * 3
            self.maxPos = [settings.SERVO_MAX_POS] * 3
            self.minPos = [settings.SERVO_MIN_POS] * 3
            self.scSpeed = [0] * 3
            self.sc_direction = [1] * 3

            self.ctrlRangeMax = 560
            self.ctrlRangeMin = 100
            self.angleRange = 180

            self.scMode = 'auto'
            self.scTime = 2.0
            self.scSteps = 30
            self.scDelay = 0.037
            self.scMoveTime = 0.037

            self.goalUpdate = 0
            self.wiggleID = 0
            self.wiggleDirection = 1

            self.robot_light = RobotLight()
            self.robot_light.start()
            self.ultrasonic_sensor = UltrasonicSensor()
            self.ultrasonic_sensor.start()

            self.initialized = True
            self.__flag = threading.Event()
            self.__terminate = threading.Event()
            self.__flag.clear()

    def pause(self):
        """
        Pause the current movement.
        """
        self.__flag.clear()

    def resume(self):
        """
        Resume the current movement.
        """
        self.__flag.set()

    def moveInit(self):
        """
        Move all servos to their initial positions.
        """
        self.scMode = 'init'
        for i in range(3):
            self.pwm.set_pwm(i, 0, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def initConfig(self, ID, initInput, moveTo):
        """
        Initialize a specific servo configuration.

        :param ID: Servo ID
        :param initInput: Initial position
        :param moveTo: Move to the initial position if True
        """
        if self.minPos[ID] < initInput < self.maxPos[ID]:
            self.initPos[ID] = initInput
            if moveTo:
                self.pwm.set_pwm(ID, 0, self.initPos[ID])
        else:
            print('initPos Value Error.')

    def moveServoInit(self, ID):
        """
        Move specific servos to their initial positions.

        :param ID: List of Servo IDs
        """
        self.scMode = 'init'
        for i in ID:
            self.pwm.set_pwm(i, 0, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def posUpdate(self):
        """
        Update positions for all servos.
        """
        self.goalUpdate = 1
        for i in range(3):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def speedUpdate(self, IDinput, speedInput):
        """
        Update speed for specific servos.

        :param IDinput: List of Servo IDs
        :param speedInput: List of speeds for the servos
        """
        for i in range(len(IDinput)):
            self.scSpeed[IDinput[i]] = speedInput[i]

    def moveAuto(self):
        """
        Move servos automatically.
        """
        for i in range(3):
            self.ingGoal[i] = self.goalPos[i]

        for i in range(self.scSteps):
            for dc in range(3):
                if not self.goalUpdate:
                    self.nowPos[dc] = int(round((self.lastPos[dc] + (((self.goalPos[dc] - self.lastPos[dc]) / self.scSteps) * (i + 1))), 0))
                    self.pwm.set_pwm(dc, 0, self.nowPos[dc])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    time.sleep(self.scTime / self.scSteps)
                    return 1
            time.sleep((self.scTime / self.scSteps - self.scMoveTime))

        self.posUpdate()
        self.pause()
        return 0

    def moveCert(self):
        """
        Move servos to a certain position with specific speed.
        """
        for i in range(3):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(3):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow > self.goalPos[i]:
                        newNow = self.goalPos[i]
                    self.nowPos[i] = newNow
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.pwmGenOut(self.scSpeed[i]) / (1 / self.scDelay)
                    newNow = int(round(self.bufferPos[i], 0))
                    if newNow < self.goalPos[i]:
                        newNow = self.goalPos[i]
                    self.nowPos[i] = newNow

                if not self.goalUpdate:
                    self.pwm.set_pwm(i, 0, self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.posUpdate()
                    return 1
            self.posUpdate()
            time.sleep(self.scDelay - self.scMoveTime)

        else:
            self.pause()
            return 0

    def pwmGenOut(self, angleInput):
        """
        Generate PWM output for a given angle.

        :param angleInput: Angle input
        :return: PWM output
        """
        return int(round(((self.ctrlRangeMax - self.ctrlRangeMin) / self.angleRange * angleInput), 0))

    def setAutoTime(self, autoSpeedSet):
        """
        Set automatic speed time.

        :param autoSpeedSet: Speed time for automatic movement
        """
        self.scTime = autoSpeedSet

    def setDelay(self, delaySet):
        """
        Set delay time.

        :param delaySet: Delay time
        """
        self.scDelay = delaySet

    def autoSpeed(self, ID, angleInput):
        """
        Set automatic speed for servos.

        :param ID: List of Servo IDs
        :param angleInput: List of angle inputs
        """
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i]) * self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]:
                newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]:
                newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.goalUpdate = 0
        self.resume()

    def certSpeed(self, ID, angleInput, speedSet):
        """
        Set certain speed for servos.

        :param ID: List of Servo IDs
        :param angleInput: List of angle inputs
        :param speedSet: List of speeds
        """
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(len(ID)):
            newGoal = self.initPos[ID[i]] + self.pwmGenOut(angleInput[i]) * self.sc_direction[ID[i]]
            if newGoal > self.maxPos[ID[i]]:
                newGoal = self.maxPos[ID[i]]
            elif newGoal < self.minPos[ID[i]]:
                newGoal = self.minPos[ID[i]]
            self.goalPos[ID[i]] = newGoal
        self.speedUpdate(ID, speedSet)
        self.goalUpdate = 0
        self.resume()

    def moveWiggle(self):
        """
        Move servos in a wiggle pattern.
        """
        self.bufferPos[self.wiggleID] += self.wiggleDirection * self.sc_direction[self.wiggleID] * self.pwmGenOut(self.scSpeed[self.wiggleID]) / (1 / self.scDelay)
        newNow = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]:
            self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]:
            self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = newNow
        self.lastPos[self.wiggleID] = newNow
        if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
            self.pwm.set_pwm(self.wiggleID, 0, self.nowPos[self.wiggleID])
        else:
            self.stopWiggle()
        time.sleep(self.scDelay - self.scMoveTime)

    def stopWiggle(self):
        """
        Stop the wiggle movement.
        """
        self.pause()
        self.posUpdate()

    def singleServo(self, ID, direcInput, speedSet):
        """
        Move a single servo with specific direction and speed.

        :param ID: Servo ID
        :param direcInput: Direction input
        :param speedSet: Speed setting
        """
        self.wiggleID = ID
        self.wiggleDirection = direcInput
        self.scSpeed[ID] = speedSet
        self.scMode = 'wiggle'
        self.posUpdate()
        self.resume()

    def moveAngle(self, ID, angleInput):
        """
        Move a servo to a specific angle.

        :param ID: Servo ID
        :param angleInput: Angle input
        """
        self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID] * self.pwmGenOut(angleInput))
        if self.nowPos[ID] > self.maxPos[ID]:
            self.nowPos[ID] = self.maxPos[ID]
        elif self.nowPos[ID] < self.minPos[ID]:
            self.nowPos[ID] = self.minPos[ID]
        self.lastPos[ID] = self.nowPos[ID]
        self.pwm.set_pwm(ID, 0, self.nowPos[ID])

    def scMove(self):
        """
        Perform the servo movement based on the current mode.
        """
        if self.scMode == 'init':
            self.moveInit()
        elif self.scMode == 'auto':
            self.moveAuto()
        elif self.scMode == 'certain':
            self.moveCert()
        elif self.scMode == 'wiggle':
            self.moveWiggle()

    def setPWM(self, ID, PWM_input):
        """
        Set a specific PWM value for a servo.

        :param ID: Servo ID
        :param PWM_input: PWM value
        """
        self.lastPos[ID] = PWM_input
        self.nowPos[ID] = PWM_input
        self.bufferPos[ID] = float(PWM_input)
        self.goalPos[ID] = PWM_input
        self.pwm.set_pwm(ID, 0, PWM_input)
        self.pause()

    def run(self):
        """
        Run the thread to handle servo movement.
        """
        while not self.__terminate.is_set():
            self.__flag.wait()
            self.scMove()

    def terminate(self):
        """
        Terminate the thread safely.
        """
        self.__terminate.set()
        self.__flag.set()
        self.cleanup()

    def cleanup(self):
        """
        Clean up the GPIO and PWM settings.
        """
        GPIO.cleanup()
        self.pwm.set_all_pwm(0, 0)

    # Additional methods from the original script
    def turnLeft(self, coe=1):
        """
        Turn left.

        :param coe: Coefficient for turning
        """
        pwm2_pos = self.initPos[settings.SERVO_WHEEL] + int(coe * 100 * self.sc_direction[settings.SERVO_WHEEL])
        pwm2_pos = self.ctrl_range(pwm2_pos, self.maxPos[settings.SERVO_WHEEL], self.minPos[settings.SERVO_WHEEL])
        self.robot_light.turn_left()
        self.pwm.set_pwm(settings.SERVO_WHEEL, 0, pwm2_pos)

    def turnRight(self, coe=1):
        """
        Turn right.

        :param coe: Coefficient for turning
        """
        pwm2_pos = self.initPos[settings.SERVO_WHEEL] - int(coe * 100 * self.sc_direction[settings.SERVO_WHEEL])
        pwm2_pos = self.ctrl_range(pwm2_pos, self.maxPos[settings.SERVO_WHEEL], self.minPos[settings.SERVO_WHEEL])
        self.robot_light.turn_right()
        self.pwm.set_pwm(settings.SERVO_WHEEL, 0, pwm2_pos)

    def turnMiddle(self):
        """
        Turn to the middle position.
        """
        pwm2_pos = self.initPos[settings.SERVO_WHEEL]
        self.robot_light.both_on()
        self.pwm.set_pwm(settings.SERVO_WHEEL, 0, pwm2_pos)

    def setPWMDirect(self, num, pos):
        """
        Set a specific PWM value directly for a servo.

        :param num: Servo number
        :param pos: Position
        """
        self.pwm.set_pwm(num, 0, pos)
        self.initPos[num] = pos
        self.nowPos[num] = pos

    def radar_scan(self):
        """
        Perform a radar scan using the ultrasonic sensor.

        :return: Scan result as a string
        """
        scan_result = 'U: '
        scan_speed = 1
        pwm1_pos = self.initPos[settings.SERVO_MID_HEAD]
        self.robot_light.cyan()
        if self.sc_direction[settings.SERVO_MID_HEAD]:
            pwm1_pos = self.maxPos[settings.SERVO_MID_HEAD]
            self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)
            time.sleep(0.5)
            scan_result += str(self.ultrasonic_sensor.get_distance())
            scan_result += ' '
            while pwm1_pos > self.minPos[settings.SERVO_MID_HEAD]:
                pwm1_pos -= scan_speed
                self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)
                scan_result += str(self.ultrasonic_sensor.get_distance())
                scan_result += ' '
            self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, self.initPos[settings.SERVO_MID_HEAD])
            pwm1_pos = self.initPos[settings.SERVO_MID_HEAD]
        else:
            pwm1_pos = self.minPos[settings.SERVO_MID_HEAD]
            self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)
            time.sleep(0.5)
            scan_result += str(self.ultrasonic_sensor.get_distance())
            scan_result += ' '
            while pwm1_pos < self.maxPos[settings.SERVO_MID_HEAD]:
                pwm1_pos += scan_speed
                self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)
                scan_result += str(self.ultrasonic_sensor.get_distance())
                scan_result += ' '
            self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, self.initPos[settings.SERVO_MID_HEAD])
            pwm1_pos = self.initPos[settings.SERVO_MID_HEAD]
        self.robot_light.both_on()
        return scan_result

    def ctrl_range(self, raw, max_genout, min_genout):
        """
        Control the range of a given value.

        :param raw: Raw value
        :param max_genout: Maximum output
        :param min_genout: Minimum output
        :return: Controlled range value
        """
        if raw > max_genout:
            raw_output = max_genout
        elif raw < min_genout:
            raw_output = min_genout
        else:
            raw_output = raw
        return int(raw_output)

    def lookLeft(self, speed):
        """
        Look left.

        :param speed: Speed for looking left
        """
        pwm1_pos = self.initPos[settings.SERVO_MID_HEAD]
        pwm1_pos += speed * self.sc_direction[settings.SERVO_MID_HEAD]
        pwm1_pos = self.ctrl_range(pwm1_pos, self.maxPos[settings.SERVO_MID_HEAD], self.minPos[settings.SERVO_MID_HEAD])
        self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)

    def lookRight(self, speed):
        """
        Look right.

        :param speed: Speed for looking right
        """
        pwm1_pos = self.initPos[settings.SERVO_MID_HEAD]
        pwm1_pos -= speed * self.sc_direction[settings.SERVO_MID_HEAD]
        pwm1_pos = self.ctrl_range(pwm1_pos, self.maxPos[settings.SERVO_MID_HEAD], self.minPos[settings.SERVO_MID_HEAD])
        self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, pwm1_pos)

    def lookUp(self, speed):
        """
        Look up.

        :param speed: Speed for looking up
        """
        pwm0_pos = self.initPos[settings.SERVO_HEAD]
        pwm0_pos -= speed * self.sc_direction[settings.SERVO_HEAD]
        pwm0_pos = self.ctrl_range(pwm0_pos, self.maxPos[settings.SERVO_HEAD], self.minPos[settings.SERVO_HEAD])
        self.pwm.set_pwm(settings.SERVO_HEAD, 0, pwm0_pos)

    def lookDown(self, speed):
        """
        Look down.

        :param speed: Speed for looking down
        """
        pwm0_pos = self.initPos[settings.SERVO_HEAD]
        pwm0_pos += speed * self.sc_direction[settings.SERVO_HEAD]
        pwm0_pos = self.ctrl_range(pwm0_pos, self.maxPos[settings.SERVO_HEAD], self.minPos[settings.SERVO_HEAD])
        self.pwm.set_pwm(settings.SERVO_HEAD, 0, pwm0_pos)

    def servoInit(self):
        """
        Initialize all servos.
        """
        try:
            self.pwm.set_all_pwm(0, settings.SERVO_INIT_POS)
        except Exception:
            pass
        self.pwm.set_pwm(settings.SERVO_HEAD, 0, self.initPos[settings.SERVO_HEAD])
        self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, self.initPos[settings.SERVO_MID_HEAD])
        self.pwm.set_pwm(settings.SERVO_WHEEL, 0, self.initPos[settings.SERVO_WHEEL])

    def cleanAll(self):
        """
        Clean up all servo settings.
        """
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(settings.PWM_FREQUENCY)
        self.pwm.set_all_pwm(0, 0)

    def ahead(self):
        """
        Move servos to the ahead position.
        """
        self.pwm.set_pwm(settings.SERVO_MID_HEAD, 0, self.initPos[settings.SERVO_MID_HEAD])
        self.pwm.set_pwm(settings.SERVO_HEAD, 0, self.initPos[settings.SERVO_HEAD])

    def getDirection(self):
        """
        Get the current direction.

        :return: Direction
        """
        return self.initPos[settings.SERVO_MID_HEAD] - settings.SERVO_MID_HEAD

def main():
    """
    Main function to demonstrate the usage of ServoCtrl class.
    """
    sc = ServoCtrl()
    sc.start()
    
    # Example usage of various methods
    try:
        sc.moveAngle(settings.SERVO_HEAD, 30)
        time.sleep(2)
        sc.moveAngle(settings.SERVO_MID_HEAD, -30)
        time.sleep(2)
        sc.autoSpeed([settings.SERVO_HEAD, settings.SERVO_MID_HEAD], [45, -45])
        time.sleep(2)
        sc.certSpeed([settings.SERVO_HEAD, settings.SERVO_MID_HEAD], [30, -30], [5, 5])
        time.sleep(2)
        sc.singleServo(settings.SERVO_HEAD, 1, 5)
        time.sleep(6)
        sc.singleServo(settings.SERVO_HEAD, -1, 30)
        time.sleep(1)
        print(sc.radar_scan())
        sc.turnLeft()
        time.sleep(2)
        sc.turnRight()
        time.sleep(2)
        sc.turnMiddle()
    except KeyboardInterrupt:
        print("Measurement stopped by user")
    finally:
        sc.terminate()
        sc.join()  # Ensure the thread is properly terminated

if __name__ == '__main__':
    main()
