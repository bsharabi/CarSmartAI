from __future__ import division
import time
import RPi.GPIO as GPIO  # type: ignore
import Adafruit_PCA9685
import threading
import random
import settings

class ServoCtrl(threading.Thread):
    def __init__(self, pwm_freq=settings.PWM_FREQUENCY):
        """
        Initialize the ServoCtrl class with default and initial positions for the servos.

        :param pwm_freq: PWM frequency for servo control.
        """
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(pwm_freq)

        self.initPos = [settings.SERVO_INIT_POS] * 16
        self.goalPos = [settings.SERVO_INIT_POS] * 16
        self.nowPos = [settings.SERVO_INIT_POS] * 16
        self.bufferPos = [settings.SERVO_INIT_POS] * 16
        self.lastPos = [settings.SERVO_INIT_POS] * 16
        self.ingGoal = [settings.SERVO_INIT_POS] * 16
        self.maxPos = [settings.SERVO_MAX_POS] * 16
        self.minPos = [settings.SERVO_MIN_POS] * 16
        self.scSpeed = [0] * 16

        self.sc_direction = [1] * 16
        self.ctrlRangeMax = settings.SERVO_MAX_POS
        self.ctrlRangeMin = settings.SERVO_MIN_POS
        self.angleRange = 180

        self.scMode = 'auto'
        self.scTime = 2.0
        self.scSteps = 30
        self.scDelay = 0.037
        self.scMoveTime = 0.037

        self.goalUpdate = 0
        self.wiggleID = 0
        self.wiggleDirection = 1

        super(ServoCtrl, self).__init__()
        self.__flag = threading.Event()
        self.__flag.clear()

    def turn_left(self, coefficient=1):
        """
        Turn left by moving the appropriate servo.

        :param coefficient: Coefficient to adjust the range of movement.
        """
        position = settings.SERVO_INIT_POS + int(coefficient * settings.SERVO_RANGE)
        self.set_servo_position(2, position)

    def turn_right(self, coefficient=1):
        """
        Turn right by moving the appropriate servo.

        :param coefficient: Coefficient to adjust the range of movement.
        """
        position = settings.SERVO_INIT_POS - int(coefficient * settings.SERVO_RANGE)
        self.set_servo_position(2, position)

    def turn_middle(self):
        """
        Turn the servo to the middle position.
        """
        self.set_servo_position(2, settings.SERVO_INIT_POS)

    def set_servo_position(self, servo_id, position):
        """
        Set the position of a specific servo.

        :param servo_id: ID of the servo.
        :param position: Position to set the servo to.
        """
        position = self.control_range(position, settings.SERVO_MAX_POS, settings.SERVO_MIN_POS)
        self.nowPos[servo_id] = position
        self.pwm.set_pwm(servo_id, 0, position)

    def radar_scan(self, ultrasonic_sensor):
        """
        Perform a radar scan by moving the servo and taking distance measurements.

        :param ultrasonic_sensor: An instance of UltrasonicSensor to measure distances.
        :return: A list of distance measurements.
        """
        results = []
        self.set_servo_position(1, settings.SERVO_MAX_POS)
        time.sleep(0.5)
        results.append(ultrasonic_sensor.get_distance())

        while self.nowPos[1] > settings.SERVO_MIN_POS:
            self.nowPos[1] -= 1
            self.set_servo_position(1, self.nowPos[1])
            results.append(ultrasonic_sensor.get_distance())

        self.set_servo_position(1, settings.SERVO_INIT_POS)
        return results

    def get_direction(self, servo_id):
        """
        Get the current direction of a specific servo.

        :param servo_id: ID of the servo.
        :return: The direction of the servo.
        """
        return abs(self.goalPos[servo_id] - self.initPos[servo_id])

    def pause(self):
        """
        Pause the servo movement.
        """
        self.__flag.clear()

    def resume(self):
        """
        Resume the servo movement.
        """
        self.__flag.set()

    def control_range(self, value, max_value, min_value):
        """
        Control the value within the specified range.

        :param value: The value to control.
        :param max_value: The maximum allowed value.
        :param min_value: The minimum allowed value.
        :return: The controlled value.
        """
        return min(max_value, max(min_value, value))

    def move_angle(self, servo_id, angle):
        """
        Move the servo to a specific angle.

        :param servo_id: ID of the servo.
        :param angle: The angle to move the servo to.
        """
        position = self.initPos[servo_id] + int(self.sc_direction[servo_id] * self.angle_to_pwm(angle))
        position = self.control_range(position, self.maxPos[servo_id], self.minPos[servo_id])
        self.set_servo_position(servo_id, position)

    def angle_to_pwm(self, angle):
        """
        Convert an angle to a PWM value.

        :param angle: The angle to convert.
        :return: The PWM value corresponding to the angle.
        """
        return int(round(((self.ctrlRangeMax - self.ctrlRangeMin) / self.angleRange) * angle, 0))

    def move_init(self):
        """
        Move all servos to their initial positions.
        """
        self.scMode = 'init'
        for i in range(16):
            self.pwm.set_pwm(i, 0, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def init_config(self, servo_id, init_position, move_to=False):
        """
        Initialize the configuration of a specific servo.

        :param servo_id: Servo ID.
        :param init_position: Initial position input.
        :param move_to: Move to the initial position if True.
        """
        if self.minPos[servo_id] < init_position < self.maxPos[servo_id]:
            self.initPos[servo_id] = init_position
            if move_to:
                self.pwm.set_pwm(servo_id, 0, self.initPos[servo_id])
        else:
            print('initPos Value Error.')

    def move_servo_init(self, servo_ids):
        """
        Move specific servos to their initial positions.

        :param servo_ids: List of servo IDs.
        """
        self.scMode = 'init'
        for i in servo_ids:
            self.pwm.set_pwm(i, 0, self.initPos[i])
            self.lastPos[i] = self.initPos[i]
            self.nowPos[i] = self.initPos[i]
            self.bufferPos[i] = float(self.initPos[i])
            self.goalPos[i] = self.initPos[i]
        self.pause()

    def pos_update(self):
        """
        Update the current positions of the servos.
        """
        self.goalUpdate = 1
        for i in range(16):
            self.lastPos[i] = self.nowPos[i]
        self.goalUpdate = 0

    def speed_update(self, servo_ids, speeds):
        """
        Update the speed of specific servos.

        :param servo_ids: List of servo IDs.
        :param speeds: List of speeds corresponding to the servo IDs.
        """
        for i in range(len(servo_ids)):
            self.scSpeed[servo_ids[i]] = speeds[i]

    def move_auto(self):
        """
        Move the servos to their goal positions automatically.
        """
        for i in range(16):
            self.ingGoal[i] = self.goalPos[i]

        for step in range(self.scSteps):
            for dc in range(16):
                if not self.goalUpdate:
                    self.nowPos[dc] = int(round(self.lastPos[dc] + ((self.goalPos[dc] - self.lastPos[dc]) / self.scSteps) * (step + 1), 0))
                    self.pwm.set_pwm(dc, 0, self.nowPos[dc])

                if self.ingGoal != self.goalPos:
                    self.pos_update()
                    time.sleep(self.scTime / self.scSteps)
                    return 1
            time.sleep((self.scTime / self.scSteps - self.scMoveTime))

        self.pos_update()
        self.pause()
        return 0

    def move_cert(self):
        """
        Move the servos to their goal positions with certain speed.
        """
        for i in range(16):
            self.ingGoal[i] = self.goalPos[i]
            self.bufferPos[i] = self.lastPos[i]

        while self.nowPos != self.goalPos:
            for i in range(16):
                if self.lastPos[i] < self.goalPos[i]:
                    self.bufferPos[i] += self.angle_to_pwm(self.scSpeed[i]) / (1 / self.scDelay)
                    new_now = int(round(self.bufferPos[i], 0))
                    if new_now > self.goalPos[i]: new_now = self.goalPos[i]
                    self.nowPos[i] = new_now
                elif self.lastPos[i] > self.goalPos[i]:
                    self.bufferPos[i] -= self.angle_to_pwm(self.scSpeed[i]) / (1 / self.scDelay)
                    new_now = int(round(self.bufferPos[i], 0))
                    if new_now < self.goalPos[i]: new_now = self.goalPos[i]
                    self.nowPos[i] = new_now

                if not self.goalUpdate:
                    self.pwm.set_pwm(i, 0, self.nowPos[i])

                if self.ingGoal != self.goalPos:
                    self.pos_update()
                    return 1
            self.pos_update()
            time.sleep(self.scDelay - self.scMoveTime)

        self.pause()
        return 0

    def pwm_gen_out(self, angle):
        """
        Generate PWM output for the given angle.

        :param angle: Angle input.
        :return: PWM output value.
        """
        return int(round(((self.ctrlRangeMax - self.ctrlRangeMin) / self.angleRange) * angle, 0))

    def set_auto_time(self, auto_speed):
        """
        Set the automatic movement time.

        :param auto_speed: Automatic movement time.
        """
        self.scTime = auto_speed

    def set_delay(self, delay):
        """
        Set the delay between steps.

        :param delay: Delay between steps.
        """
        self.scDelay = delay

    def auto_speed(self, servo_ids, angles):
        """
        Set the automatic speed for the servos.

        :param servo_ids: List of servo IDs.
        :param angles: List of angle inputs corresponding to the servo IDs.
        """
        self.scMode = 'auto'
        self.goalUpdate = 1
        for i in range(len(servo_ids)):
            new_goal = self.initPos[servo_ids[i]] + self.pwm_gen_out(angles[i]) * self.sc_direction[servo_ids[i]]
            if new_goal > self.maxPos[servo_ids[i]]: new_goal = self.maxPos[servo_ids[i]]
            elif new_goal < self.minPos[servo_ids[i]]: new_goal = self.minPos[servo_ids[i]]
            self.goalPos[servo_ids[i]] = new_goal
        self.goalUpdate = 0
        self.resume()

    def cert_speed(self, servo_ids, angles, speeds):
        """
        Set the certain speed for the servos.

        :param servo_ids: List of servo IDs.
        :param angles: List of angle inputs corresponding to the servo IDs.
        :param speeds: List of speeds corresponding to the servo IDs.
        """
        self.scMode = 'certain'
        self.goalUpdate = 1
        for i in range(len(servo_ids)):
            new_goal = self.initPos[servo_ids[i]] + self.pwm_gen_out(angles[i]) * self.sc_direction[servo_ids[i]]
            if new_goal > self.maxPos[servo_ids[i]]: new_goal = self.maxPos[servo_ids[i]]
            elif new_goal < self.minPos[servo_ids[i]]: new_goal = self.minPos[servo_ids[i]]
            self.goalPos[servo_ids[i]] = new_goal
        self.speed_update(servo_ids, speeds)
        self.goalUpdate = 0
        self.resume()

    def move_wiggle(self):
        """
        Move the servo in a wiggle motion.
        """
        self.bufferPos[self.wiggleID] += self.wiggleDirection * self.sc_direction[self.wiggleID] * self.pwm_gen_out(self.scSpeed[self.wiggleID]) / (1 / self.scDelay)
        new_now = int(round(self.bufferPos[self.wiggleID], 0))
        if self.bufferPos[self.wiggleID] > self.maxPos[self.wiggleID]: self.bufferPos[self.wiggleID] = self.maxPos[self.wiggleID]
        elif self.bufferPos[self.wiggleID] < self.minPos[self.wiggleID]: self.bufferPos[self.wiggleID] = self.minPos[self.wiggleID]
        self.nowPos[self.wiggleID] = new_now
        self.lastPos[self.wiggleID] = new_now
        if self.bufferPos[self.wiggleID] < self.maxPos[self.wiggleID] and self.bufferPos[self.wiggleID] > self.minPos[self.wiggleID]:
            self.pwm.set_pwm(self.wiggleID, 0, self.nowPos[self.wiggleID])
        else:
            self.stop_wiggle()
        time.sleep(self.scDelay - self.scMoveTime)

    def stop_wiggle(self):
        """
        Stop the wiggle motion.
        """
        self.pause()
        self.pos_update()

    def single_servo(self, servo_id, direction, speed):
        """
        Move a single servo in a wiggle motion.

        :param servo_id: Servo ID.
        :param direction: Direction input.
        :param speed: Speed of the servo.
        """
        self.wiggleID = servo_id
        self.wiggleDirection = direction
        self.scSpeed[servo_id] = speed
        self.scMode = 'wiggle'
        self.pos_update()
        self.resume()

    def set_pwm(self, servo_id, pwm_value):
        """
        Set the PWM value for a specific servo.

        :param servo_id: Servo ID.
        :param pwm_value: PWM input value.
        """
        self.lastPos[servo_id] = pwm_value
        self.nowPos[servo_id] = pwm_value
        self.bufferPos[servo_id] = float(pwm_value)
        self.goalPos[servo_id] = pwm_value
        self.pwm.set_pwm(servo_id, 0, pwm_value)
        self.pause()

    def run(self):
        """
        Run the servo control thread.
        """
        while True:
            self.__flag.wait()
            self.sc_move()

    def sc_move(self):
        """
        Perform the servo movement based on the current mode.
        """
        if self.scMode == 'init':
            self.move_init()
        elif self.scMode == 'auto':
            self.move_auto()
        elif self.scMode == 'certain':
            self.move_cert()
        elif self.scMode == 'wiggle':
            self.move_wiggle()

def main():
    sc = ServoCtrl()
    sc.start()

    try:
        # Test turn_left function
        print("Turning left")
        sc.turn_left()
        time.sleep(1)

        # Test turn_right function
        print("Turning right")
        sc.turn_right()
        time.sleep(1)

        # # Test turn_middle function
        # print("Turning to middle")
        # sc.turn_middle()
        # time.sleep(1)

        # # Test radar_scan function (assuming UltrasonicSensor class is defined)
        # from UltrasonicSensor import UltrasonicSensor
        # sensor = UltrasonicSensor(settings.ULTRASONIC_TR_PIN, settings.ULTRASONIC_EC_PIN)
        # print("Radar scanning")
        # scan_results = sc.radar_scan(sensor)
        # print("Scan results:", scan_results)
        
        # # Test get_direction function
        # print("Getting direction of servo 1")
        # direction = sc.get_direction(1)
        # print("Direction:", direction)

        # # Test move_angle function
        # print("Moving servo 1 to 45 degrees")
        # sc.move_angle(1, 45)
        # time.sleep(1)

        # # Test single_servo function
        # print("Wiggling servo 1")
        # sc.single_servo(1, 1, 10)
        # time.sleep(2)
        # sc.stop_wiggle()

        # # Test auto_speed function
        # print("Moving servo 1 to 30 degrees with auto speed")
        # sc.auto_speed([1], [30])
        # time.sleep(2)

        # # Test cert_speed function
        # print("Moving servo 1 to 60 degrees with certain speed")
        # sc.cert_speed([1], [60], [5])
        # time.sleep(2)

    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        sc.pause()

if __name__ == '__main__':
    main()
