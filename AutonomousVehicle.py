import time
from RobotMove import RobotMove
from RobotLight import RobotLight
from UltrasonicSensor import UltrasonicSensor
from ServoCtrl import ServoCtrl

class AutonomousVehicle:
    def __init__(self):
        self.robot_move = RobotMove()
        self.robot_light = RobotLight()
        self.ultrasonic_sensor = UltrasonicSensor()
        self.servo_ctrl = ServoCtrl()

        self.distance_threshold = 30  # cm, distance threshold to consider an obstacle
        self.speed = 50  # initial speed
        self.scan_delay = 0.5  # delay between scans

    def start(self):
        """
        Start the threads and initiate the autonomous driving.
        """
        self.robot_move.start()
        self.robot_light.start()
        self.ultrasonic_sensor.start()
        self.servo_ctrl.start()
        
        self.robot_light.setColor(0, 255, 0)  # Set lights to green to indicate start
        time.sleep(2)  # Wait for all systems to start

        try:
            self.autonomous_drive()
        except KeyboardInterrupt:
            print("Autonomous driving stopped by user.")
        finally:
            self.cleanup()

    def cleanup(self):
        """
        Stop all systems and clean up.
        """
        self.robot_move.terminate()
        self.robot_light.pause()
        self.ultrasonic_sensor.terminate()
        self.servo_ctrl.terminate()
        
        self.robot_move.join()
        self.ultrasonic_sensor.join()
        self.servo_ctrl.join()

    def autonomous_drive(self):
        """
        Main loop to perform autonomous driving with obstacle avoidance.
        """
        while True:
            front_distance = self.ultrasonic_sensor.get_distance()
            left_distance, right_distance, top_distance, bottom_distance = self.scan_surroundings()

            print(f"Distances - Front: {front_distance}, Left: {left_distance}, Right: {right_distance}, Top: {top_distance}, Bottom: {bottom_distance}")

            if front_distance and front_distance < self.distance_threshold:
                print("Obstacle detected in front!")
                self.robot_move.pause()
                self.robot_light.setColor(255, 0, 0)  # Set lights to red to indicate obstacle

                if left_distance and right_distance:
                    if left_distance > right_distance:
                        self.servo_ctrl.turnLeft()
                        self.robot_move.move(self.speed, 'forward')
                    else:
                        self.servo_ctrl.turnRight()
                        self.robot_move.move(self.speed, 'forward')
                elif left_distance:
                    self.servo_ctrl.turnLeft()
                    self.robot_move.move(self.speed, 'forward')
                elif right_distance:
                    self.servo_ctrl.turnRight()
                    self.robot_move.move(self.speed, 'forward')
                else:
                    print("No clear path forward, moving backward.")
                    self.robot_move.move(self.speed, 'backward')
                    time.sleep(1)
                    self.robot_move.pause()
                    # Reduce speed to avoid collision when moving backward
                    self.robot_move.move(self.speed // 2, 'backward')

            else:
                print("Path clear, moving forward.")
                self.robot_move.move(self.speed, 'forward')
                self.robot_light.setColor(0, 255, 0)  # Set lights to green to indicate clear path

            time.sleep(self.scan_delay)

    def scan_surroundings(self):
        """
        Scan the surroundings using the servo and ultrasonic sensor.

        :return: Distances to the left, right, top, and bottom obstacles.
        """
        self.servo_ctrl.lookLeft(45)
        time.sleep(0.5)
        left_distance = self.ultrasonic_sensor.get_distance()

        self.servo_ctrl.lookRight(45)
        time.sleep(0.5)
        right_distance = self.ultrasonic_sensor.get_distance()

        self.servo_ctrl.lookUp(45)
        time.sleep(0.5)
        top_distance = self.ultrasonic_sensor.get_distance()

        self.servo_ctrl.lookDown(45)
        time.sleep(0.5)
        bottom_distance = self.ultrasonic_sensor.get_distance()

        self.servo_ctrl.ahead()

        return left_distance, right_distance, top_distance, bottom_distance

def main():
    av = AutonomousVehicle()
    av.start()

if __name__ == '__main__':
    main()
