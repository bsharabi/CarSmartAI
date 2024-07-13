import time
import RPi.GPIO as GPIO # type: ignore
import threading
import Adafruit_PCA9685
import os
import UltrasonicSensor
import Kalman_filter

from RobotMove import RobotMove
from ServoCtrl import ServoCtrl

robot_move = RobotMove()
sc = ServoCtrl()

kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)






line_pin_right = 20
line_pin_middle = 16
line_pin_left = 19

def pwmGenOut(angleInput):
	return int(round(23/9*angleInput))


def setup():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(line_pin_right,GPIO.IN)
	GPIO.setup(line_pin_middle,GPIO.IN)
	GPIO.setup(line_pin_left,GPIO.IN)


class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'
		self.steadyGoal = 0

		self.scanNum = 3
		self.scanList = [0,0,0]
		self.scanPos = 1
		self.scanDir = 1
		self.rangeKeep = 0.7
		self.scanRange = 100
		self.scanServo = 1
		self.turnServo = 2
		self.turnWiggle = 200

		setup()

		super(Functions, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	def radarScan(self):
		return sc.radar_scan()


	def pause(self):
		self.functionMode = 'none'
		robot_move.motorStop()
		self.__flag.clear()


	def resume(self):
		self.__flag.set()


	def automatic(self):
		self.functionMode = 'Automatic'
		self.resume()


	def trackLine(self):
		self.functionMode = 'trackLine'
		self.resume()


	def keepDistance(self):
		self.functionMode = 'keepDistance'
		self.resume()


	def steady(self,goalPos):
		self.functionMode = 'Steady'
		self.steadyGoal = goalPos
		self.resume()


	def speech(self):
		self.functionMode = 'speechRecProcessing'
		self.resume()


	def trackLineProcessing(self):
		status_right = GPIO.input(line_pin_right)
		status_middle = GPIO.input(line_pin_middle)
		status_left = GPIO.input(line_pin_left)
		#print('R%d   M%d   L%d'%(status_right,status_middle,status_left))
		if status_middle == 0:
			sc.moveAngle(2, 0)
			robot_move.move(80,'forward')
			
		elif status_left == 0:
			sc.moveAngle(2, 30)
			robot_move.move(80,'forward')
			
		elif status_right == 0:
			sc.moveAngle(2,-30)
			robot_move.move(80,'forward')
			
		else:
			robot_move.move(0,'forward')
		print(status_left,status_middle,status_right)
		time.sleep(0.1)


	def automaticProcessing(self):
		print('automaticProcessing')

		sc.moveAngle(2, 0)
		if self.scanPos == 1:
			pwm.set_pwm(self.scanServo, 0, sc.initPos[1]+self.scanRange)
			time.sleep(0.3)
			self.scanList[0] = UltrasonicSensor.checkdist()
		elif self.scanPos == 2:
			pwm.set_pwm(self.scanServo, 0, sc.initPos[1])
			time.sleep(0.3)
			self.scanList[1] = UltrasonicSensor.checkdist()
		elif self.scanPos == 3:
			pwm.set_pwm(self.scanServo, 0, sc.initPos[1]-self.scanRange)
			time.sleep(0.3)
			self.scanList[2] = UltrasonicSensor.checkdist()

		self.scanPos = self.scanPos + self.scanDir

		if self.scanPos > self.scanNum or self.scanPos < 1:
			if self.scanDir == 1:self.scanDir = -1
			elif self.scanDir == -1:self.scanDir = 1
			self.scanPos = self.scanPos + self.scanDir*2
		print(self.scanList)

		if min(self.scanList) < self.rangeKeep:
			if self.scanList.index(min(self.scanList)) == 0:
				sc.moveAngle(2, -30)
			elif self.scanList.index(min(self.scanList)) == 1:
				if self.scanList[0] < self.scanList[2]:
					sc.moveAngle(2, -45)
				else:
					sc.moveAngle(2, 45)
			elif self.scanList.index(min(self.scanList)) == 2:
				sc.moveAngle(2, 30)
			if max(self.scanList) < self.rangeKeep or min(self.scanList) < self.rangeKeep/3:
				robot_move.move(80,'backward')
				
		else:
			#move along
			robot_move.move(80,'forward')
			
			pass

	
# Filter out occasional incorrect distance data.
	def distRedress(self): 
		mark = 0
		distValue = UltrasonicSensor.checkdist()* 100
		while True:
			distValue = UltrasonicSensor.checkdist()* 100
			if distValue > 900:
				mark +=  1
			elif mark > 5 or distValue < 900:
					break
			print(distValue)
		return round(distValue,2)

	def automaticProcessing(self):
		# print('automaticProcessing')
		sc.moveAngle(1, 0)
		dist = self.distRedress()
		print(dist, "cm")
		time.sleep(0.2)
		if dist >= 40:			# More than 40CM, go straight.
			sc.moveAngle(2, 0)
			robot_move.move(80,'forward')
			
			print("Forward")
		# More than 20cm and less than 40cm, detect the distance between the left and right sides.
		elif dist > 20 and dist < 50:	
			robot_move.move(0,'backward')
			sc.moveAngle(1, 30)
			time.sleep(0.3)
			distLeft = self.distRedress()
			self.scanList[0] = distLeft

			# Go in the direction where the detection distance is greater.
			sc.moveAngle(1, -30)
			time.sleep(0.3)
			distRight = self.distRedress()
			self.scanList[1] = distRight
			print(self.scanList)
			sc.moveAngle(1, 0)
			if self.scanList[0] >= self.scanList[1]:
				sc.moveAngle(2, 30)
				time.sleep(0.3)
				robot_move.move(80,'forward')
				
				print("Left")
				time.sleep(1)
			else:
				sc.moveAngle(2, -30)
				time.sleep(0.3)
				robot_move.move(80,'forward')
				
				print("Right")
				time.sleep(1)
		else:		# The distance is less than 20cm, back.
			robot_move.move(80,'backward')
			
			print("Back")
			time.sleep(1)


	def steadyProcessing(self):
		print('steadyProcessing')
		xGet = sensor.get_accel_data()
		xGet = xGet['x']
		xOut = kalman_filter_X.kalman(xGet)
		pwm.set_pwm(2, 0, self.steadyGoal+pwmGenOut(xOut*9))
		# pwm.set_pwm(2, 0, self.steadyGoal+pwmGenOut(xGet*10))
		time.sleep(0.05)


	# def speechRecProcessing(self):
	# 	print('speechRecProcessing')
	# 	speech.run()


	def keepDisProcessing(self):
		print('keepDistanceProcessing')
		distanceGet = UltrasonicSensor.checkdist()
		if distanceGet > (self.rangeKeep/2+0.1):
			robot_move.move(80,'forward')
			
		elif distanceGet < (self.rangeKeep/2-0.1):
			robot_move.move(80,'backward')
			
		else:
			robot_move.motorStop()


	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
		elif self.functionMode == 'Steady':
			self.steadyProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
		# elif self.functionMode == 'speechRecProcessing':
		# 	self.speechRecProcessing()
		elif self.functionMode == 'keepDistance':
			self.keepDisProcessing()


	def run(self):
		while 1:
			self.__flag.wait()
			self.functionGoing()
			pass


if __name__ == '__main__':
	pass
	try:
		fuc=Functions()
		# fuc.radarScan()
		# fuc.start()
		# fuc.automatic()
		fuc.trackLine()
		# # fuc.steady(300)
		# time.sleep(30)
		# fuc.pause()
		# time.sleep(1)
		# move.move(80, 'no', 'no', 0.5)
	except KeyboardInterrupt:
			robot_move.motorStop()
