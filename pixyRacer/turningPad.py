import pixyRacer
import time
import math
import numpy as np

class turningPad(object):
	def __init__(self, racerBot):
		# actual init stuff
		self.bot = racerBot
		# exercise 1: single wheel turn
		self.maxSpeed = 100
		self.a0 = [[-62.113, -53.056], [-51.201, -45.214]]
		self.b0 = [[4.926, 4.890], [5.123, 4.836]]
		# exercise 2: symmetric  turn
		self.frictionCorrection = 0
		# maximum turn rate of the wheels that we want is 95% of the slowest wheel
		# because of non-linearities at the extremes
		self.maxT = 95 
		self.maxTurnRate_SW = 0
		# exercise 3: straight lines
		self.maxTurnRate_SYM = 0
		self.wheelDiameter = 0.066
		self.motorCorrection = [0, 0]

		self.maxTurnRate_SW = min(	(self.a0[0][0] + self.b0[0][0]*self.maxT), 
									(self.a0[0][1] + self.b0[0][1]*self.maxT), 
							   		(self.a0[1][0] + self.b0[1][0]*self.maxT), 
							   		(self.a0[1][1] + self.b0[1][1]*self.maxT))


	# runTime	- amount of time ([s]) to turn for
	# speed 	- percentage of max speed to apply to the wheel (0-100)
	# side 		- the motor on which side should be activated? ("left" or "right")
	def singleWheelTurn(self, speed, side, runTime):

		if abs(speed) > self.maxSpeed:
			speed = (speed > 0) * self.maxSpeed 

		if side == "right":
			self.bot.setMotorSpeeds(speed, 0)
		elif side == "left":
			self.bot.setMotorSpeeds(0, speed)

		t_start = time.time()
		t_end = t_start + runTime		

		while time.time() < t_end:
			pass
			
		self.bot.setMotorSpeeds(0, 0)

		return

	# turningRate 	- percentage of maximum rate at which to spin the robot ((-1)-(+1))
	# runTime 		- (optional) amount of time ([s]) to run motors for
	#				-  if not set, motors will run until the next command is issued
	def symmetricTurn(self, turningRate, runTime=None):
		self.bot.setTurnRates(turningRate, turningRate, runTime)
		return

	# speed 	- speed ([ms^(-1)]) to set the robot to
	# runTime 	- (optional) amount of time ([s]) to run motors for
	#			-  if not set, motors will run until the next command is issued
	# TODO: I expect the difference between the expected and real symmetric turn rates to vary proportionally. 
	# test this and add stuff to the code to correct the turn rates
	def straightLine(self, speed, correction=[0, 0], runTime=None):
		lSpeed = (1.0+correction[0])*speed
		rSpeed = -(1.0+correction[1])*speed
		
		self.bot.setTurnRates(lSpeed, rSpeed, runTime)


	def deadReckoningTrack(self, timePoints, motorCommands):

		if not (len(timePoints) == len(motorCommands)):
			print "timePoints and motorCommands must have same length!"
			return

		self.bot.setServoPosition(0)

		t_start = time.time()

		for i in range(len(timePoints)):
			self.bot.setTurnRates(motorCommands[i][0], motorCommands[i][1])
			# self.bot.setMotorSpeeds(motorCommands[i][0], motorCommands[i][1])

			while (time.time() - t_start) < sum(timePoints[0:(i+1)]):
				pass

		return
