### ALBiR 2018-19 Spring
### Made by: HTL, LH, DK
### Last updated: 02/19 by DK

# Modified functions: laneFollowing(~), stopAtStationaryObstacles(~), avoidStationaryObstacles(~)
# Added function: steerTowards(objectID)
# Added function: steerAway(objectID)
# Added inputs, targetDist & targetAngle to function: stopAtStationaryObstacles(speed,targetDist,targetAngle)
# Added inputs, targetDist & targetAngle to function: avoidStationaryObstacles(speed,targetDist,targetAngle)

import pixyRacer
import pixy
import time
import math
from helper import PID_controller, blocksAreNew, copyBlockArray


class obstacleAvoidance(object):
	def __init__(self, racerBot):
		self.bot = racerBot

		self.constantServoPosition = 0

		# variables for pixy
		self.pixyMinX = 0.0
		self.pixyMaxX = 316.0
		self.pixyX_FoV = 50.0
		self.pixyY_FoV = 50.0
		self.pixyMinY = 0.0
		self.pixyMaxY = 208.0
		self.pixyCenterX = (self.pixyMaxX - self.pixyMinX)/2
		self.pixyCenterY = (self.pixyMaxY - self.pixyMinY)/2

		self.oldBlocks = pixy.BlockArray(100)
		self.oldCount = self.bot.pixy.ccc_get_blocks(100, self.oldBlocks)
		self.newBlocks = pixy.BlockArray(100)
		self.newCount = self.bot.pixy.ccc_get_blocks(100, self.newBlocks)

		self.centerLineID	= 1
		self.leftLineID 	= 2
		self.rightLineID 	= 3
		self.obstacleID 	= 4

		# obstacle variables
		self.nObservations = 10
		self.frameTimes = [float('nan') for i in range(self.nObservations)]
		self.obstacleSize = [float('nan') for i in range(self.nObservations)]
		self.obstacleAngle = [float('nan') for i in range(self.nObservations)]

		self.gimbal = PID_controller(0.06, 0.0, 0.06)

	def updateBlocks(self):
		copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
		self.oldCount = self.newCount
		self.newCount = self.bot.pixy.ccc_get_blocks (100, self.newBlocks)

	def detectObstacle(self): # find largest block with the obstacle ID, if it exists, and get its parameters
		self.biggestObstacle = -1
		for i in range(self.newCount):
			if self.newBlocks[i].m_signature == self.obstacleID:
				self.biggestObstacle = i
				break

		if self.biggestObstacle >= 0:                       
			pixelSize = self.newBlocks[self.biggestObstacle].m_width; 
			angleSize = pixelSize/self.pixyMaxX*self.pixyX_FoV #???
			self.obstacleSize.append(angleSize)
			self.frameTimes.append(time.time())
			self.dObstacleSize = [(self.obstacleSize[i+1] - self.obstacleSize[i])/(self.frameTimes[i+1] - self.frameTimes[i]) for i in range(len(self.obstacleSize)-1)]
			self.obstacleSize.pop(0)

			pixelDistance =self.newBlocks[self.biggestObstacle].m_x -  self.pixyCenterX
			angleDistance = pixelDistance/self.pixyMaxX*self.pixyX_FoV
			self.obstacleAngle.append(angleDistance)
			self.dObstacleAngle = [(self.obstacleAngle[i+1] - self.obstacleAngle[i])/(self.frameTimes[i+1] - self.frameTimes[i]) for i in range(len(self.obstacleAngle)-1)]
			self.obstacleAngle.pop(0)
			self.frameTimes.pop(0)
			return True
		else:
			return False

	def drive(self, drive, bias): # Differential drive function
		maxDrive = 1 # set safety limit for the motors
		
		totalDrive = drive * maxDrive # the throttle of the car
		diffDrive = bias * totalDrive # set how much throttle goes to steering
		straightDrive = totalDrive - abs(diffDrive) # the rest for driving forward (or backward)
		
		lDrive = straightDrive + diffDrive
		rDrive = -(straightDrive - diffDrive)
		self.bot.setTurnRates(lDrive, rDrive)
		
	def visTrack(self, objectID): # Get pixycam to rotate to track an object
			if (self.newCount-1) < objectID or objectID < 0: # do nothing when we block doesn't exist
				self.bot.setServoPosition(0)
				return -1   			
			else:
				visError =  self.newBlocks[objectID].m_x - self.pixyCenterX # error in pixels
				visAngularError = -(visError/self.pixyMaxX*self.pixyX_FoV) # error converted to angle
				visTargetAngle = self.bot.servoPosition + self.gimbal.update(visError) # error relative to pixycam angle
				self.bot.setServoPosition(visTargetAngle)

	def getDistance(self, objectID):
		if (self.newCount-1) < objectID or objectID < 0: # do nothing when we block doesn't exist
			return -1

		#camHeight = 0.095 # this is measured for the robot kit
		camHeight = 0.095
		camAngle = 12.0/180.0*math.pi # this is the camera angle in radian

		pixelDistance = self.pixyCenterY - (self.newBlocks[objectID].m_y + self.newBlocks[objectID].m_height/2.0)

		angle = pixelDistance/self.pixyMaxY*self.pixyY_FoV/180.0*math.pi

		distance = camHeight * math.tan(math.pi/2.0 - camAngle + angle)

		return distance

	# New Addition
	def steerTowards(self, objectIDIndex, angle):
		percentageError = 0
		if not objectIDIndex:
			return 0
		else:
			error = [2*(self.newBlocks[i].m_x/self.pixyMaxX - 0.5) for i in objectIDIndex]
			if len(error) == 1:
				percentageError = error[0]
			elif len(error) == 2:
				percentageError = 0.8*error[0] + 0.2*error[1]
			elif len(error) == 3:
				percentageError = 0.65*error[0] + 0.25*error[1] + 0.1*error[2]
			elif len(error) == 4:
				percentageError = 0.5*error[0] + 0.3*error[1] + 0.15*error[2] + 0.05*error[-1]
			steering = -angle*percentageError
			return steering
	# New Addition
	def steerAway(self, objectIDIndex, angle):
		if not objectIDIndex:
			return 0
		else:
			error = [2*(self.newBlocks[i].m_x/self.pixyMaxX - 0.5) for i in objectIDIndex]
			if len(error) == 1:
				percentageError = error[0]
			elif len(error) == 2:
				percentageError = 0.8*error[0] + 0.2*error[1]
			elif len(error) == 3:
				percentageError = 0.65*error[0] + 0.25*error[1] + 0.1*error[2]
			elif len(error) == 4:
				percentageError = 0.5*error[0] + 0.3*error[1] + 0.15*error[2] + 0.05*error[-1]
			steering = angle*percentageError
			return steering

	def laneFollowing(self, speed):		
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		previousSpeed = 0
		targetSpeed = 0
		
		while True:
			self.updateBlocks()
			smallestCenterLine = -1
			smallestObstacle = -1
			bias = 0
			previousSpeed = targetSpeed
			targetSpeed = speed

			centerIndex = []
			leftIndex = []
			rightIndex = []

			for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
				if self.newBlocks[i].m_signature == self.centerLineID:
					centerIndex.append(i)
				elif self.newBlocks[i].m_signature == self.leftLineID:
					leftIndex.append(i)
				elif self.newBlocks[i].m_signature == self.rightLineID:
					rightIndex.append(i)
				elif self.newBlocks[i].m_signature == self.obstacleID:
					smallestObstacle = i

			# centerIndex = [i for i in range(0,len(self.newCount)) if self.newBlocks[i] == self.centerLineID]

			if not centerIndex: # stop the racer and wait for new blocks
				#if previousSpeed > 0:
				#	targetSpeed = previousSpeed - 0.05
				lbias = self.steerAway(leftIndex,0.4)
				rbias = self.steerAway(rightIndex,0.4)
				bias = lbias + rbias
				#print([self.newBlocks[i].m_signature for i in range(0,self.newCount)])
				#if self.bot.servoPosition <= 0:
				#	self.bot.setServoPosition(self.bot.servoPosition + 15)
				#elif self.bot.servoPosition > 5:
				#	self.bot.setServoPosition(self.bot.servoPosition - 30)

			else: # drive while we see a center line
				self.bot.setServoPosition(0)
			###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error. 
			### Come up with a steering command to send to self.drive(speed, steering) function
				bias = self.steerTowards(centerIndex,0.4)
				#self.visTrack(centerIndex[0])	
				
			self.drive(targetSpeed,bias)					
				
	def stopAtStationaryObstacles(self, speed, targetDist, targetAngle):	
		# make sure the speed never goes to unreasonable values
		if speed < 0:
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		
		while True:
			self.updateBlocks()

			targetSpeed = speed
			bias = 0

			centerIndex = []
			leftIndex = []
			rightIndex = []			
			
			for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
				if self.newBlocks[i].m_signature == self.centerLineID:
					centerIndex.append(i)
				elif self.newBlocks[i].m_signature == self.leftLineID:
					leftIndex.append(i)
				elif self.newBlocks[i].m_signature == self.rightLineID:
					rightIndex.append(i)
			
			if not centerIndex: # stop the racer and wait for new blocks
				self.bot.setMotorSpeeds(0, 0)

			else: # drive while we see a center line
				bias = self.steerTowards(centerIndex,0.4)				
				if self.detectObstacle():
					dist = self.getDistance(self.biggestObstacle)

					if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.obstacleAngle[0]) < targetAngle):
						targetSpeed = 0
				else: 
					#self.visTrack(centerIndex[0])
					self.bot.setServoPosition(0)
				self.drive(targetSpeed,bias)

	def avoidStationaryObstacles(self, speed, targetDist, targetAngle):		
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0)

		while True:
			# self.updateBlocks()
			# self.detectObstacle()
			
			# centerLine = []
			# leftLine = []
			# rightLine = []
			# obstacle = []

			# centerLineBlock = -1
			# leftLineBlock = -1
			# rightLineBlock = -1
			# obstacleBlock = -1

			# trackingTarget = -1
						
			# lineSteering = 0
			# obstacleSteering = 0
			
			# for i in range(0, self.newCount):
			# 	centerLine.append(self.newBlocks[i].m_signature == self.centerLineID)
			# 	if centerLine[i]:
			# 		centerLineBlock = i


			self.updateBlocks()

			targetSpeed = speed
			bias = 0

			centerIndex = []
			leftIndex = []
			rightIndex = []		

			for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
				if self.newBlocks[i].m_signature == self.centerLineID:
					centerIndex.append(i)
				elif self.newBlocks[i].m_signature == self.leftLineID:
					leftIndex.append(i)
				elif self.newBlocks[i].m_signature == self.rightLineID:
					rightIndex.append(i)
			
			if not centerIndex: # stop the racer and wait for new blocks
				targetSpeed = 0

			else: # drive while we see a center line
				bias = self.steerTowards(centerIndex,0.4)            	
				if self.detectObstacle():
					#targetSpeed = 0
					###Level 5### Please insert code here to derive non-zero obstacleSteering and keep the robot running
					### You need to first identify the obstacle ID and decide whether you want to track it. Then you use what you learn from Level 2,3,4 to implement detection and steering.
					self.visTrack(centerIndex[0]) # Lvl 5
					dist = self.getDistance(self.biggestObstacle)
					if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.bot.servoPosition) < targetAngle):
						bias = steerAway(self.biggestObstacle,0.6) # Lvl 5

				else:             		
					self.bot.setServoPosition(0)

			self.drive(targetSpeed,bias)		

			#steering = obstacleSteering + lineSteering # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
			#self.drive(targetSpeed, steering) 

	def avoidMovingObstacles(self, speed):
		###Level 6 Bonus### If you feel comfortable, try to implement a behaviour to avoid a moving obstacle
		return