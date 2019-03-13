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
#import numpy
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

		# 1:Red; 2:Blue; 3: Green; 4: Yellow; 5: Magenta
		self.centerLineID	= 1
		self.leftLineID 	= 2
		self.rightLineID 	= 3
		self.obstacleID 	= 4
		self.shortcutID		= 5

		# obstacle variables
		self.nObservations = 10
		self.frameTimes = [float('nan') for i in range(self.nObservations)]
		self.obstacleSize = [float('nan') for i in range(self.nObservations)]
		self.obstacleAngle = [float('nan') for i in range(self.nObservations)]

		self.shortcutFirstTime = 0
		self.gimbal = PID_controller(0.06, 0.0, 0.0)

		# Left/Right Offset average
		self.leftOffset = -145.5
		self.rightOffset = 145.5

		#
		self.sign = lambda x: (1, -1)[x < 0]


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

	def detectShortcut(self):
		self.shortcutBlock = -1
		for i in range(self.newCount):
			if self.newBlocks[i].m_signature == self.shortcutID:
				self.shortcutBlock = i
				break

		if self.shortcutBlock >= 0:                       
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
			if abs(visTargetAngle) > 90:
				visTargetAngle = self.sign(visTargetAngle)*90
			self.bot.setServoPosition(visTargetAngle)

	def visTrackBetween(self, objectIDs):
		if not objectIDs:
			return 0 
		else:
			visError =  sum([self.newBlocks[x].m_x for x in objectIDs])/len(objectIDs) - self.pixyCenterX
			visAngularError = -(visError/self.pixyMaxX*self.pixyX_FoV) # error converted to angle
			visTargetAngle = self.bot.servoPosition + self.gimbal.update(visError) # error relative to pixycam angle
			if abs(visTargetAngle) > 90:
				visTargetAngle = self.sign(visTargetAngle)*90
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
			elif len(error) >= 4:
				percentageError = 0.5*error[0] + 0.3*error[1] + 0.15*error[2] + 0.05*error[-1]
			steering = -angle*percentageError
			return steering
	# New Addition
	def steerAway(self, objectIDIndex, angle):
		bias = -self.steerTowards(objectIDIndex,angle)
		return bias
		
	def steerBeside(self, objectIDIndex, angle, offset):
		if not objectIDIndex:
			return 0
		else:
			error = [2*((self.newBlocks[i].m_x - offset)/self.pixyMaxX - 0.5) for i in objectIDIndex]
			if len(error) == 1:
				percentageError = error[0]
			elif len(error) == 2:
				percentageError = 0.8*error[0] + 0.2*error[1]
			elif len(error) == 3:
				percentageError = 0.65*error[0] + 0.25*error[1] + 0.1*error[2]
			elif lineen(error) >= 4:
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
			shortcut = -1
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
				elif self.newBlocks[i].m_signature == self.shortcutID:
					shortcut = i

			# centerIndex = [i for i in range(0,len(self.newCount)) if self.newBlocks[i] == self.centerLineID]
			#centerIndex = []

			if (time.time() - self.shortcutFirstTime < 0.2) and not (self.shortcutFirstTime == 0):
				#bias = self.steerTowards([shortcut],0.4)
				bias = -0.4 + self.steerTowards(leftIndex[0:1],0.5)
				print(['Shorcut: Held', bias])

			elif (shortcut > 0):
				#print('Shortcut',self.shortcutFirstTime)
				if (time.time() - self.shortcutFirstTime > 0.6):
					self.shortcutFirstTime = time.time()
					bias = -0.5
					print('Shortcut: First Time')

			elif (time.time() - self.frameTimes[-1] < 0.4):
				#targetSpeed = 0
				###Level 5### Please insert code here to derive non-zero obstacleSteering and keep the robot running
				### You need to first identify the obstacle ID and decide whether you want to track it. Then you use what you learn from Level 2,3,4 to implement detection and steering.
				 # Lvl 5
				bias = self.steerTowards(centerIndex,0.4) 
				dist = self.getDistance(self.biggestObstacle)
				if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.bot.servoPosition) < targetAngle):
					self.visTrack(self.biggestObstacle)
					bias = 0.5*self.bot.servoPosition/90 # Lvl 5
					servoList.append(self.bot.servoPosition)
					#print(biasList)
				elif (time.time() - self.frameTimes[-1] > 0.05) and servoList and not (self.bot.servoPosition == 0):	
					#bias = -self.bot.servoPosition/90
					#print(biasList)
					bias = -0.7*servoList[-1]/90
					self.bot.setServoPosition(servoList[-1])
					servoList.pop(-1)
				#print(len(biasList))

			elif not centerIndex: # stop the racer and wait for new blocks
				#if previousSpeed > 0:
				#	targetSpeed = previousSpeed - 0.05
				lbias = self.steerAway(leftIndex,0.4)
				rbias = self.steerAway(rightIndex,0.4)
				bias = lbias + rbias
				targetSpeed = 0.6*targetSpeed

				#print([self.newBlocks[i].m_signature for i in range(0,self.newCount)])
				#if self.bot.servoPosition <= 0:
				#	self.bot.setServoPosition(self.bot.servoPosition + 15)
				#elif self.bot.servoPosition > 5:
				#	self.bot.setServoPosition(self.bot.servoPosition - 30)

			elif centerIndex: # drive while we see a center line
				#self.visTrack(centerIndex[0])
				self.bot.setServoPosition(0)
			###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error. 
			### Come up with a steering command to send to self.drive(speed, steering) function
				bias = self.steerTowards(centerIndex,0.4)
				if leftIndex:
					self.leftOffset = math.floor(self.leftOffset + (self.newBlocks[leftIndex[0]].m_x - self.pixyMaxX/2))/2
				if rightIndex:
					self.rightOffset = math.floor(self.rightOffset + (self.newBlocks[rightIndex[0]].m_x - self.pixyMaxX/2))/2
				#print([self.leftOffset, self.rightOffset])
				#bias = self.bot.servoPosition/90
				#self.visTrack(centerIndex[0])	
				
			self.drive(targetSpeed,bias)					

	def altLaneFollowing(self, speed):		
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

			if not leftIndex: # stop the racer and wait for new blocks
				#if previousSpeed > 0:
				#	targetSpeed = previousSpeed - 0.05
				#bias = self.steerTowards(centerLine,0.4)
				targetSpeed = 0.1*targetSpeed
				#print([self.newBlocks[i].m_signature for i in range(0,self.newCount)])
				#if self.bot.servoPosition <= 0:
				#	self.bot.setServoPosition(self.bot.servoPosition + 15)
				#elif self.bot.servoPosition > 5:
				#	self.bot.setServoPosition(self.bot.servoPosition - 30)

			else: # drive while we see a center line
				self.bot.setServoPosition(0)
			###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error. 
			### Come up with a steering command to send to self.drive(speed, steering) function
				bias = self.steerBeside([leftIndex[0]],0.4,self.leftOffset)
				#self.visTrack(centerIndex[0])	
				
			self.drive(targetSpeed,bias)			

	def followVisTrack(self,speed):
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		previousSpeed = 0
		targetSpeed = 0
		sign = 1

		while True:
			self.updateBlocks()
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

			if leftIndex and self.detectObstacle():
				self.visTrackBetween(centerIndex[0],self.biggestObstacle)
				bias = -self.bot.servoPosition/90
				#print(bias)
			else:
				targetSpeed = 0.5*speed

				servoPos = self.bot.servoPosition
				if abs(servoPos) > 50:
					sign = -sign
					servoPos = servoPos + 2*5*sign
				else:
					servoPos = servoPos + 5*sign
				self.bot.setServoPosition(servoPos)
				self.drive(targetSpeed,bias)
			print([targetSpeed,bias,self.bot.servoPosition])
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
				self.visTrack(centerIndex[0]);
				bias = -self.bot.servoPosition/90
				#bias = self.steerTowards(centerIndex,0.4)				
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
		biasList = []
		servoList = []
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
			self.detectObstacle()

			targetSpeed = speed
			bias = 0
			prevBias = 0

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

			

			#print(time.time() - self.frameTimes[-1])
			

			if (time.time() - self.frameTimes[-1] < 0.4):
				#targetSpeed = 0
				###Level 5### Please insert code here to derive non-zero obstacleSteering and keep the robot running
				### You need to first identify the obstacle ID and decide whether you want to track it. Then you use what you learn from Level 2,3,4 to implement detection and steering.
				 # Lvl 5
				bias = self.steerTowards(centerIndex,0.4) 
				dist = self.getDistance(self.biggestObstacle)
				if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.bot.servoPosition) < targetAngle):
					self.visTrack(self.biggestObstacle)
					bias = 0.5*self.bot.servoPosition/90 # Lvl 5
					servoList.append(self.bot.servoPosition)
					#print(biasList)
				elif (time.time() - self.frameTimes[-1] > 0.05) and servoList and not (self.bot.servoPosition == 0):	
					#bias = -self.bot.servoPosition/90
					#print(biasList)
					bias = -0.7*servoList[-1]/90
					self.bot.setServoPosition(servoList[-1])
					servoList.pop(-1)
				print(len(biasList))
					#if self.bot.servoPosition > 0:
					#	self.bot.setServoPosition(self.bot.servoPosition - 1)
					#else:
					#	self.bot.setServoPosition(self.bot.servoPosition + 1)
				#elif (time.time() - self.frameTimes[-1] > 0.1):
				#	self.bot.setServoPosition(0)
				#	servoList = []
			#elif (time.time() - self.frameTimes[-1] > 2):             		
			#	self.bot.servoPosition
				#else:
				#	

			elif not centerIndex: # stop the racer and wait for new blocks
				targetSpeed = 0

			else: # drive while we see a center line
				bias = self.steerTowards(centerIndex,0.4)  
				self.bot.setServoPosition(0)
				servoList = []
				#biasList = []          	
				
			prevBias = bias
			self.drive(targetSpeed,bias)		

			#steering = obstacleSteering + lineSteering # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
			#self.drive(targetSpeed, steering) 

	def avoidMovingObstacles(self, speed):
		###Level 6 Bonus### If you feel comfortable, try to implement a behaviour to avoid a moving obstacle
		return

	def idle(self,speed):
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		previousSpeed = 0
		targetSpeed = 0
		sign = 1
		
		while True:
			bias = 0

			targetSpeed = 0.2*speed
			servoPos = self.bot.servoPosition
			if abs(servoPos) > 50:
				sign = -sign
				servoPos = servoPos + 2*5*sign
			else:
				servoPos = servoPos + 5*sign
			self.bot.setServoPosition(servoPos)
			self.drive(targetSpeed,bias)

	def newRace(self,speed):
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		previousSpeed = 0
		targetSpeed = 0
		sign = 1
		
		while True:
			self.updateBlocks()
			shortcut = -1
			bias = 0
			targetSpeed = speed

			servoPos = self.bot.servoPosition			

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
				elif self.newBlocks[i].m_signature == self.shortcutID:
					shortcut = i

			# centerIndex = [i for i in range(0,len(self.newCount)) if self.newBlocks[i] == self.centerLineID]
			#centerIndex = []

			if (time.time() - self.shortcutFirstTime < 0.25) and not (self.shortcutFirstTime == float('nan')):
					bias = -0.2 + self.steerTowards(leftIndex[0:1],0.6)
					self.bot.setServoPosition(0)
					print(['Shorcut: Held', bias])

			elif shortcut > 0:
				self.shortcutFirstTime = time.time()
				bias = -0.6
				print('Shortcut: First Time')

			elif (time.time() - self.frameTimes[-1] < 0.4):
				print("Obstacle Routine: Detected")
				bias = self.steerTowards(centerIndex,0.4)
				dist = self.getDistance(self.biggestObstacle)
				if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.bot.servoPosition) < targetAngle):
					self.visTrack(self.biggestObstacle)
					bias = 0.5*self.bot.servoPosition/90 # Lvl 5
					servoList.append(self.bot.servoPosition)
					#print(biasList)
				elif (time.time() - self.frameTimes[-1] > 0.05) and servoList and not (self.bot.servoPosition == 0):	
					#bias = -self.bot.servoPosition/90
					#print(biasList)
					bias = -0.7*servoList[-1]/90
					self.bot.setServoPosition(servoList[-1])
					servoList.pop(-1)
				#print(len(biasList))

			elif centerIndex:
				print("CenterIndex: Detected")
				self.visTrackBetween(centerIndex)
				bias = -self.bot.servoPosition/90
				if leftIndex:
					self.leftOffset = math.floor(self.leftOffset + (self.newBlocks[leftIndex[0]].m_x - self.pixyMaxX/2))/2
				if rightIndex:
					self.rightOffset = math.floor(self.rightOffset + (self.newBlocks[rightIndex[0]].m_x - self.pixyMaxX/2))/2

			elif leftIndex or rightIndex:
				print("Left/Right Index: Detected")

				targetSpeed = 0.5*targetSpeed
				lbias = self.steerAway(leftIndex,0.4)
				rbias = self.steerAway(rightIndex,0.4)
				bias = -servoPos/90

				if abs(servoPos) > 10:
					servoPos = servoPos - (self.sign(servoPos)*10)
				else:
					servoPos = 0
				self.bot.setServoPosition(servoPos)

			else:
				print("Idle: Nothing")
				targetSpeed = 0.5*speed
				if abs(servoPos) > 90:
					sign = -self.sign(servoPos)
					servoPos = servoPos + 2*5*sign
				else:
					servoPos = servoPos + 5*sign
				self.bot.setServoPosition(servoPos)
				#print(sign)
				
			self.drive(targetSpeed,bias)

