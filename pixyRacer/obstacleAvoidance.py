### ALBiR 2018-19 Spring
### Made by: HTL, LH, DK
### Last updated: 02/19 by DK

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

		self.gimbal = PID_controller(0.06, 0.0, 0.0)

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

	def drive(self, drive, bias): # Differential drive function
		maxDrive = 0.5 # set safety limit for the motors
		
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
		camAngle = 15.0/180.0*math.pi # this is the camera angle in radian

		pixelDistance = self.pixyCenterY - (self.newBlocks[objectID].m_y + self.newBlocks[objectID].m_height/2.0)

		angle = pixelDistance/self.pixyMaxY*self.pixyY_FoV/180.0*math.pi

		distance = camHeight * math.tan(math.pi/2.0 - camAngle + angle)

		return distance

	def laneFollowing(self, speed , targetDist):
		if speed < 0: # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		
		while True:
			self.updateBlocks()
			smallestCenterLine = -1
			smallestRightLine = -1
			smallestLeftLine = -1
			obstacle = -1
			centerIndex = []
			for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
				if self.newBlocks[i].m_signature == self.centerLineID:
					if len(centerIndex) < 3:
						centerIndex.append(i)
					smallestCenterLine = i

				elif self.newBlocks[i].m_signature == self.leftLineID:
					smallestLeftLine = i
				elif self.newBlocks[i].m_signature == self.rightLineID:
					smallestRightLine = i
				elif self.newBlocks[i].m_signature == self.obstacleID:
					smallestObstacle = i
					#print(smallestObstacle)
			
			if not smallestCenterLine >= 0: # stop the racer and wait for new blocks
				self.bot.setMotorSpeeds(0, 0)

			else: # drive while we see a center line
				#self.drive(self,0.5,0)
				
				error1 = 2*(self.newBlocks[centerIndex[0]].m_x - self.pixyCenterX)/self.pixyMaxX
				error2 = 2*(self.newBlocks[smallestCenterLine].m_x - self.pixyCenterX)/self.pixyMaxX
				percentageError = 0.8*(error1) + 0.2*(error2)
				bias = -0.4*percentageError
				#print([error1,error2,percentageError])
				self.drive(speed, bias)

				# dist = self.getDistance(smallestObstacle)
				# print("Linear Distance:", dist)
				#
				# pixelDistance = self.newBlocks[smallestObstacle].m_x - self.pixyCenterX
				# angleDistance = pixelDistance / self.pixyMaxX * self.pixyX_FoV
				# print("Angle Distance:", angleDistance)
				#
				# if ((dist - targetDist < 0.02) and (dist - targetDist > -1)):
				# 	# self.bot.setMotorSpeeds(0, 0)
				# 	if smallestLeftLine == -1:
				# 		biasSign = 1
				# 		print("Left line is far away")
				# 	elif smallestRightLine == -1:
				# 		biasSign = -1
				# 		print("Right line is far away")
				# 	elif smallestLeftLine and smallestRightLine > 0:
				# 		print("Both bounds are visible")
				# 		cleftLineArea = self.newBlocks[smallestLeftLine].m_width * self.newBlocks[smallestLeftLine].m_height
				# 		crightLineArea = self.newBlocks[smallestRightLine].m_width * self.newBlocks[smallestRightLine].m_height
				#
				# 		# Should be such that it sets direction of avoidance based on which bound line is closer
				# 		# Move further away from the closest one
				# 		if cleftLineArea > crightLineArea:
				# 			print("Left Line is closer")
				# 			# Check that you have biased it the correct way - it should be moving to the right
				# 			biasSign = -1
				# 		elif crightLineArea > cleftLineArea:
				# 			print("Right line is closer")
				# 			# Should go to the left
				# 			biasSign = 1
				# 		else:
				# 			print("Error: Cannot detect boundary lines.")
				#
				# 	# Simple obstacle avoidance
				# 	angle = 0
				# 	speed = 0.1
				# 	while angle < math.pi/4:
				# 		# Radius it actually turns is dependent on the steps between angle changes
				# 		# (Rate of change of angular displace
				# 		ment but I cannot test this rn)
				# 		# TODO: Check that you have biased it the right way
				# 		bias = math.cos(angle) * biasSign
				# 		self.drive(speed, bias)
				# 		# self.drive(1, 0)
				# 		# print("Angle:", angle)
				# 		angle += math.pi / 50

			###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error. 
            ### Come up with a steering command to send to self.drive(speed, steering) function
				self.updateBlocks()

	def stopAtStationaryObstacles(self, speed):	
		# make sure the speed never goes to unreasonable values
		if speed < 0:
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0) # set servo to centre
		self.bot.setMotorSpeeds(0, 0) # set racer to stop
		
		while True:
			self.updateBlocks()
			smallestCenterLine = -1
			
			for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
				if self.newBlocks[i].m_signature == self.centerLineID:
					smallestCenterLine = i

            ###Level 2### Set up an if statement to set targetSpeed = 0 when you detect and obstacle.
            ### Understand how center line detection code works. Can you implement the same thing for the obstacle?\ 
                      
            ###Level 3### Modify your if statement to set targetSpeed = 0 when you detect a obstacle at a specific distance.
            ### Here you want to use the getDistance(obstacleBlock) and try to compute the obstacle distacne 
                      
            ###Level 4### Modify your if statement to set targetSpeed = 0 when you detect a obstacle at within a distance and a frontal angular space.  
            ### Here you want to incorporate the servo pan angle to reconstruct the obstacle orienation relative to the robot            

	def avoidStationaryObstacles(self, speed):		
		if speed < 0:	 # make sure the speed never goes to unreasonable values
			speed = 0
		elif speed > 1:
			speed = 1

		self.bot.setServoPosition(0)

		while True:
			self.updateBlocks()
			self.detectObstacle()
			
			centerLine = []
			leftLine = []
			rightLine = []
			obstacle = []

			centerLineBlock = -1
			leftLineBlock = -1
			rightLineBlock = -1
			obstacleBlock = -1

			trackingTarget = -1
                        
			lineSteering = 0
			obstacleSteering = 0
			
			for i in range(0, self.newCount):
				centerLine.append(self.newBlocks[i].m_signature == self.centerLineID)
				if centerLine[i]:
					centerLineBlock = i
                    
		###Level 5### Please insert code here to derive non-zero obstacleSteering and keep the robot running
		### You need to first identify the obstacle ID and decide whether you want to track it. Then you use what you learn from Level 2,3,4 to implement detection and steering.

			steering = obstacleSteering + lineSteering	 # we set the final steering as a linear combination of the obstacle steering and center line steering - but it's all up to you!
			self.drive(targetSpeed, steering) 

	def avoidMovingObstacles(self, speed):
		###Level 6 Bonus### If you feel comfortable, try to implement a behaviour to avoid a moving obstacle
		return

	def dodge(self, direction):
		st = [0.2, -0.21]
		turn90 = [0.2, 0.2]
		stop = [0, 0]

		st_t = 0.7
		turn_t = 0.5
		stop_t = 0.1

		durations = [turn_t, stop_t, st_t, stop_t,
					 turn_t, stop_t, st_t, stop_t,
					 turn_t, stop_t, st_t, stop_t,
					 turn_t, stop_t, st_t, stop_t, ]
		speeds = [turn90 * direction, stop, st, stop,
				  turn90 * direction * -1, stop, st, stop,
				  turn90 * direction * -1, stop, st, stop,
				  turn90 * direction, stop, st, stop]

		self.tp.deadReckoningTrack(durations, speeds)