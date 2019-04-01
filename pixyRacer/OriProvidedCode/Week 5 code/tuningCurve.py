import pixyRacer
import pixy
import time
import csv
import os.path
from helper import PID_controller, blocksAreNew, copyBlockArray

class tuningCurve(object):
	def __init__(self, racerBot, controller):
		# pixy parameters
		self.pixyMinX = 0
		self.pixyMaxX = 316
		self.pixyMinY = 0
		self.pixyMaxY = 208
		self.pixyCenterX = (self.pixyMaxX - self.pixyMinX)/2
		self.pixyCenterY = (self.pixyMaxY - self.pixyMinY)/2
		self.targetSignature = 1
		self.calibrationSignature = 2
		# servo parameters
		self.servoMinPos = -90
		self.servoMaxPos = 90
		self.servoPos = 0
		# variables for pixy
		self.oldBlocks = pixy.BlockArray(100)
		self.newBlocks = pixy.BlockArray(100)
		self.oldCount = pixy.ccc_get_blocks(100, self.oldBlocks)
		self.newCount = pixy.ccc_get_blocks (100, self.newBlocks)
		# controller needs to have an update function must have an 
		# update function that takes in error (in X) and returns 
		# change in angle to send to the servo
		self.gimbal = controller
		self.calibrationController = PID_controller(0.06, 0, 0.06)
		# pixyRacer to control
		self.racerBot = racerBot
		# tuning curve parameters
		self.minAngle = 0
		self.maxAngle = 0
		self.targetMinAngle = -35
		self.targetMaxAngle = 35
		
	def calibrate(self):
		self.minAngle = 0
		self.maxAngle = 0
		# wait for a blob with signature 2 (Red) to appear
		while not ((self.newCount > 0) 
			   and (self.newBlocks[0].m_signature == self.calibrationSignature)
			   and (blocksAreNew(self.oldCount, self.oldBlocks, self.newCount, self.newBlocks))):
			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
		# track the min and max position of the servo, making sure the 
		# error is not too big
		while self.newBlocks[0].m_signature == self.calibrationSignature:
			# wait for new blocks
			while not blocksAreNew(self.oldCount, self.oldBlocks, \
								   self.newCount, self.newBlocks):	
				copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
				self.oldCount = self.newCount
				self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
			# track the calibration target
			error =  self.newBlocks[0].m_x - self.pixyCenterX
			self.servoPos = self.servoPos + self.calibrationController.update(error)
			if self.servoPos > self.servoMaxPos:
				self.servoPos = self.servoMaxPos
			elif self.servoPos < self.servoMinPos:
				self.servoPos = self.servoMinPos
			self.racerBot.setServoPosition(self.servoPos)
			if error < 20:
				if self.servoPos < self.minAngle:
					self.minAngle = self.servoPos
				elif self.servoPos > self.maxAngle:
					self.maxAngle = self.servoPos
			# get new frame
			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
	
	def measure(self, f_run):
		# track 10 periods of the stimulus
		t_run = 10.0/f_run
		calibrations = 0
		fileNumber = 0
		times = []
		errors = []
		angles = []
		while ((self.minAngle > self.targetMinAngle)
		   and (self.maxAngle < self.targetMaxAngle)):
			if calibrations > 0:
				print "The robot's camera should be 10 cm from the \
					   screen in the middle of the calibration track"
				print self.minAngle, self.maxAngle
			self.calibrate()
			calibrations = calibrations + 1
		
		t_start = time.time()
		t_end = t_start + t_run
		
		while time.time() < t_end:
			# wait for new blocks
			while not blocksAreNew(self.oldCount, self.oldBlocks, \
								   self.newCount, self.newBlocks):	
				copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
				self.oldCount = self.newCount
				self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
			# track the target
			error = self.newBlocks[0].m_x - self.pixyCenterX 
			self.servoPos = self.servoPos + self.gimbal.update(error)
			if self.servoPos > self.servoMaxPos:
				self.servoPos = self.servoMaxPos
			elif self.servoPos < self.servoMinPos:
				self.servoPos = self.servoMinPos
			self.racerBot.setServoPosition(self.servoPos)
			
			times.append(time.time() - t_start)
			errors.append(error)
			angles.append(self.servoPos)
			
			# get new frame
			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
		
		fileName = "calibrationCurve" + str(fileNumber) + ".csv"
		while os.path.exists(fileName):
			fileNumber = fileNumber + 1
			fileName = "calibrationCurve" + str(fileNumber) + ".csv"
		with open(fileName, "w+") as myCsv:
			csvWriter = csv.writer(myCsv, delimiter=',')
			csvWriter.writerows([times, errors, angles])
		
	def visualPredication(self, predicationFunction):

		times = []
		errors = []
		angles = []
		
		t_start = time.time()

		self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)

		# wait for a red blob to show up
		while not ((self.count > 0) and (self.newBlocks[0].m_signature == 1)):
			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)

		# while we do see a red blob, track it
		while ((self.count > 0) and (self.newBlocks[0].m_signature == 1)):
			error = self.newBlocks[0].m_x - self.pixyCenterX 
			self.servoPos = self.servoPos + self.gimbal.update(error)
			if self.servoPos > self.servoMaxPos:
				self.servoPos = self.servoMaxPos
			elif self.servoPos < self.servoMinPos:
				self.servoPos = self.servoMinPos
			self.racerBot.setServoPosition(self.servoPos)

			times.append(time.time() - t_start)
			errors.append(error)
			angles.append(self.servoPos)

			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)

		# once the blob disappears, invoke the function supplied by the students
		while not ((self.count > 0) and (self.newBlocks[0].m_signature == 1)):
			time_now = time.time() - t_start
			try:
				pos = predicationFunction(times, time_now, errors, angles)
			except:
				return None
			# move to the position demanded by the students
			self.racerBot.setServoPosition(pos)

			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)

		# at this point the blob should have reappeared and we are just checking how good the tracking was
		# check if the blob is already in view
		if self.newBlocks[0].m_signature == targetSignature:
			error =  self.newBlocks[0].m_x - self.pixyCenterX
			return error
		else:
			return float('Inf')

		time_now = time.time()
		max_time =  10
		while (time.time()-time_now < max_time):
			copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
			self.oldCount = self.newCount
			self.newCount = self.racerBot.pixy.ccc_get_blocks (100, self.newBlocks)
			if self.newBlocks[0].m_signature == targetSignature:
				error =  self.newBlocks[0].m_x - self.pixyCenterX
				return 'late', error

		return 'noShow', None
		