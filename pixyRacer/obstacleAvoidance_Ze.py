### ALBiR 2018-19 Spring
### Made by: HTL, LH, DK
### Modified & Updated by HotWheels Team: Ze, Ife, Aymeric
### Last updated: 1/4/19 by Ze

# Modified functions: drive(~), detectObstacle(~), laneFollowing(~), stopAtStationaryObstacles(~), avoidStationaryObstacles(~)
# Added inputs, targetDist & targetAngle to function: stopAtStationaryObstacles(speed,targetDist,targetAngle)
# Added inputs, targetDist & targetAngle to function: avoidStationaryObstacles(speed,targetDist,targetAngle)
# Added function: steerTowards(objectID)
# Added function: steerAway(objectID)
# Added function: visTrackBetween(objectIDs)
# Added function: steerBeside(objectIDIndex, angle, offset)
# Added function: altLaneFollowing(speed)
# Added function: idle(speed)
# Added function: getAngle(objectIDs)
# Added function: followVisTrack(speed)
# Added function: newRace(speed)

# Added function: dodge(direction)

# Deleted Unused Commented Sections
# Changed laneFollowing to finalRace and rewrote laneFollowing
# Modified function: laneFollowing(~)
# Added function: finalRace(speed)

import pixyRacer
import pixy
import time
import math
from helper import PID_controller, blocksAreNew, copyBlockArray
from turningPad import turningPad


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
        self.pixyCenterX = (self.pixyMaxX - self.pixyMinX) / 2
        self.pixyCenterY = (self.pixyMaxY - self.pixyMinY) / 2

        self.oldBlocks = pixy.BlockArray(100)
        self.oldCount = self.bot.pixy.ccc_get_blocks(100, self.oldBlocks)
        self.newBlocks = pixy.BlockArray(100)
        self.newCount = self.bot.pixy.ccc_get_blocks(100, self.newBlocks)

        # 1:Red; 2:Blue; 3: Green; 4: Yellow; 5: Magenta
        self.centerLineID = 1
        self.leftLineID = 2
        self.rightLineID = 3
        self.obstacleID = 4
        self.shortcutID = 5

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

        # Self defined sign function (Returns sign of x)
        self.sign = lambda x: (1, -1)[x < 0]

        self.tp = turningPad(racerBot)

    def updateBlocks(self):
        copyBlockArray(self.newCount, self.newBlocks, self.oldBlocks)
        self.oldCount = self.newCount
        self.newCount = self.bot.pixy.ccc_get_blocks(100, self.newBlocks)

    # Modified: Made it return 1 and 0
    def detectObstacle(self):  # find largest block with the obstacle ID, if it exists, and get its parameters
        self.biggestObstacle = -1
        for i in range(self.newCount):
            if self.newBlocks[i].m_signature == self.obstacleID:
                self.biggestObstacle = i
                break

        if self.biggestObstacle >= 0:
            pixelSize = self.newBlocks[self.biggestObstacle].m_width;
            angleSize = pixelSize / self.pixyMaxX * self.pixyX_FoV  # ???
            self.obstacleSize.append(angleSize)
            self.frameTimes.append(time.time())
            self.dObstacleSize = [
                (self.obstacleSize[i + 1] - self.obstacleSize[i]) / (self.frameTimes[i + 1] - self.frameTimes[i]) for i
                in range(len(self.obstacleSize) - 1)]
            self.obstacleSize.pop(0)

            pixelDistance = self.newBlocks[self.biggestObstacle].m_x - self.pixyCenterX
            angleDistance = pixelDistance / self.pixyMaxX * self.pixyX_FoV
            self.obstacleAngle.append(angleDistance)
            self.dObstacleAngle = [
                (self.obstacleAngle[i + 1] - self.obstacleAngle[i]) / (self.frameTimes[i + 1] - self.frameTimes[i]) for
                i in range(len(self.obstacleAngle) - 1)]
            self.obstacleAngle.pop(0)
            self.frameTimes.pop(0)
            return True
        else:
            return False

    # Modified: Inverted drive setting for right wheel (i.e. lWheelDrive = -rWheelDrive)
    def drive(self, drive, bias):  # Differential drive function
        maxDrive = 1  # set safety limit for the motors

        totalDrive = drive * maxDrive  # the throttle of the car
        diffDrive = bias * totalDrive  # set how much throttle goes to steering
        straightDrive = totalDrive - abs(diffDrive)  # the rest for driving forward (or backward)

        lDrive = straightDrive + diffDrive
        rDrive = -(straightDrive - diffDrive) # Modified due to wheels setup
        self.bot.setTurnRates(lDrive, rDrive)

    def visTrack(self, objectID):  # Get pixycam to rotate to track an object
        if (self.newCount - 1) < objectID or objectID < 0:  # do nothing when we block doesn't exist
            self.bot.setServoPosition(0)
            return -1
        else:
            visError = self.newBlocks[objectID].m_x - self.pixyCenterX  # error in pixels
            visAngularError = -(visError / self.pixyMaxX * self.pixyX_FoV)  # error converted to angle
            visTargetAngle = self.bot.servoPosition + self.gimbal.update(visError)  # error relative to pixycam angle
            if abs(visTargetAngle) > 90:
                visTargetAngle = self.sign(visTargetAngle) * 90
            self.bot.setServoPosition(visTargetAngle)

    # New Addition: Visually Track Between Objects (mean x error between all objects)
    def visTrackBetween(self, objectIDs): 
        if not objectIDs:
            return 0
        else:
            visError = sum([self.newBlocks[x].m_x for x in objectIDs]) / len(objectIDs) - self.pixyCenterX
            visAngularError = -(visError / self.pixyMaxX * self.pixyX_FoV)  # error converted to angle
            visTargetAngle = self.bot.servoPosition + self.gimbal.update(visError)  # error relative to pixycam angle
            if abs(visTargetAngle) > 90:
                visTargetAngle = self.sign(visTargetAngle) * 90
            self.bot.setServoPosition(visTargetAngle)

    def getDistance(self, objectID):
        if (self.newCount - 1) < objectID or objectID < 0:  # do nothing when we block doesn't exist
            return -1

        # camHeight = 0.095 # this is measured for the robot kit
        camHeight = 0.095
        camAngle = 12.0 / 180.0 * math.pi  # this is the camera angle in radian

        pixelDistance = self.pixyCenterY - (self.newBlocks[objectID].m_y + self.newBlocks[objectID].m_height / 2.0)
        angle = pixelDistance / self.pixyMaxY * self.pixyY_FoV / 180.0 * math.pi
        distance = camHeight * math.tan(math.pi / 2.0 - camAngle + angle)

        return distance

    # New Addition: Gets Angle between Objects (mean angle between all objects)
    def getAngle(self, objectIDs):
        if not objectIDs:
            return 0
        else:
            pixelDistance = self.pixyCenterX - sum([self.newBlocks[x].m_x for x in objectIDs]) / len(objectIDs)
            angle = pixelDistance / self.pixyMaxX * self.pixyY_FoV / 180.0 * math.pi
            # distance = camHeight * math.tan(math.pi/2.0 - camAngle + angle)
            return angle

    # New Addition: Creates bias to steer towards Objects (angle is actually a gain)
    def steerTowards(self, objectIDIndex, angle):
        percentageError = 0
        if not objectIDIndex:
            return 0
        else:
            error = [2 * (self.newBlocks[i].m_x / self.pixyMaxX - 0.5) for i in objectIDIndex]
            if len(error) == 1:
                percentageError = error[0]
            elif len(error) == 2:
                percentageError = 0.8 * error[0] + 0.2 * error[1]
            elif len(error) == 3:
                percentageError = 0.65 * error[0] + 0.25 * error[1] + 0.1 * error[2]
            elif len(error) >= 4:
                percentageError = 0.5 * error[0] + 0.3 * error[1] + 0.15 * error[2] + 0.05 * error[-1]
            steering = -angle * percentageError
            return steering

    # New Addition: Creates bias to steer away Objects (angle is actually a gain) 
    # Not Fully Working as intended (i.e. bias should be more when closer but is not)
    def steerAway(self, objectIDIndex, angle):
        bias = -self.steerTowards(objectIDIndex, angle)
        return bias

    # New Addition: Creates bias to steer a set offset away from Objects (angle is actually a gain)
    # Does not really work
    def steerBeside(self, objectIDIndex, angle, offset): # Idea is supposed a set distance away from object
        if not objectIDIndex:
            return 0
        else:
            error = [2 * ((self.newBlocks[i].m_x - offset) / self.pixyMaxX - 0.5) for i in objectIDIndex]
            if len(error) == 1:
                percentageError = error[0]
            elif len(error) == 2:
                percentageError = 0.8 * error[0] + 0.2 * error[1]
            elif len(error) == 3:
                percentageError = 0.65 * error[0] + 0.25 * error[1] + 0.1 * error[2]
            elif lineen(error) >= 4:
                percentageError = 0.5 * error[0] + 0.3 * error[1] + 0.15 * error[2] + 0.05 * error[-1]
            steering = angle * percentageError
            return steering

    # Modified: Performed laneFollowing
    # Modified: Used to perform the whole race
    # Remade (Only does laneFollowing)
    def laneFollowing(self, speed):
        if speed < 0: # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0) # set servo to centre
        self.bot.setMotorSpeeds(0, 0) # set racer to stop
        
        while True:
            self.updateBlocks()
            smallestCenterLine = -1
            smallestObstacle = -1

            centerIndex = []
            leftIndex = []
            rightIndex = []

            for i in range(0, self.newCount): # go through all the blocks from PixyCam to find the center line block
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    lefIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)
                elif self.newBlocks[i].m_signature == self.obstacleID:
                    smallestObstacle = i

            # centerIndex = [i for i in range(0,len(self.newCount)) if self.newBlocks[i] == self.centerLineID]

            if not centerIndex: # stop the racer and wait for new blocks
                self.bot.setMotorSpeeds(0, 0)

            else: # drive while we see a center line
            ###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error. 
            ### Come up with a steering command to send to self.drive(speed, steering) function
                bias = self.steerTowards(centerIndex) # Steer towards the centerIndex line
                self.drive(speed,bias)

    # Modified: Performs lane following and stops when obstacle within range
    def stopAtStationaryObstacles(self, speed, targetDist, targetAngle):
        # make sure the speed never goes to unreasonable values
        if speed < 0:
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop

        while True:
            self.updateBlocks()

            targetSpeed = speed
            bias = 0

            centerIndex = []
            leftIndex = []
            rightIndex = []

            for i in range(0, self.newCount):  # go through all the blocks from PixyCam to find the center line block
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    leftIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)

            if not centerIndex:  # stop the racer and wait for new blocks
                self.bot.setMotorSpeeds(0, 0)

            else:  # drive while we see a center line
                self.visTrack(centerIndex[0])
                bias = -self.bot.servoPosition / 90
                # bias = self.steerTowards(centerIndex,0.4)
                if self.detectObstacle():
                    dist = self.getDistance(self.biggestObstacle)

                    if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (
                            abs(self.obstacleAngle[0]) < targetAngle):
                        targetSpeed = 0
                else:
                    # self.visTrack(centerIndex[0])
                    self.bot.setServoPosition(0)
                self.drive(targetSpeed, bias)

    # Modified: Performs lane following and avoids obstacles
    # Kind of works...
    def avoidStationaryObstacles(self, speed, targetDist, targetAngle):
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)
        biasList = []
        servoList = []

        while True:
            self.updateBlocks()
            self.detectObstacle()

            targetSpeed = speed
            bias = 0

            centerIndex = []
            leftIndex = []
            rightIndex = []

            for i in range(0, self.newCount):  # go through all the blocks from PixyCam to find the center line block
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    leftIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)

            # If an obstacle was seen in the past 0.4 seconds
            if (time.time() - self.frameTimes[-1] < 0.4):
                bias = self.steerTowards(centerIndex, 0.4)
                dist = self.getDistance(self.biggestObstacle)
                # If the obstacle is within distance and angle
                if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (abs(self.bot.servoPosition) < targetAngle):
                    self.visTrack(self.biggestObstacle)
                    bias = 0.5*self.bot.servoPosition/90 # Lvl 5
                    servoList.append(self.bot.servoPosition)
                # After passing the object, retrace the "arc" it took to avoid obstacle
                elif (time.time() - self.frameTimes[-1] > 0.05) and servoList and not (self.bot.servoPosition == 0):    
                    bias = -0.7*servoList[-1]/90
                    self.bot.setServoPosition(servoList[-1])
                    servoList.pop(-1)
                print(len(biasList))

            elif not centerIndex: # stop the racer and wait for new blocks
                targetSpeed = 0

            else:   # drive while we see a center line
                bias = self.steerTowards(centerIndex,0.4)  
                self.bot.setServoPosition(0)
                servoList = []

            self.drive(targetSpeed, bias)

    # Not Completed
    def avoidMovingObstacles(self, speed): 
        ###Level 6 Bonus### If you feel comfortable, try to implement a behaviour to avoid a moving obstacle
        return

    # New Addition (Follows beside leftLane - Doesn't Work Well)
    def altLaneFollowing(self, speed): # An Alternate Method to do Lane Following
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop
        previousSpeed = 0
        targetSpeed = 0

        while True:
            self.updateBlocks()
            smallestCenterLine = -1
            smallestObstacle = -1
            bias = 0 # reset the bias after each iteration
            previousSpeed = targetSpeed
            targetSpeed = speed

            centerIndex = []
            leftIndex = []
            rightIndex = []

            for i in range(0, self.newCount):
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    leftIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)
                elif self.newBlocks[i].m_signature == self.obstacleID:
                    smallestObstacle = i

            if not leftIndex:  
                targetSpeed = 0.1 * targetSpeed

            else:  # drive while we see a left line
                self.bot.setServoPosition(0)
                bias = self.steerBeside([leftIndex[0]], 0.4, self.leftOffset) # sets bias to steer beside the leftLane

            self.drive(targetSpeed, bias)

    # New Addition (Follows path between leftLane and Obstacle)
    def followVisTrack(self, speed):
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop
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

            for i in range(0, self.newCount):  # go through all the blocks from PixyCam to find the center line block
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    leftIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)

            # currently set to drive between leftIndex and Obstacle
            if leftIndex and self.detectObstacle():
                self.visTrackBetween([leftIndex[0], self.biggestObstacle])
                bias = -self.bot.servoPosition / 90
            else:
                targetSpeed = 0.5 * speed
                # Idle Function
                servoPos = self.bot.servoPosition
                if abs(servoPos) > 50:
                    sign = -sign
                    servoPos = servoPos + 2 * 5 * sign
                else:
                    servoPos = servoPos + 5 * sign
                self.bot.setServoPosition(servoPos)
                self.drive(targetSpeed, bias)
            # print([targetSpeed, bias, self.bot.servoPosition])
            self.drive(targetSpeed, bias)

    # New Addition (Testing Code to set Idle Behavior - Not Used)
    def idle(self, speed):
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop
        previousSpeed = 0
        targetSpeed = 0
        sign = 1

        while True:
            bias = 0

            targetSpeed = 0.2 * speed
            servoPos = self.bot.servoPosition
            if abs(servoPos) > 50:
                sign = -sign
                servoPos = servoPos + 2 * 5 * sign
            else:
                servoPos = servoPos + 5 * sign
            self.bot.setServoPosition(servoPos)
            self.drive(targetSpeed, bias)

    # New Addition (Testing Code to test Visually Tracking Method)
    def newRace(self, speed):
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop
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

            for i in range(0, self.newCount):  # go through all the blocks from PixyCam to find the center line block
                if self.newBlocks[i].m_signature == self.centerLineID:
                    centerIndex.append(i)
                elif self.newBlocks[i].m_signature == self.leftLineID:
                    leftIndex.append(i)
                elif self.newBlocks[i].m_signature == self.rightLineID:
                    rightIndex.append(i)
                elif self.newBlocks[i].m_signature == self.shortcutID:
                    shortcut = i

            # centerIndex = [i for i in range(0,len(self.newCount)) if self.newBlocks[i] == self.centerLineID]
            # centerIndex = []

            if (time.time() - self.shortcutFirstTime < 0.25) and not (self.shortcutFirstTime == float('nan')):
                bias = -0.2 + self.steerTowards(leftIndex[0:1], 0.6)
                self.bot.setServoPosition(0)
                print(['Shorcut: Held', bias])

            elif shortcut > 0:
                self.shortcutFirstTime = time.time()
                bias = -0.6
                print('Shortcut: First Time')

            elif (time.time() - self.frameTimes[-1] < 0.4):
                print("Obstacle Routine: Detected")
                bias = self.steerTowards(centerIndex, 0.4)
                dist = self.getDistance(self.biggestObstacle)
                if (dist - targetDist < 0.02) and (dist - targetDist > -1) and (
                        abs(self.bot.servoPosition) < targetAngle):
                    self.visTrack(self.biggestObstacle)
                    bias = 0.5 * self.bot.servoPosition / 90  # Lvl 5
                    servoList.append(self.bot.servoPosition)
                # print(biasList)
                elif (time.time() - self.frameTimes[-1] > 0.05) and servoList and not (self.bot.servoPosition == 0):
                    # bias = -self.bot.servoPosition/90
                    # print(biasList)
                    bias = -0.7 * servoList[-1] / 90
                    self.bot.setServoPosition(servoList[-1])
                    servoList.pop(-1)
            # print(len(biasList))

            elif centerIndex:
                print("CenterIndex: Detected")
                self.visTrackBetween(centerIndex)
                bias = -self.bot.servoPosition / 90
                if leftIndex:
                    self.leftOffset = math.floor(
                        self.leftOffset + (self.newBlocks[leftIndex[0]].m_x - self.pixyMaxX / 2)) / 2
                if rightIndex:
                    self.rightOffset = math.floor(
                        self.rightOffset + (self.newBlocks[rightIndex[0]].m_x - self.pixyMaxX / 2)) / 2

            elif leftIndex or rightIndex:
                print("Left/Right Index: Detected")

                targetSpeed = 0.5 * targetSpeed
                lbias = self.steerAway(leftIndex, 0.4)
                rbias = self.steerAway(rightIndex, 0.4)
                bias = -servoPos / 90

                if abs(servoPos) > 10:
                    servoPos = servoPos - (self.sign(servoPos) * 10)
                else:
                    servoPos = 0
                self.bot.setServoPosition(servoPos)

            else:
                print("Idle: Nothing")
                targetSpeed = 0.5 * speed
                if abs(servoPos) > 90:
                    sign = -self.sign(servoPos)
                    servoPos = servoPos + 2 * 5 * sign
                else:
                    servoPos = servoPos + 5 * sign
                self.bot.setServoPosition(servoPos)
            # print(sign)

            self.drive(targetSpeed, bias)

    # New Addition (Testing Code to dodge obstacles using manual inputs)
    def dodge(self, direction):
        print("Entering dodge()")
        st = [0.2, -0.21]
        turn90 = [0.2, 0.2]
        stop = [0, 0]

        st_t = 0.7
        turn_t = 0.5
        stop_t = 0.1

        print("Turn 90 array:", turn90 * direction)

        durations = [turn_t, stop_t, st_t, stop_t,
                     turn_t, stop_t, st_t, stop_t,
                     turn_t, stop_t, st_t, stop_t,
                     turn_t, stop_t, st_t, stop_t, ]
        speeds = [turn90 * direction, stop, st, stop,
                  turn90 * direction * -1, stop, st, stop,
                  turn90 * direction * -1, stop, st, stop,
                  turn90 * direction, stop, st, stop]

        self.tp.deadReckoningTrack(durations, speeds)

    # New Addition (Code Used for Final Race)
    def finalRace(self, speed):
        if speed < 0:  # make sure the speed never goes to unreasonable values
            speed = 0
        elif speed > 1:
            speed = 1

        self.bot.setServoPosition(0)  # set servo to centre
        self.bot.setMotorSpeeds(0, 0)  # set racer to stop

        previousSpeed = 0
        targetSpeed = 0
        servoList = []

        firstTime = True
        secondTime = False

        while True:
            self.updateBlocks()
            self.detectObstacle()
            smallestCenterLine = -1
            smallestObstacle = -1
            shortcut = -1
            bias = 0
            previousSpeed = targetSpeed
            targetSpeed = speed
            targetDist = 0.5
            targetAngle = 50

            centerIndex = []
            leftIndex = []
            rightIndex = []

            for i in range(0, self.newCount):  # go through all the blocks from PixyCam to find the center line block
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

            if shortcut > 0:
                print("sp")
                firstTime = False
                self.shortcutFirstTime = time.time()
                targetSpeed = 0.75
                while (time.time() - self.shortcutFirstTime < 0.44):
                    bias = -0.35
                    self.drive(targetSpeed * 0.8, bias)

            elif not centerIndex:
                dist = self.getDistance(self.biggestObstacle)
                loop_start = 0
                if (dist - targetDist < 0.02) and (dist - targetDist > -1):
                    targetSpeed = 0.7

                    lAngle = 0
                    rAngle = 0
                    yAngle = 0
                    
                    if leftIndex:
                        lAngle = self.getAngle(leftIndex)                        

                    if rightIndex:
                        rAngle = self.getAngle(rightIndex)                        

                    if self.biggestObstacle >= 0:
                        yAngle = self.getAngle([self.biggestObstacle])
                        
                    if not lAngle == 0 and not rAngle == 0 and not yAngle == 0:
                        lDiff = lAngle - yAngle
                        rDiff = rAngle - yAngle
                        if abs(lDiff) > abs(rDiff):
                            bias = 0.18
                            print("sBgY")
                            loop_start = time.time()                       
                        else:
                            bias = -0.18
                            print("sbGY")
                            loop_start = time.time()

                        targetSpeed = 0.5 * targetSpeed
                        while time.time() - loop_start < 0.3:
                            self.drive(targetSpeed, bias)
                            

                    elif not rAngle == 0 and not yAngle == 0:  # If no leftLane
                        rDiff = rAngle - yAngle
                        if abs(rDiff) >= 0.4 and (abs(rDiff) < 0.65):
                            loop_start = time.time()
                            bias = -0.25
                            print("sgy")
                        elif (abs(rDiff) < 0.4):
                            loop_start = time.time()
                            bias = 0.25
                            print("sag")

                        targetSpeed = 0.5 * targetSpeed
                        while time.time() - loop_start < 0.3:
                            self.drive(targetSpeed, bias)

                    elif not lAngle == 0 and not yAngle == 0:  # If no rightLane
                        lDiff = lAngle - yAngle
                        if abs(lDiff) >= 0.65:
                            bias = 0
                            print("sby")
                        elif (abs(lDiff) >= 0.4) and (abs(lDiff) < 0.65):
                            loop_start = time.time()
                            bias = -0.12
                        else:
                            loop_start = time.time()
                            bias = -0.18
                            print("sab")
                            
                        targetSpeed = 0.5 * targetSpeed
                        while time.time() - loop_start < 0.3:
                            self.drive(targetSpeed, bias)
                            
                else:
                    lbias = 0
                    rbias = 0

                    if len(leftIndex) > 0:
                        lAngle = self.getAngle(leftIndex)                        
                        targetSpeed = 0.6 * targetSpeed
                        if abs(lAngle) <= 0.2:
                            print("sal")
                            lbias = -0.15
                    if len(rightIndex) > 0:
                        rAngle = self.getAngle(rightIndex)                        
                        targetSpeed = 0.6 * targetSpeed
                        if abs(rAngle) <= 0.2:
                            print("sar")
                            rbias = 0.15

                    bias = lbias + rbias

            elif centerIndex:  # drive while we see a center line

                redDist = self.getDistance(centerIndex[0])
                if (not firstTime) and (redDist < 0.17):
                    firstTime = True
                    print("se")
                    self.shortcutFirstTime = time.time()
                    targetSpeed = 0.72
                    while (time.time() - self.shortcutFirstTime < 0.35):
                        bias = -0.3
                        self.drive(targetSpeed * 0.85, bias)

                self.bot.setServoPosition(0)
                bias = self.steerTowards(centerIndex, 0.4)

                if leftIndex:
                    self.leftOffset = math.floor(
                        self.leftOffset + (self.newBlocks[leftIndex[0]].m_x - self.pixyMaxX / 2)) / 2
                if rightIndex:
                    self.rightOffset = math.floor(
                        self.rightOffset + (self.newBlocks[rightIndex[0]].m_x - self.pixyMaxX / 2)) / 2

            self.drive(targetSpeed, bias)