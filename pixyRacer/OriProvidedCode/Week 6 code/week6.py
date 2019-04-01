from pixyRacer import pixyRacer
from obstacleAvoidance import obstacleAvoidance

def main():
	servoCorrection = 3 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	racer.setServoPosition(0)	
	oa = obstacleAvoidance(racer)

	#oa.laneFollowing(0.8)
	#oa.avoidStationaryObstacles(0.3)
	oa.avoidMovingObstacles(0.2)

	
main()