from pixyRacer import pixyRacer
from obstacleAvoidance_Ze import obstacleAvoidance

def main():
	servoCorrection = 20 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	racer.setServoPosition(0)	
	oa = obstacleAvoidance(racer)

	oa.laneFollowing(1) #old code
	#oa.laneFollowing(0.3, 0.3)
	#oa.stopAtStationaryObstacles(0.3,0.3,30)
	# oa.avoidStationaryObstacles(0.3,0.3,50)
	#oa.avoidMovingObstacles(0.3,0.3,30)

	
main()
