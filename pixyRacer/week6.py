from pixyRacer import pixyRacer
from obstacleAvoidance_Ze import obstacleAvoidance

def main():
	servoCorrection = 20 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	racer.setServoPosition(0)	
	oa = obstacleAvoidance(racer)

	oa.laneFollowing(0.7) #old code
	#oa.laneFollowing(0.3, 0.3)
	#oa.stopAtStationaryObstacles(0.3,0.3,30)
	#oa.avoidStationaryObstacles(0.3,0.3,75)
	#oa.avoidMovingObstacles(0.3,0.3,30)
	#oa.altLaneFollowing(0.3)
	#oa.idle(0.5)
	#oa.followVisTrack(0.3)
	#oa.newRace(0.7)

	
main()
