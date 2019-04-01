from pixyRacer import pixyRacer
from turningPad import turningPad

def singleWheelTurn(tp, driveLevel, wheelSide, time):
	tp.singleWheelTurn(driveLevel, wheelSide, time)

def straightLine(tp, speed, runTime=None, correction=[0, 0]):
	tp.straightLine(speed, correction, runTime)

def symmetricTurn(tp, rotationRate, runTime=None):
	tp.symmetricTurn(rotationRate, runTime)

def deadReckoningCourse(tp):

	# program a sequence of robot commands to complete the deadreckoning game
	# Hint: add breaks between consecutive motor commands (~1s long)

	# list durations of motor settings (including pauses) here in order of execution
	durations = []
	# list corresponding motor settings (i.e. list of lists) for each time period in the list above
	motorSpeeds = []

	tp.deadReckoningTrack(durations, motorSpeeds)


def main():

	servoCorrection = 0 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	tp = turningPad(racer)

	#singleWheelTurn(tp, 50, 'left', 10)

	#straightLine(tp, 10, 10)

	#symmetricTurn(tp)

	#deadReckoningCourse(tp)

main()
