from pixyRacer import pixyRacer
from turningPad import turningPad

var = 0.4
ti = 0.63
cor = [0,-0.02]

# 200mm straight
#deadt = [1]
#deadm = [[0.2,-0.204]]

# 90 left
#deadt = [1.4784]
#deadm = [[-0.25,-0.195]]
#deadm =  [[0.2,0.21]]

tst = 1.1
ttu = 0.52


st = [0.2,-0.21]
l90 = [0.2,0.2]
r90 = [-0.20,-0.20]

pa = 0.5
pam = [0, 0]

#deadt = [tst, pa, ttu, pa, tst, pa, ttu, pa, tst, pa, ttu, pa, tst, pa, ttu, pa, tst]
#deadm = [st, pam, l90, pam, st, pam, l90, pam, st, pam, r90, pam, st, pam, r90, pam, st] 

#deadt = [0.8557]
#deadm = [l90]

deadt = [1.55, pa, 1.0384, pa, 2.9, pa, 0.441, pa, 2.45, pa, 0.8857, pa, 4.67, pa, 0.905, pa, 2.725]
deadm = [st, pam, r90, pam, st, pam, l90, pam, st, pam, l90, pam, st, pam, r90, pam, st]

def singleWheelTurn(tp, driveLevel, wheelSide, time):
	tp.singleWheelTurn(driveLevel, wheelSide, time)

def straightLine(tp, speed, runTime=None, correction=[0, 0]):
	tp.straightLine(speed, correction, runTime)

def symmetricTurn(tp, rotationRate, runTime=None):
	tp.symmetricTurn(rotationRate, runTime)

def deadReckoningCourse(tp,durations,motorSpeeds):
	 

	# program a sequence of robot commands to complete the deadreckoning game
	# Hint: add breaks between consecutive motor commands (~1s long)

	# list durations of motor settings (including pauses) here in order of execution
	
	# section 1
	#durations = [	0.762, 0.5, # straight 200mm
	#				0.456, 0.5, # left 90 degrees
	#				0.762, 0.5, # straight 200mm
	#				0.456, 0.5, # left 90 degrees
	#				0.762, 0.5, # straight 200mm
	#				0.458, 0.5, # right 90 degrees
	#				0.762, 0.5, # straight 200mm
	#				0.458, 0.5, # right 90 degrees
	#				1.762, 0.5, # straight 200mm
	#				]
	# list corresponding motor settings (i.e. list of lists) for each time period in the list above
	#motorSpeeds = [	[29.67, 28.67], [0, 0], # straight 200mm
	#				[29.67, -27.51], [0, 0], # left 90 degrees
	#				[29.67, 28.67], [0, 0], # straight 200mm
	#				[29.67, -27.51], [0, 0], # left 90 degrees
	#				[29.67, 28.67], [0, 0], # straight 200mm
	#				[-28.17, 28.67], [0, 0], # right 90 degrees
	#				[29.67, 28.67], [0, 0], # straight 200mm
	#				[-28.17, 28.67], [0, 0], # right 90 degrees
	#				[29.67, 28.67], [0, 0], # straight 200mm
					
	#				]
	
	

	# section 2
	#durations = [	1.143, 0.5, # straight 300mm
	#				0.776, 0.5, # right 160 degrees
	#				2.236, 0.5, # straight 600mm
	#				0.435, 0.5, # left 80 degrees
	#				1.905, 0.5, # straight 500mm
	#				0.730, 0.5, # left 150 degrees
	#				3.481, 0.5, # straight 940mm
	#				0.616, 0.5, # right 160 degrees
	#				3.095, 0.5, # straight 550mm
	#				]
	# list corresponding motor settings (i.e. list of lists) for each time period in the list above
	#motorSpeeds = [	[29.67, 28.67], [0, 0], # straight 300mm
	#				[-28.17, 28.67], [0, 0], # right 160 degrees
	#				[29.67, 28.67], [0, 0], # straight 600mm
	#				[29.67, -27.51], [0, 0], # left 80 degrees
	#				[29.67, 28.67], [0, 0], # straight 500mm
	#				[29.67, -27.51], [0, 0], # left 150 degrees
	#				[29.67, 28.67], [0, 0], # straight 940mm
	#				[-28.17, 28.67], [0, 0], # right 134 degrees
	#				[29.67, 28.67], [0, 0], # straight 550mm
	#				]

	#"""

	#print(durations)
	#print(motorSpeeds)
	tp.deadReckoningTrack(durations, motorSpeeds)


def main():

	servoCorrection = 0 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	tp = turningPad(racer)

	#singleWheelTurn(tp, var, 'right', ti)

	#straightLine(tp, var, ti, cor)

	#symmetricTurn(tp, var, ti)

	deadReckoningCourse(tp,deadt,deadm)

main()
