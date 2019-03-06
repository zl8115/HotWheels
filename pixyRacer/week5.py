from pixyRacer import pixyRacer
from tuningCurve import tuningCurve
from helper import PID_controller

def predicate(timePoints, time, errors, angles):
	servoPos = 0

	# use the timePoints, angles and errors to calculate where the stimulus will reappear!  
	return servoPos

def main():
	servoCorrection = 20 # put the servo correction for your robot here
	racer = pixyRacer(servoCorrection)
	racer.setServoPosition(0)
	# for the second part of the visual tracking tuning curves, improve this controller
	# either by tuning the PID gain values, or by writing an entirely new controller!
	# any controller has to have an 'update' function that takes in an error and returns a control
	servoController = PID_controller(0.06, 0.00, 0.06)
	tc = tuningCurve(racer, servoController)

	# visual tracking tuning curves
	f = 2.0 # set the frequency to the stimulus frequency selected in Matlab
	tc.measure(f)

	# visual predication
	startPos = 0 # set a starting angle for the servo
	result, error = tc.visualPredication(predicate, startPos)


main()
