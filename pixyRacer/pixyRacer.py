import pigpio
import pixy
import time

class Servo(object):
    def __init__(self, servo_pin, pi):
        self.servo_pin = servo_pin
        self.pi = pi
    
    def setPosition(self, position):
        position = 90 - position
        if position < 0:
            position = 0
        if position > 180:
            position = 180
        self.pi.set_servo_pulsewidth(self.servo_pin, int(600 + position/180.0*1800))

class Motor(object):

    def __init__(self, pwm_pin, dir_pin, pi):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pi = pi
        self.MAX_SPEED = 100

    def setSpeed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 1
        else:
            dir_value = 0

        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED

        self.pi.write(self.dir_pin, dir_value)
        self.pi.hardware_PWM(self.pwm_pin, 20000, int(speed * 10000));
        # 20 kHz PWM, duty cycle in range 0-1000000 as expected by pigpio

class pixyRacer(object):
    def __init__(self, servoCorrection=0, pi=None, pin_Servo=14, pin_M1PWM=12, pin_M1DIR=24, \
                pin_M2PWM=13, pin_M2DIR=25, pin_nFAULT=6, pin_nEN=5):
        # low level robot control
        if not pi == None:
            self.pi = pi
        else:
            self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Can't connect to pigpio")
            
        self.servoCorrection = servoCorrection
        self.pin_Servo = pin_Servo
        self.pin_M1PWM = pin_M1PWM
        self.pin_M1DIR = pin_M1DIR
        self.pin_M2PWM = pin_M2PWM
        self.pin_M2DIR = pin_M2DIR
        self.pin_nFAULT = pin_nFAULT
        self.pin_nEN = pin_nEN
        self.motor1 = Motor(self.pin_M1PWM, self.pin_M1DIR, self.pi)
        self.motor2 = Motor(self.pin_M2PWM, self.pin_M2DIR, self.pi)
        self.servo = Servo(self.pin_Servo, self.pi)
        self.pi.set_pull_up_down(self.pin_nFAULT, pigpio.PUD_UP)
        self.pi.write(self.pin_nEN, 0)
        self.pixy = pixy
        self.pixy.init()

        self.servoPosition = 0

        # hign level robot control
        self.maxSpeed = 100
        # a and b are arranged as 
        # [[left wheel forwards, right wheel forwards], [left wheel backwards, right wheel backwards]]
        self.a = [[-63.45, -69], [-53.5, -62]]
        self.b = [[5.11, 5.775], [5.125, 5.65]]
        # maximum turn rate of the wheels that we want is 95% of the slowest wheel
        # because of non-linearities at the extremes
        self.maxT = 95 
        self.maxTurnRate = min(  (self.a[0][0] + self.b[0][0]*self.maxT), 
                                 (self.a[0][1] + self.b[0][1]*self.maxT), 
                                 (self.a[1][0] + self.b[1][0]*self.maxT), 
                                 (self.a[1][1] + self.b[1][1]*self.maxT))

    def __del__(self):
        self.forceStop()
        
    def setMotorSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)
        
    def setServoPosition(self, position):
        self.servoPosition = position
        self.servo.setPosition(position+self.servoCorrection)
        
    def getFault(self):
        return not self.pi.read(self.pin_nFAULT)

    def enable(self):
        self.pi.write(self.pin_nEN, 0)

    def disable(self):
        self.pi.write(self.pin_nEN, 1)
        
    def forceStop(self):
        # reinitialize the pigpio interface in case we interrupted another command
        # (so this method works reliably when called from an exception handler)
        
        self.pi.stop()
        self.pi = pigpio.pi()
        self.pi.hardware_PWM(self.pin_M1PWM, 20000, 0);
        self.pi.hardware_PWM(self.pin_M2PWM, 20000, 0);
        self.pi.stop()

    # l         - percentage of maximum turning rate to set the left wheel to (0-1)
    # r         - percentage of maximum turning rate to set the right wheel to (0-1)
    # runTime   - (optional) amount of time ([s]) to run motors for
    #           -  if not set, motors will run until the next command is issued
    def setTurnRates(self, l, r, runTime=None):

        # TODO: idiot proofing, make sure l & r are between -1 and 1

        if l >= 0:
            lSpeed = (self.maxTurnRate*l-self.a[0][0])/self.b[0][0]
        elif l < 0:
            lSpeed = -((self.maxTurnRate*(-l)-self.a[1][0])/self.b[1][0])

        if r >= 0:
            rSpeed = (self.maxTurnRate*r-self.a[0][1])/self.b[0][1]
        elif r < 0:
            rSpeed = -((self.maxTurnRate*(-r)-self.a[1][1])/self.b[1][1])

        self.setMotorSpeeds(rSpeed, lSpeed)

        # None > 0 evaluates to false
        if runTime > 0:

            t_start = time.time()
            t_end = t_start + runTime

            while time.time() < t_end:
                pass
                
            self.setMotorSpeeds(0, 0)

        return


