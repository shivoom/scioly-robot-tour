from time import sleep
sleep(2)
from buildhat import Motor # type: ignore
from buildhat import MotorPair
import math
import RPi.GPIO as GPIO

# Right motor is A, Left is B
motora = Motor('A')
motorb = Motor('B')
gearing = 3 # constant based on gear ratio (final drive ratio)
overshoot = -0.1 # correction based on surface friction (rotations)
wheelradius = 3.5 # in centimeters for racecar wheels
wheelcircumfrence = 2 * math.pi * wheelradius
fiftycm = gearing * ((50 / wheelcircumfrence) + overshoot) # rotations to travel 50cm
turningconstant = gearing * (0.25) # rotations to rotate 90 degrees about the other wheel
speed = 75
tspeed = 25
buttonpin = 37
done = 0

GPIO.setmode(GPIO.BOARD) # Numbers GPIOs by physical location
GPIO.setup(buttonpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def right ():
    motora.run_for_rotations(-turningconstant, speed = tspeed, blocking=False)
    motorb.run_for_rotations(-turningconstant, speed = tspeed)
def forward ():
    motora.run_for_rotations(-fiftycm, speed = speed + 5, blocking=False)
    motorb.run_for_rotations(fiftycm, speed = speed) 
def left ():
    motora.run_for_rotations(turningconstant + .05, speed = tspeed, blocking=False)
    motorb.run_for_rotations(turningconstant +.05, speed = tspeed)
def back ():
    motora.run_for_rotations(fiftycm, speed = speed, blocking = False)
    motorb.run_for_rotations(-fiftycm, speed = speed)
def halfforward ():
    motora.run_for_rotations(-0.5 * fiftycm, speed = speed, blocking = False)
    motorb.run_for_rotations(0.5 * fiftycm, speed = speed)
def halfback ():
    motora.run_for_rotations(0.5 * fiftycm, speed = speed, blocking = False)
    motorb.run_for_rotations(-0.5 * fiftycm, speed = speed)
def motorready ():
    motora.run_for_rotations(0.3, speed = speed, blocking = False)
    motorb.run_for_rotations(-0.3, speed = speed)

# wait until button is pressed to begin
def ready (ev=None):
    print('complete')
    global done
    done = 1

motorready()

GPIO.add_event_detect(buttonpin, GPIO.FALLING, callback=ready, bouncetime=200)
while done == 0:
    sleep(1)

sleep(2) # time to remove hand

# add instructions below:
halfforward()
sleep(2)
forward()
sleep(2)
forward()
sleep(2)
forward()
sleep(40)

