import qwiic_otos
import time
import math
from simple_pid import PID
from buildhat import Motor
# Initialize motors:
leftmotor = Motor('A')
rightmotor = Motor('B')

# Set name and initialize
odo = qwiic_otos.QwiicOTOS()
odo.begin()

for i in range(1, 0, -1):
        print("Calibrating in %d seconds..." % i)
        time.sleep(1)

print("Calibrating...")

# Calibrate the IMU, which removes the accelerometer and gyroscope offsets
odo.calibrateImu()

# Set Units
odo.setLinearUnit(odo.kLinearUnitMeters)
odo.setAngularUnit(odo.kAngularUnitDegrees)

odo.resetTracking()

# Initial point (cm):
currentx = 0
currenty = 0
currenth = 0

# Goal point (cm):
goalx = 1
goaly = 50

# PID Setup
headingpid = PID(1, 0.1, 0.05)
# Setup complete

def wya ():
        # Get the latest position, which includes the x and y 
        # coordinates, plus the heading angle
        myPosition = odo.getPosition()

        currentx = (myPosition.y * -100)
        currenty = (myPosition.x * 100)
        currenth = (myPosition.h)
        currentheading = myPosition.h
        print("goal heading: ", math.tan(abs(goaly)/abs(goalx)))
        headingpid.setpoint = 90
        pidcoeff = headingpid(currentheading)
        motorbalance = pidcoeff / 90
        print(pidcoeff)
        print(currentx)
        print(currenty)
        print(currenth, " deg")
        print(" ")
        leftmotor.start(10 + (5*(motorbalance)))
        rightmotor.start(0 - (5*(motorbalance)))

        # Wait a bit so we don't spam the serial port
        return(currentx, currenty)

while True:
        currentx, currenty = wya()

        time.sleep(0.1)