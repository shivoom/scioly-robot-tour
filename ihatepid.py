import qwiic_otos
import time
import math
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
offset = 0
goalheading = 45
while True:
    myPosition = odo.getPosition()
    offset = ((goalheading - abs(myPosition.h))/goalheading)
    print(myPosition.h, " deg")
    print(" ")
    leftmotor.start(20 + (50*(offset)))
    rightmotor.start(-20 + (50*(offset)))
    time.sleep(0.01)