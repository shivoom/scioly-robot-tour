import qwiic_otos
import sys
import time

# Set name and initialize
odo = qwiic_otos.QwiicOTOS()
odo.begin()

for i in range(3, 0, -1):
        print("Calibrating in %d seconds..." % i)
        time.sleep(1)

print("Calibrating...")

# Calibrate the IMU, which removes the accelerometer and gyroscope offsets
odo.calibrateImu()

# Set Units
odo.setLinearUnit(odo.kLinearUnitMeters)
odo.setAngularUnit(odo.kAngularUnitDegrees)

odo.resetTracking()

# Setup complete

# Main loop
while True:
        # Get the latest position, which includes the x and y 
        # coordinates, plus the heading angle
        myPosition = odo.getPosition()

        # Print measurement
        print()
        print("Position:")
        print("X (Meters): {}".format(myPosition.x))
        print("Y (Meters): {}".format(myPosition.y))
        print("Heading (Radians): {}".format(myPosition.h))

        # Wait a bit so we don't spam the serial port
        time.sleep(0.5)