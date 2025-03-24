
import qwiic_otos
import sys
import time

def runExample():
    print("\nQwiic OTOS Example 2 - Set Units\n")

    # Create instance of device
    myOtos = qwiic_otos.QwiicOTOS()

    # Check if it's connected
    if myOtos.is_connected() == False:
        print("The device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    # Initialize the device
    myOtos.begin()

    print("Ensure the OTOS is flat and stationary during calibration!")
    for i in range(5, 0, -1):
        print("Calibrating in %d seconds..." % i)
        time.sleep(1)

    print("Calibrating IMU...")

    # Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu()

    # Set the desired units for linear and angular measurements. Can be either
    # meters or inches for linear, and radians or degrees for angular. If not
    # set, the default is inches and degrees. Note that this setting is not
    # stored in the sensor, it's part of the library, so you need to set at the
    # start of all your programs.
    myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
    myOtos.setAngularUnit(myOtos.kAngularUnitDegrees)

    # Reset the tracking algorithm - this resets the position to the origin,
    # but can also be used to recover from some rare tracking errors
    myOtos.resetTracking()

    # Main loop
    while True:
        # Get the latest position, which includes the x and y coordinates, plus
        # the heading angle
        myPosition = myOtos.getPosition()

        # Print measurement
        print()
        print("Position:")
        print("X (Meters): {}".format(myPosition.x))
        print("Y (Meters): {}".format(myPosition.y))
        print("Heading (Radians): {}".format(myPosition.h))

        # Wait a bit so we don't spam the serial port
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example")
        sys.exit(0)