import qwiic_otos
from buildhat import Motor
from simple_pid import PID
import time
import sys

def pid():
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

        left_motor = Motor('A')
        right_motor = Motor('B')

        #defining position
        target_y = 10
        target_x = 10


        pid_y = PID(1.0, 0.0, 0.05, setpoint = target_y)
        pid_x = PID(1.0, 0.0, 0.05, setpoint = target_x)


        while True:
                myPosition = odo.getPosition()
                current_x = myPosition.y 
                current_y = myPosition.x
                #calculating error for x and y
                error_y = target_y - current_y
                error_x = target_x - current_x
                
                #update pid controller with error values
                linear_speed = pid_y(error_y) # y measures forward and backwards therefore linear
                angular_speed = pid_x(error_x) # x measures right and left therefore angular (essentially drift)
                
                #calculating speed for each motor
                left_speed = ((linear_speed - angular_speed))
                right_speed = ((-1 * (linear_speed + angular_speed)))
                left_motor.set_default_speed(left_speed)
                right_motor.set_default_speed(right_speed)
                #applying speed to motors
                left_motor.start()
                right_motor.start()
                print("current x: ", myPosition.x)
                print("current y: ", myPosition.y)
                print("right speed: ", (right_speed))
                print("left speed", (left_speed))
                time.sleep(0.5)

if __name__ == '__main__':
    try:
        pid()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example")
        sys.exit(0)