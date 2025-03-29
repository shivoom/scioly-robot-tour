from simple_pid import PID
import qwiic_otos
import time
import math
from buildhat import Motor
# Initialize motors:
def initialization():
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
    # Reset odo to all zeros
    odo.resetTracking()
    return odo, rightmotor, leftmotor
odo, rightmotor, leftmotor = initialization()

def goalcalc(goalx, goaly):
    myPosition = odo.getPosition()
    # Forward is positive x, left is positive y (so invert). Multiplied by 100 to convert from meters to centimeters
    xdisp = goalx + myPosition.y*100
    ydisp = goaly - myPosition.x*100
    #print("x position: ", myPosition.y)
    #print("y position: ", myPosition.x)
    # Determine goalheading based on the quadrant of movement on a coordinate plane with origin at initial position
    if xdisp == 0:
        if ydisp > 0:
             goalheading = 0
        elif ydisp < 0:
             goalheading = 180
    if ydisp == 0:
        if xdisp > 0:
             goalheading = 90
        elif xdisp < 0:
             goalheading = -90
    if xdisp > 0 and ydisp > 0:
        goalheading = math.degrees(math.atan(abs(xdisp/ydisp)))
    if xdisp > 0 and ydisp < 0:
        goalheading = 90 + math.degrees(math.atan(abs(ydisp/xdisp)))
    if xdisp < 0 and ydisp > 0:
        goalheading = -math.degrees(math.atan(abs(xdisp/ydisp)))
    if xdisp < 0 and ydisp < 0:
        goalheading = -90 - math.degrees(math.atan(abs(ydisp/xdisp)))
    #print("calculated xdisp: ", xdisp)
    #print("calculated ydisp: ", ydisp)
    #print("calculated goalheading: ", goalheading)
    return goalheading, xdisp, ydisp

def linearcontrol(goalx, goaly, goalheading):
    myPosition = odo.getPosition()
    currentx = -myPosition.y*100
    currenty = myPosition.x*100
    print("Location: ", currentx, " ", currenty)
    heading = -myPosition.h
    displacement = math.dist((goalx, goaly), (currentx, currenty))
    
    # Slows linear speed when pointing the wrong way
    if abs(goalheading - heading) < 15:
        directioncorrect = 1
    elif abs(goalheading - heading) >= 15:
        directioncorrect = 1 / abs(goalheading - heading)
    
    # Slows down on approach to goal point
    if displacement < 10:
        closeslowdown = (displacement / 20)
    elif displacement >= 10:
        closeslowdown = 1
    
    # Combine the two slowdown coefficients together
    linearspeed = 5 + 35*directioncorrect*closeslowdown
    return linearspeed, displacement
    

def goto(goalx, goaly):
    while True:
        myPosition = odo.getPosition()
        heading = -myPosition.h
        goalheading, xdisp, ydisp = goalcalc(goalx, goaly)
        # Compute new output from the PID according to the system's current value
        pid.setpoint = goalheading
        linearspeed, displacement = linearcontrol(goalx, goaly, goalheading)
        #print("linearspeed: ", linearspeed)
        # Slows down on approach to goal point
        if displacement < 10:
            closeslowdownturn = (displacement / 8)
        elif displacement >= 10:
            closeslowdownturn = 1
        controlright = (1 * pid(heading) * closeslowdownturn) - (1 * linearspeed)
        controlleft = (1 * pid(heading) * closeslowdownturn) + (1 * linearspeed)
        if controlleft > 100:
            #print("control left was too big! ", controlleft)
            controlleft = 100    
        if controlleft < -100:
            controlleft = -100
            #print("control left was too small! ", controlleft)
        if controlright > 100:
            #print("control right was too big! ", controlright)
            controlright = 100
        if controlright < -100:
            #print("control right was too small! ", controlright)
            controlright = -100
       # print("Current:", heading, " degrees")
        #print("Goal: ", goalheading, " degrees")
        #print("Pid value: ", pid(heading))
        leftmotor.start(controlleft)
        rightmotor.start(controlright)
        #print("Displacement ", displacement)
        if displacement < 2:
            break
        time.sleep(0.002)

time.sleep(1)
# PID Attempt one: 0.22, 0.01, 0.3 Best Coords: (50, 49)
pid = PID(0.4, 0.02, 0.2)
goto(0, 75)
print(" ")
print(" ")
print(" ")
print("end of goto 1")
goto(0, 25)