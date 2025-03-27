import qwiic_otos
import time
import math
import array
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
# Reset odo to all zeros
odo.resetTracking()

prevangleoffset = 0
#offsethistory = array.array('f', [])
heading = 0
Proportional = 0
Derivative = 0
Integral = 0
# Prevents a bug where the first query can give the final position of the last execution and immediately end the program
firsttry = True

def anglecorrect(firsttry, goalheading, prevangleoffset):
    # Proportional term: 
    # This section gets the heading from -180 (left) to 180 (right) and calculates offset from goal:
    myPosition = odo.getPosition()
    heading = -myPosition.h
    if firsttry == True:
          heading = 0
          firsttry = False
    if goalheading > 0:
        Proportional = ((goalheading - heading)/goalheading)
    elif goalheading < 0:
        Proportional = -((goalheading - heading)/goalheading)
    elif goalheading == 0:
        Proportional = 0
    if Proportional > 1:
        Proportional = 1
    elif Proportional < -1:
        Proportional = -1
    print(heading, " deg")
    # Array record of previous P terms
    #offsethistory.append(Proportional)
    #if len(offsethistory) >= 100:
        #offsethistory.remove(0)
    # Integral term: Integral of the last x measurements, basically measures accumulated error
    # Integral = 0
    # for i in range(len(offsethistory)-1, 0, -1):
        #Integral = Integral + (offsethistory[i]*0.1)
    # Derivative term: Rate of change of the proportional term, so it's the previous angle minus the new angle
    Derivative = (prevangleoffset - Proportional)*50
    # Make sure the Derivative term can't be more than one and give the motor speed error...
    if Derivative > 1:
         Derivative = 1
    elif Derivative < -1:
        Derivative = -1
    return heading, Proportional, Proportional, Derivative, firsttry

def headcalc(goalx, goaly):
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

def distleft(xdisp, ydisp):
    distanceleft = (math.sqrt(xdisp**2 + ydisp**2))
    if distanceleft < 5:
        speedmult = distanceleft / 10
    else:
        speedmult = 1
    return speedmult, distanceleft

def goto(firsttry, prevangleoffset, goalx, goaly):
    while True:
        # Calculate the direction the robot needs to travel and current remaining displacements to goal point
        goalheading, xdisp, ydisp = headcalc(goalx, goaly)
        # Orientation correction PID function
        heading, Proportional, prevangleoffset, Derivative, firsttry = anglecorrect(firsttry, goalheading, prevangleoffset)
        # Linear movement function
        speedmult, distanceleft = distleft(xdisp, ydisp)
        print("goalheading: ", goalheading)
        if goalheading == 0:
            linearspeed = 5 + 35*(speedmult)
        else:
            # Slows linear speed when pointing the wrong way
            if abs((goalheading - heading)/goalheading) > 1:
                directioncorrect = 1
            elif abs((goalheading - heading)/goalheading) <= 1:
                directioncorrect = abs((goalheading - heading)/goalheading)
        
            linearspeed = 5 + 35*(speedmult)*directioncorrect
        print("Proportional: ", Proportional)
        print("Derivative: ", Derivative)
        # Here's where the PID Values are actually injected into the motor speeds with a coefficient in front of each for adjustment
        print("Linearspeed ", linearspeed)
        print("Speed: ", linearspeed + (25*(Proportional)) + (25*(Derivative)))
        leftmotor.start(linearspeed + (25*(Proportional)) + (25*(Derivative)))
        rightmotor.start(-linearspeed + (25*(Proportional)) + (25*(Derivative)))
        time.sleep(0.002)
        # End movement when the destination is reached
        if abs(distanceleft) < 1:
             break
    return heading, prevangleoffset, Derivative, firsttry
heading, prevangleoffset, Derivative, firsttry = goto(firsttry, prevangleoffset, 50, 0)