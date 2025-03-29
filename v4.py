from simple_pid import PID
import qwiic_otos
import time
import math
from buildhat import Motor


pid = PID(0.4, 0.02, 0.2)
pidturn = PID(0.4, 0.02, 0.2)
pidtime = PID(0.2, 0, 0)
starttime = time.perf_counter()
goaltime = 59
currentpos = 0
checkpoints = [(0,25), (50, 25), (50, 175), 180, (50,75), (0,75), (-100, 125), (-150, 125), (-150, 25), 0, (-150, 175), (-2, 175)]
for i in range(len(checkpoints)):
    totaldistance = math.dist(checkpoints[i], checkpoints[i +1])

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

def goalcalc(goalx, goaly, heading):
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

    if abs(heading - goalheading) > 180:
        if heading > 0:
            goalheading = goalheading + 360
        if heading < 0:
            goalheading = goalheading - 360
    
    # Extra tests for if the goal heading should be over 180 or under -180

    #print("calculated xdisp: ", xdisp)
    #print("calculated ydisp: ", ydisp)
    #print("calculated goalheading: ", goalheading)
    return goalheading, xdisp, ydisp

def linearcontrol(goalx, goaly, goalheading, heading):
    myPosition = odo.getPosition()
    currentx = -myPosition.y*100
    currenty = myPosition.x*100
    print("Location: ", currentx, " ", currenty)
    displacement = math.dist((goalx, goaly), (currentx, currenty))
    
    # Slows linear speed when pointing the wrong way
    if abs(goalheading - heading) < 15:
        directioncorrect = 1
    elif abs(goalheading - heading) >= 15:
        directioncorrect = 1 / abs(goalheading - heading)
    print(goalheading)
    print(heading)
    print("abs goal - current heading: ", abs(goalheading - heading))
    print("Directioncorrect: ", directioncorrect)
    
    # Slows down on approach to goal point
    if displacement < 10:
        closeslowdown = (displacement / 20)
    elif displacement >= 10:
        closeslowdown = 1
    
    # Combine the two slowdown coefficients together
    linearspeed = 5 + 35*directioncorrect*closeslowdown
    return linearspeed, displacement

def rotate(absheading):
    while True:
        myPosition = odo.getPosition()
        heading = -myPosition.h
        goalheading = absheading
        # Compute new output from the PID according to the system's current value
        pidturn.setpoint = goalheading
        #print("linearspeed: ", linearspeed)
        # Slows down on approach to goal point
        if abs(goalheading - heading) < 30:
            closeslowdown = ((abs(goalheading - heading))/ 75)
        else:
            closeslowdown = 1
        controlright = (0.75 * pidturn(heading) * closeslowdown)
        controlleft = (0.75 * pidturn(heading) * closeslowdown)
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
        #print("Current:", heading, " degrees")
        #print("Goal: ", goalheading, " degrees")
        #print("Pid value: ", pid(heading))
        leftmotor.start(controlleft)
        rightmotor.start(controlright)
        #print("Displacement ", displacement)
        if abs(goalheading - heading) < 1:
            break
        time.sleep(0.002)

def timer(adddisp):
    global totaldistance, pidtime
    timeelapsed = (time.perf_counter() - starttime)
    timeleft = goaltime - timeelapsed
    currentgoalposition = (totaldistance / goaltime) * timeelapsed
    currentdisp = currentpos + adddisp
    pidtime.setpoint = currentgoalposition
    timemult = pidtime(currentdisp)
    if timemult > 1:
        timemult = 1
    elif timemult < 0:
        timemult = 0.2
    return timemult

def goto(goalx, goaly):
    wentneg = False
    wentpos = False
    while True:
        myPosition = odo.getPosition()
        preheading = -myPosition.h
        # Creates a relative heading that can be > 180 or < -180 for the PID loop to use in error calculations
        if wentneg == False and wentpos == False:
            if preheading < -178:
                wentneg = True
            elif preheading > 178:
                wentpos = True
            else:
                heading = preheading
        if wentneg == True:
            if preheading < 0:
                heading = preheading
            elif preheading > 0:
                heading = preheading - 360
        if wentpos == True:
            if preheading > 0:
                heading = preheading
            elif preheading < 0:
                heading = preheading + 360
        goalheading, xdisp, ydisp = goalcalc(goalx, goaly, heading)
        # Compute new output from the PID according to the system's current value
        global pid
        pid.setpoint = goalheading
        linearspeed, displacement = linearcontrol(goalx, goaly, goalheading, heading)
        #print("linearspeed: ", linearspeed)
        # Slows down on approach to goal point
        if displacement < 10:
            closeslowdownturn = (displacement / 8)
        elif displacement >= 10:
            closeslowdownturn = 1
        # Time management multiplier
        timemult = timer(math.dist((0,0), (goalx, goaly)) - (math.dist((0,0), (xdisp, ydisp))))
        controlright = timemult * (1 * pid(heading) * closeslowdownturn) - (1 * linearspeed)
        controlleft = timemult * (1 * pid(heading) * closeslowdownturn) + (1 * linearspeed)
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
        print("Current:", heading, " degrees")
        #print("Goal: ", goalheading, " degrees")
        #print("Pid value: ", pid(heading))
        leftmotor.start(controlleft)
        rightmotor.start(controlright)
        #print("Displacement ", displacement)
        if displacement < 2:
            # Add distance traveled for time function...
            global currentpos
            currentpos = currentpos + math.dist((0,0), (goalx, goaly))
            break
        time.sleep(0.002)
        
# Actually move
for i in range(len(checkpoints)):
    if type(i) == int:
        rotate(i)
    else:
        goto(i)

print("Time elapsed: ", time.perf_counter() - starttime, " seconds.")

    


