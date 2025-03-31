from simple_pid import PID
import qwiic_otos
import time
import math
from buildhat import Motor
import RPi.GPIO as GPIO
buttonpin = 37
GPIO.setmode(GPIO.BOARD) # Numbers GPIOs by physical location
GPIO.setup(buttonpin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
leftmotor = Motor('A')
rightmotor = Motor('B')
leftmotor.run_for_rotations(0.15, speed = 50, blocking = False)
rightmotor.run_for_rotations(-0.15, speed = 50)
odo = None
lastrun = False
# Initialize everything:
def initialization():
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

    return odo

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
    #print("Location: ", currentx, " ", currenty)
    displacement = math.dist((goalx, goaly), (currentx, currenty))
    
    # Slows linear speed when pointing the wrong way
    if abs(goalheading - heading) < 25:
        directioncorrect = 1
    elif abs(goalheading - heading) >= 25:
        directioncorrect = 1 / abs(goalheading - heading)
    #print(goalheading)
    #print(heading)
    #print("abs goal - current heading: ", abs(goalheading - heading))
    #print("Directioncorrect: ", directioncorrect)
    
    # Slows down on approach to goal point
    if displacement < 10:
        closeslowdown = (displacement / 15)
    elif displacement >= 10:
        closeslowdown = 1
    
    # Combine the two slowdown coefficients together
    linearspeed = 5 + 35*directioncorrect*closeslowdown
    return linearspeed, displacement

def rotate(absheading):
    while True:
        myPosition = odo.getPosition()
        heading = -myPosition.h
        if absheading > 0:
            goalheading = absheading -40
        elif absheading < 0:
            goalheading = absheading +40
        elif absheading == 0:
            goalheading = -40
        # Compute new output from the PID according to the system's current value
        pidturn.setpoint = goalheading
        #print("linearspeed: ", linearspeed)
        # Slows down on approach to goal point
        if abs(goalheading - heading) < 30:
            closeslowdown = ((abs(goalheading - heading))/ 75)
        else:
            closeslowdown = 1
        controlright = (0.3 * pidturn(heading) * closeslowdown)
        controlleft = (0.3 * pidturn(heading) * closeslowdown)
        if controlleft > 100:
            controlleft = 100    
        if controlleft < -100:
            controlleft = -100
        if controlright > 100:
            controlright = 100
        if controlright < -100:
            controlright = -100
        #print("Current:", heading, " degrees")
        #print("Goal: ", goalheading, " degrees")
        #print("Pid value: ", pid(heading))
        leftmotor.start(controlleft)
        rightmotor.start(controlright)
        #print("Displacement ", displacement)
        if abs(goalheading - heading) < 3:
            break
        time.sleep(0.002)

def goto(goalx, goaly):
    wentneg = False
    wentpos = False
    rightmotor.start(0)
    leftmotor.start(0)
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
        pid.setpoint = goalheading
        linearspeed, displacement = linearcontrol(goalx, goaly, goalheading, heading)
        #print("linearspeed: ", linearspeed)
        # Slows down on approach to goal point
        if displacement < 10:
            closeslowdownturn = (displacement / 8)
        elif displacement >= 10:
            closeslowdownturn = 1
        if lastrun == True:
            endspeed = 0.3
        else:
            endspeed = 1
        controlright = (1.5 * pid(heading) * closeslowdownturn) - endspeed * (1.25 * linearspeed)
        controlleft = (1.5 * pid(heading) * closeslowdownturn) + endspeed * (1.25 * linearspeed)
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
        if displacement < 15 and lastrun == False:
            break
        elif displacement < 0.5 and lastrun == True:
            break

        time.sleep(0.002)

pid = PID(0.6, 0.02, 0.35)
pidturn = PID(0.4, 0.02, 0.2)

def instruct(ev=None):
    global odo, lastrun
    odo = initialization()
    time.sleep(1)
    starttime = time.perf_counter()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    goto()
    lastrun = True
    goto(
    print("Time elapsed: ", time.perf_counter() - starttime, " seconds.")
    rightmotor.stop()
    leftmotor.stop()
    lastrun = False

GPIO.add_event_detect(buttonpin, GPIO.FALLING, callback=instruct, bouncetime=200)

while True:
    time.sleep(1)