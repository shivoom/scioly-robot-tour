from simple_pid import PID
import time

pid = PID(0.8, 0.1, 0.05)
initial = 0
while True:
    # Compute new output from the PID according to the systems current value
    pid.setpoint = 90
    control = pid(initial)

    # Feed the PID output to the system and get its current value
    print("I'm at:", initial)
    print("pid go: ", control)
    initial = initial + control
    time.sleep(0.5)