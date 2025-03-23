import qwiic_otos
import math
from buildhat import Motor
from simple_pid import PID

my_odo = qwiic_otos.QwiicOTOS() #easier to access the QwiicOTOS() function
my_position = my_odo.getPosition() #sets position to a variable

left_motor = Motor('A')
right_motor = Motor('B')

#defining position
x = my_position.x
y = my_position.y
target_x = 1
target_y = 1


pid_x = PID(1.0, 0.0, 0.05, setpoint = 0)
pid_y = PID(1.0, 0,0, 0.05, setpoint = 0)

while True:
    #calculating error for x and y
    error_x = target_x - x
    error_y = target_y - y
    
    #update pid controller with error values
    speed_x = pid_x(error_x)
    speed_y = pid_y(error_y)

    left_motor





