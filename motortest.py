from buildhat import Motor
import time

lmotor = Motor('A')
rmotor = Motor('B')

for i in range(0, 10, 1):
    lmotor.set_default_speed(i * 50)
    rmotor.set_default_speed(i * 50)
    lmotor.start()
    rmotor.start()
    time.sleep(10)

lmotor.stop()
rmotor.stop()

