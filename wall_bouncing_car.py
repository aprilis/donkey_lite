import datetime
import time

import car_config
import parts

my_car = car_config.my_car()
bluepill = parts.BluePill(**car_config.bluepill_configs[my_car])

timer = parts.Timer(frequency=20)

DIST_THRESHOLD = 750
FORWARD_SPEED = 0.15
BACKWARD_SPEED = -0.3
back_counter = 0

try:
    while True:
        timer.tick()
        car_status = bluepill.get_status()
        distance = car_status.distance
        print(distance)

        if back_counter > 0:
            print("break ticks left: ", back_counter)
            bluepill.drive(0.75, BACKWARD_SPEED)
            back_counter -= 1
            #bluepill.drive(0.0,0.0)
        else:
            if distance > DIST_THRESHOLD:
                bluepill.drive(0, FORWARD_SPEED)
            else:
                print('breaking starting')
                bluepill.drive(0, -1)
                time.sleep(0.1)
                bluepill.drive(0, 0.0)
                time.sleep(0.1)
                bluepill.drive(0, BACKWARD_SPEED)
                back_counter = 15
        
finally:
    bluepill.stop_and_disengage_autonomy()
