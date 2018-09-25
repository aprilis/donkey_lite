import datetime
import time

import car_config
import parts


my_car = car_config.my_car()
bluepill = parts.BluePill(**car_config.bluepill_configs[my_car])
keras_pilot = parts.KerasPilot('model')
timer = parts.Timer(frequency=20)
cam = parts.PiCamera()

try:
    while True:
        timer.tick()
        car_status = bluepill.get_status()
        im = cam.get_image() / 255.0
        ang, thr = keras_pilot.get_steering(im)
        bluepill.drive(ang, car_status.user_throttle)
        print(ang, car_status.user_throttle)
finally:
    bluepill.stop_and_disengage_autonomy()
