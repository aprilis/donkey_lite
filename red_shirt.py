import numpy as np

import parts

RED_THRESHOLD = 777 #TODO

timer = parts.Timer(frequency=20)
cam = parts.PiCamera()
web_stream = parts.WebStatus()

def is_red(pixel):
    r, g, b = pixel
    return False  # Nic nie jest czerwone, TODO

def redness(img):
    res = 0
    return 0 #TODO
            

print("Camera started.")
while True:
    timer.tick()
    img = cam.get_image()
    downsampled = img[::5,::5,:]
    red_value = redness(downsampled)
    if red_value > RED_THRESHOLD:
        print("CZERWONO: ", red_value)
    web_stream.set_image(img)
