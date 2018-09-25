import os

bluepill_configs = {
    '01': {'steer_min': 3558,
           'steer_mid': 4732,
           'steer_max': 5743,
           'throttle_min': 2921,
           'throttle_mid': 4626,
           'throttle_max': 6327},
    '02': {'steer_min': 3579,
           'steer_mid': 4641,
           'steer_max': 5666,
           'throttle_min': 2791,
           'throttle_mid': 4594,
           'throttle_max': 6299},
    '03': {'steer_min': 3281,
           'steer_mid': 4421,
           'steer_max': 5395,
           'throttle_min': 2769,
           'throttle_mid': 4545,
           'throttle_max': 6305},
    '04': {},
    '05': {},
    '06': {},
    '07': {},
    '08': {},
    '09': {},
    '10': {'steer_min': 3186,
           'steer_mid': 4346,
           'steer_max': 5366,
           'throttle_min': 2722,
           'throttle_mid': 4552,
           'throttle_max': 6272},
}

def my_car():
    if 'MY_CAR' in os.environ:
        return os.environ['MY_CAR']
    else:
        print("Please set car ID by typing:\nexport MY_CAR=XX\n"
              "Where XX is the number on the receiver.")
        raise ValueError("Please export MY_CAR env var.")
