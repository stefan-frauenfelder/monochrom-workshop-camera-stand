
# the purpose of this module is to instantiate hardware resources which do not depend on any higher level classes

import pigpio

from joystick import Joystick
from zcame2 import ZCamE2
from rotary import RotaryEncoder

# Jog wheel
JOG_WHEEL_A_GPIO = 23
JOG_WHEEL_B_GPIO = 24

# connect the pigpio library to the pigpiod demon which needs to be already running on the raspberry gpio
gpio = pigpio.pi()


def rotary_callback(counter):
    print('Counter value: ', counter)


# Rotary encoder jog wheel
wheel = RotaryEncoder(gpio, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=rotary_callback)

# create the camera interface instance to control the camera
cam = ZCamE2()

# create a joystick
joystick = Joystick()
