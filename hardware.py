import time

# the purpose of this module is to instantiate hardware resources which do not depend on any higher level classes

import pigpio
import graycode
from smbus2 import SMBus
from bitstring import BitArray

from joystick import Joystick
from zcame2 import ZCamE2
from rotary import RotaryEncoder
from ch423 import Ch423
from sequent_ports import SequentPorts

# The digital pins of raspberry pi in BCM code

# Jog wheel
JOG_WHEEL_A_GPIO = 20
JOG_WHEEL_B_GPIO = 21

# connected to the INT pin (GPO15) of the CH423 I/O expander
IO_EXPANDER_INT_PIN = 26

# port extension cards
SEQUENT_INTERRUPT_GPIO = 5

# connect the pigpio library to the pigpiod demon which needs to be already running on the raspberry gpio
gpio = pigpio.pi()

# 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
sm_bus = SMBus(1)

def rotary_callback(counter):
    print('Counter value: ', counter)


# Rotary encoder jog wheel
wheel = RotaryEncoder(gpio, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=rotary_callback)

# create the camera interface instance to control the camera
cam = ZCamE2()

# create a joystick
joystick = Joystick()

# set up the hardware opto-isolated input and relay outputs cards
sequent_ports = SequentPorts(sm_bus, SEQUENT_INTERRUPT_GPIO)

# rotary selector mode
mode = 0

# create the I/O-expander
io_expander = Ch423(sm_bus)


def io_expander_general_callback(gpio_pin, _level, _tick):
    global mode
    # read the values from the I/O expander
    value = io_expander.gpio_digital_read(io_expander.eGPIO_TOTAL)
    # write values back to clear the interrupt (comparison of input state and output state)
    io_expander.gpio_digital_write(io_expander.eGPIO_TOTAL, value)
    # get the rotary selector bits
    inverse_graycode_mode = value & 0x0F

    bit_array_mode = BitArray(uint=inverse_graycode_mode, length=4)
    # bit-wise invert
    bit_array_mode.invert(range(4))
    # rotate in place
    bit_array_mode.reverse()
    # convert to graycode integer
    graycode_integer = bit_array_mode.uint
    # convert to two's complement integer
    mode = graycode.gray_code_to_tc(graycode_integer)
    # subtract 2, for whatever reason
    mode -= 2
    print('Hardware: mode switched to  ' + str(mode) + '.')


io_expander.enable_interrupt()

# wait for 10 milliseconds before reacting to the interrupt to debounce
gpio.set_glitch_filter(IO_EXPANDER_INT_PIN, 10000)
gpio.callback(IO_EXPANDER_INT_PIN, pigpio.FALLING_EDGE, io_expander_general_callback)
