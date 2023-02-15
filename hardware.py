import time

# the purpose of this module is to instantiate hardware resources which do not depend on any higher level classes

import pigpio
import graycode
from bitstring import BitArray

from sequent_ports import SequentPorts
from joystick import Joystick
from zcame2 import ZCamE2
from rotary import RotaryEncoder
from ch423 import Ch423
from rgbbutton import RgbButton
from display import Display

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


def rotary_callback(counter):
    print('Counter value: ', counter)


# Rotary encoder jog wheel
wheel = RotaryEncoder(gpio, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=rotary_callback)

# create the camera interface instance to control the camera
cam = ZCamE2()

# create a joystick
joystick = Joystick()

display = Display()
display.demo()

# set up the hardware opto-isolated input and relay outputs cards
sequent_ports = SequentPorts(SEQUENT_INTERRUPT_GPIO)


# create the RGB button instance
rgb_button_light = RgbButton()
rgb_button_light.begin()
rgb_button_light.set_rgb_color(RgbButton.e_blue)

# create the I/O-expander
io_expander = Ch423()
# maintain a state of the I/O expander pin levels for change comparison
io_expander_values = 0x00
# maintain a state of the rotary selector value for external access
rotary_selector_value = 0


def io_expander_general_callback(gpio_pin, _level, _tick):
    global io_expander_values
    # read the values from the I/O expander
    new_values = io_expander.gpio_read()
    # write values back to clear the interrupt (comparison of input state and output state)
    io_expander.gpio_write(new_values)
    # check what changed using bitwise xor
    changed_bits = io_expander_values ^ new_values
    # update the state
    io_expander_values = new_values
    # call the updaters
    update_rotary_selector_value(values=new_values, changed_bits=changed_bits)
    update_joystick_button(values=new_values, changed_bits=changed_bits)
    update_rgb_button(values=new_values, changed_bits=changed_bits)


def update_rotary_selector_value(values, changed_bits):
    # only update if one of the rotary values changed
    if changed_bits & 0x0F:
        inverse_graycode_bits = values & 0x0F
        bit_array_values = BitArray(uint=inverse_graycode_bits, length=4)
        # bit-wise invert
        bit_array_values.invert(range(4))
        # rotate in place
        bit_array_values.reverse()
        # convert to graycode integer
        graycode_integer = bit_array_values.uint
        # convert to two's complement integer
        new_value = graycode.gray_code_to_tc(graycode_integer)
        # subtract 2, for whatever reason
        new_value -= 2
        cb_rotary_selector_switched(value=new_value)


def cb_rotary_selector_switched(value):
    print('Hardware: rotary selector is switched to  ' + str(value) + '.')


def update_joystick_button(values, changed_bits):
    # only update if the joystick button bit changed
    if changed_bits & 0x10:
        joystick_button_up = values & 0x10  # zero means pressed
        if joystick_button_up:
            cb_joystick_button_released()
        else:
            cb_joystick_button_pressed()


def update_rgb_button(values, changed_bits):
    # only update if the rgb button bit changed
    if changed_bits & 0x20:
        button_down = values & 0x20  # one means pressed
        if button_down:
            cb_rgb_button_pressed()
        else:
            cb_rgb_button_released()


def cb_joystick_button_pressed():
    print('Hardware: Joystick button is pressed.')


def cb_rgb_button_pressed():
    print('Hardware: RGB button is pressed.')


def cb_joystick_button_released():
    print('Hardware: Joystick button is released.')


def cb_rgb_button_released():
    print('Hardware: RGB button is released.')


io_expander.enable_interrupt()

# wait for 10 milliseconds before reacting to the interrupt to debounce
gpio.set_glitch_filter(IO_EXPANDER_INT_PIN, 10000)
gpio.callback(IO_EXPANDER_INT_PIN, pigpio.FALLING_EDGE, io_expander_general_callback)

# call the general update function once at startup
io_expander_general_callback(0, 0, 0)
