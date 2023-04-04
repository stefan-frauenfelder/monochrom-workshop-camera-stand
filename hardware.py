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

# The digital pins of raspberry pi in BCM code

# Jog wheel
JOG_WHEEL_A_GPIO = 20
JOG_WHEEL_B_GPIO = 21
# connected to the INT pin (GPO15) of the CH423 I/O expander
IO_EXPANDER_INT_PIN = 26
# port extension cards
SEQUENT_INTERRUPT_GPIO = 5
# joystick pushbutton
JOYSTICK_BUTTON_GPIO = 6
# RGB 0 pushbutton
RGB_0_BUTTON_GPIO = 24


class HardwareManager:

    def __init__(self):
        # connect the pigpio library to the pigpiod demon which needs to be already running on the raspberry pi
        self.gpio = pigpio.pi()

        # Rotary encoder jog wheel
        self.wheel = RotaryEncoder(self.gpio, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
        self.wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=self.rotary_encoder_callback)
        self.wheel_callbacks = []

        # create the camera interface instance to control the camera
        self.cam = ZCamE2()

        # create a joystick
        self.joystick = Joystick()
        self.joystick_button_callbacks = [None]

        # set up the hardware opto-isolated input and relay outputs cards
        self.sequent_ports = SequentPorts(SEQUENT_INTERRUPT_GPIO)

        # create the RGB button instance (on I2C)
        self.rgb_0_button_light = RgbButton()
        self.rgb_0_button_light.begin()
        self.rgb_0_button_light.set_rgb_color(RgbButton.e_blue)

        self.rgb_0_button_callbacks = [None]

        # create the I/O-expander
        self.io_expander = Ch423()
        # maintain a state of the I/O expander pin levels for change comparison
        self.io_expander_values = 0x00

        self.io_expander.enable_interrupt()

        # wait for 10 milliseconds before reacting to the interrupt to debounce
        self.gpio.set_glitch_filter(IO_EXPANDER_INT_PIN, 10000)
        self.gpio.callback(IO_EXPANDER_INT_PIN, pigpio.FALLING_EDGE, self.io_expander_general_callback)

        # joystick button
        self.gpio.set_glitch_filter(JOYSTICK_BUTTON_GPIO, 10000)
        self.gpio.set_pull_up_down(JOYSTICK_BUTTON_GPIO, pigpio.PUD_UP)
        self.gpio.callback(JOYSTICK_BUTTON_GPIO, pigpio.EITHER_EDGE, self.cb_joystick_button)

        # RGB 0 button
        self.gpio.set_glitch_filter(RGB_0_BUTTON_GPIO, 10000)
        self.gpio.set_pull_up_down(RGB_0_BUTTON_GPIO, pigpio.PUD_DOWN)
        self.gpio.callback(RGB_0_BUTTON_GPIO, pigpio.EITHER_EDGE, self.cb_rgb_0_button)

        # maintain a state of the rotary selector value for external access
        self.rotary_selector_value = 0
        self.rotary_selector_callbacks = []

        self.soft_key_1_callbacks = [None]  # Only one, mutable
        self.soft_key_2_callbacks = [None]  # Only one, mutable
        self.soft_key_3_callbacks = [None]  # Only one, mutable

        # call the general update function once at startup
        self.io_expander_general_callback(0, 0, 0)

    def io_expander_general_callback(self, gpio_pin, _level, _tick):
        # read the values from the I/O expander
        new_values = self.io_expander.gpio_read()
        # write values back to clear the interrupt (comparison of input state and output state)
        self.io_expander.gpio_write(new_values)
        # check what changed using bitwise xor
        changed_bits = self.io_expander_values ^ new_values
        # update the state
        self.io_expander_values = new_values
        # call the updaters
        self.update_rotary_selector_value(values=new_values, changed_bits=changed_bits)
        self.update_soft_key_1(values=new_values, changed_bits=changed_bits)
        self.update_soft_key_2(values=new_values, changed_bits=changed_bits)
        self.update_soft_key_3(values=new_values, changed_bits=changed_bits)

    def update_rotary_selector_value(self, values, changed_bits):
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
            self.cb_rotary_selector_switched(value=new_value)

    def update_soft_key_1(self, values, changed_bits):
        mask = 0x10
        # only update if the button bit changed
        if changed_bits & mask:
            self.cb_soft_key_1(bool(values & mask))  # one means pressed

    def update_soft_key_2(self, values, changed_bits):
        mask = 0x20
        # only update if the button bit changed
        if changed_bits & mask:
            self.cb_soft_key_2(bool(values & mask))  # one means pressed

    def update_soft_key_3(self, values, changed_bits):
        mask = 0x40
        # only update if the button bit changed
        if changed_bits & mask:
            self.cb_soft_key_3(bool(values & mask))  # one means pressed

    def cb_rotary_selector_switched(self, value):
        print('Hardware: rotary selector is switched to  ' + str(value) + '.')
        for callback in self.rotary_selector_callbacks:
            callback(value)

    def cb_joystick_button(self, gpio_pin, _level, _tick):
        if not _level:
            print('Hardware: joystick button pressed.')
            for callback in self.joystick_button_callbacks:
                if callback:
                    callback()
        else:
            print('Hardware: joystick button released.')


    def cb_rgb_0_button(self, gpio_pin, _level, _tick):
        if _level:
            print('Hardware: RGB 0 button pressed.')
            for callback in self.rgb_0_button_callbacks:
                if callback:
                    callback()
        else:
            print('Hardware: RGB 0 button released.')


    def cb_soft_key_1(self, _level):
        if _level:
            print('Hardware: Soft key 1 pressed.')
            if self.soft_key_1_callbacks:
                for callback in self.soft_key_1_callbacks:
                    if callback:
                        callback()
        else:
            print('Hardware: Soft key 1 released.')

    def cb_soft_key_2(self, _level):
        if _level:
            print('Hardware: Soft key 2 pressed.')
            if self.soft_key_2_callbacks:
                for callback in self.soft_key_2_callbacks:
                    if callback:
                        callback()
        else:
            print('Hardware: Soft key 2 released.')

    def cb_soft_key_3(self, _level):
        if _level:
            print('Hardware: Soft key 3 pressed.')
            if self.soft_key_3_callbacks:
                for callback in self.soft_key_3_callbacks:
                    if callback:
                        callback()
        else:
            print('Hardware: Soft key 3 released.')

    def rotary_encoder_callback(self, counter):
        print('Hardware: Wheel value changed to: ', counter)
        for callback in self.wheel_callbacks:
            callback(counter)


# create an instance
hardware_manager = HardwareManager()
