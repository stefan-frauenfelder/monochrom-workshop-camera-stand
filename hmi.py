import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

import hardware
from hardware import wheel
from hardware import cam
from hardware import gpio
from hardware import io_expander

from fsm import mechanics_fsm

# Switches
ENTER_SW_GPIO = 17

SPARKFUN_BUTTON_GPIO = 21


class StateObserver(object):
    def __init__(self, name, owner):
        self.name = name
        self.owner = owner

    def on_enter_state(self, target, event):
        print(f'Mechanics FSM entering state {target.name} triggered by {event}')
        self.owner.update()


class View(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        # create an observer which can observe an FSM to observe the mechanics FSM
        mechanics_fsm_observer = StateObserver('views_mechanics_observer', self)
        # add the observer to the mechanics FSM, it will call View's update method on every state change
        mechanics_fsm.add_observer(mechanics_fsm_observer)

        self.initialize_button.clicked.connect(mechanics_fsm.e_initialize)

        self.homing_button.clicked.connect(mechanics_fsm.e_home)

        self.joystick_calibration_button.clicked.connect(self.quick_test)

        self.estop_button.clicked.connect(mechanics_fsm.e_eshutdown)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        mechanics_fsm.e_eshutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        while True:
            value = io_expander.gpio_digital_read(gpio=io_expander.eGPIO_TOTAL)
            print(value)
            time.sleep(0.5)

    def update(self):

        if mechanics_fsm.current_state.name == 'Initialized':
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)


class Controller:

    def __init__(self):
        self.setup_hid_callbacks()

    def rotary_callback(self, counter):
        print('Counter value: ', counter)

    def button_down_callback(self, _gpio, _level, _tick):
        mechanics_fsm.e_jog()

    def button_up_callback(self, _gpio, _level, _tick):
        mechanics_fsm.e_stop_jog()

    def setup_hid_callbacks(self):
        gpio.set_glitch_filter(SPARKFUN_BUTTON_GPIO, 1000)
        gpio.set_pull_up_down(SPARKFUN_BUTTON_GPIO, pigpio.PUD_UP)
        gpio.callback(SPARKFUN_BUTTON_GPIO, pigpio.FALLING_EDGE, self.button_down_callback)
        gpio.callback(SPARKFUN_BUTTON_GPIO, pigpio.RISING_EDGE, self.button_up_callback)

