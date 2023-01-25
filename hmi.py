import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from coordinator import *
from rotary import RotaryEncoder

# Jog wheel
JOG_WHEEL_A_GPIO = 23
JOG_WHEEL_B_GPIO = 24
# Switches
ENTER_SW_GPIO = 17

SPARKFUN_BUTTON_GPIO = 21

class View(QtWidgets.QMainWindow):

    def __init__(self, fsm):
        super().__init__()

        self.fsm = fsm

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(self.fsm.initialize)

        self.homing_button.clicked.connect(self.fsm.home)

    def closeEvent(self, event):
        print("Window close command issued, passing exit event to FSM.")
        self.fsm.exit()
        # close the window and exit the application
        event.accept()


class Controller:

    def __init__(self, fsm, coordinator):
        # let the controller know about the fsm
        self.fsm = fsm
        # hand the controller a reference to the coordinator
        self.coordinator = coordinator
        # connect the pigpio library to the pigpiod demon which needs to be already running on the raspberry pi
        self.gpios = pigpio.pi()
        # Rotary encoder jog wheel
        self.wheel = RotaryEncoder(gpios=self.gpios, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
        self.wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=self.rotary_callback)

        self.setup_hid_callbacks()

    def rotary_callback(self, counter):
        print('Counter value: ', counter)

    def button_down_callback(self, _gpio, _level, _tick):
        self.fsm.jog_arm()

    def button_up_callback(self, _gpio, _level, _tick):
        self.fsm.stop_jog()


    def setup_hid_callbacks(self):

        self.gpios.set_glitch_filter(SPARKFUN_BUTTON_GPIO, 1000)
        self.gpios.set_pull_up_down(SPARKFUN_BUTTON_GPIO, pigpio.PUD_UP)
        self.gpios.callback(SPARKFUN_BUTTON_GPIO, pigpio.FALLING_EDGE, self.button_down_callback)
        self.gpios.callback(SPARKFUN_BUTTON_GPIO, pigpio.RISING_EDGE, self.button_up_callback)
