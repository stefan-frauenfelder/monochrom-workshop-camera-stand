import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from joystick import *
from coordinator import *
from rotary import RotaryEncoder
from zcame2 import *

# Jog wheel
JOG_WHEEL_A_GPIO = 23
JOG_WHEEL_B_GPIO = 24
# Switches
ENTER_SW_GPIO = 17

SPARKFUN_BUTTON_GPIO = 21

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V = 0x02  # 4.096V range = Gain 1

class View(QtWidgets.QMainWindow):

    def __init__(self, fsm):
        super().__init__()

        self.fsm = fsm

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(self.fsm.e_initialize)

        self.homing_button.clicked.connect(self.fsm.e_home)

        self.joystick_calibration_button.clicked.connect(self.fsm.e_calibrate_joystick)

        self.estop_button.clicked.connect(self.fsm.e_exit)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        self.fsm.e_exit()
        # close the window and e_exit the application
        event.accept()


class Controller:

    def __init__(self, fsm, coordinator, cam: ZCamE2):
        # let the controller know about the fsm
        self.fsm = fsm
        # hand the controller a reference to the coordinator
        self.coordinator = coordinator
        # connect the pigpio library to the pigpiod demon which needs to be already running on the raspberry gpio
        self.gpios = pigpio.pi()
        # Rotary encoder jog wheel
        self.wheel = RotaryEncoder(gpios=self.gpios, a_gpio=JOG_WHEEL_A_GPIO, b_gpio=JOG_WHEEL_B_GPIO)
        self.wheel.setup_rotary(min=0, max=1000, scale=1, debounce=0, rotary_callback=self.rotary_callback)

        self.setup_hid_callbacks()

        # create a joystick
        self.joystick = Joystick()

        self.cam = cam

    def rotary_callback(self, counter):
        print('Counter value: ', counter)

    def button_down_callback(self, _gpio, _level, _tick):
        self.fsm.e_jog()

    def button_up_callback(self, _gpio, _level, _tick):
        self.fsm.e_stop_jog()

    def setup_hid_callbacks(self):
        self.gpios.set_glitch_filter(SPARKFUN_BUTTON_GPIO, 1000)
        self.gpios.set_pull_up_down(SPARKFUN_BUTTON_GPIO, pigpio.PUD_UP)
        self.gpios.callback(SPARKFUN_BUTTON_GPIO, pigpio.FALLING_EDGE, self.button_down_callback)
        self.gpios.callback(SPARKFUN_BUTTON_GPIO, pigpio.RISING_EDGE, self.button_up_callback)

    def calibrate_joystick(self):

        self.cam.start_recording()
        time.sleep(10)
        self.cam.stop_recording()

