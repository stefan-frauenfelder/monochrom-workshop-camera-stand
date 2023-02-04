import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from hardware import wheel
from hardware import cam
from hardware import gpio

from fsm import fsm


# Switches
ENTER_SW_GPIO = 17

SPARKFUN_BUTTON_GPIO = 21

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V = 0x02  # 4.096V range = Gain 1


class View(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(fsm.e_initialize)

        self.homing_button.clicked.connect(fsm.e_home)

        self.joystick_calibration_button.clicked.connect(fsm.e_calibrate_joystick)

        self.estop_button.clicked.connect(fsm.e_exit)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        fsm.e_exit()
        # close the window and e_exit the application
        event.accept()


class Controller:

    def __init__(self):

        self.setup_hid_callbacks()

    def rotary_callback(self, counter):
        print('Counter value: ', counter)

    def button_down_callback(self, _gpio, _level, _tick):
        fsm.e_jog()

    def button_up_callback(self, _gpio, _level, _tick):
        fsm.e_stop_jog()

    def setup_hid_callbacks(self):
        gpio.set_glitch_filter(SPARKFUN_BUTTON_GPIO, 1000)
        gpio.set_pull_up_down(SPARKFUN_BUTTON_GPIO, pigpio.PUD_UP)
        gpio.callback(SPARKFUN_BUTTON_GPIO, pigpio.FALLING_EDGE, self.button_down_callback)
        gpio.callback(SPARKFUN_BUTTON_GPIO, pigpio.RISING_EDGE, self.button_up_callback)

    def calibrate_joystick(self):

        cam.start_recording()
        time.sleep(10)
        cam.stop_recording()


# create the controller which handles manual user inputs
controller = Controller()
