import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from ads1115 import ADS1115

from coordinator import *
from rotary import RotaryEncoder

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

        self.initialize_button.clicked.connect(self.fsm.initialize)

        self.homing_button.clicked.connect(self.fsm.home)

        self.joystick_calibration_button.clicked.connect(self.fsm.calibrate_joystick)

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
        # create an instance of the ADS1115 ADC
        self.adc = ADS1115()


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

    def calibrate_joystick(self, flag):

        x_mid_initial = self.adc.read_voltage(0)
        time.sleep(0.1)
        y_mid_initial = self.adc.read_voltage(1)
        time.sleep(0.1)

        x_min = x_mid_initial
        x_max = x_mid_initial
        y_min = y_mid_initial
        y_max = y_mid_initial

        while flag.is_set():

            x = self.adc.read_voltage(0)
            time.sleep(0.1)
            y = self.adc.read_voltage(1)
            time.sleep(0.1)

            x_min = min(x_min, x)
            x_max = max(x_max, x)

            y_min = min(y_min, y)
            y_max = max(y_max, y)

            print('X: Min:' + str(x_min) + ' Mid:' + str(x_mid_initial) + ' Max:' + str(x_max))
            print('Y: Min:' + str(y_min) + ' Mid:' + str(y_mid_initial) + ' Max:' + str(y_max))




