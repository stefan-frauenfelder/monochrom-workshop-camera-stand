import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from hardware import hardware_manager
from hsm import hsm
from motion_control import motion_controller


class View(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        # add a callback to the HSN to be notified about state changes
        hsm.state_changed_callbacks.append(self.update)

        self.initialize_button.clicked.connect(hsm.initialize)

        self.homing_button.clicked.connect(hsm.home)

        # utility buttons
        self.joystick_calibration_button.clicked.connect(self.quick_test)
        self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(hsm.emergency_shutdown)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        hsm.emergency_shutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def update(self):
        if hsm.is_initialized():
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)


class Controller:

    def __init__(self):

        self.mode = 0

        # add a callback to the HSN to be notified about state changes
        hsm.state_changed_callbacks.append(self.update())

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.rgb_button_callbacks.append(self.cb_joystick_button_change)

    def cb_rotary_selector_switch(self, new_mode):
        print('Controller: switched from mode ' + str(self.mode) +  ' to mode ' + str(new_mode))
        if new_mode == 0:
            hsm.e_joystick_control()
        elif self.mode == 0:
            hsm.e_idle()
        # update
        self.mode = new_mode

    def cb_joystick_button_change(self, value):
        # only if button is pressed
        if value:
            # different behavior depending on current mode
            if self.mode == 0:  # joystick
                motion_controller.toggle_joystick_axes_set()
                print('Toggled joystick axes.')

    def update(self):
        pass
