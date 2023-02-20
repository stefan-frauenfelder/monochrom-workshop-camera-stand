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

        self.initialize_button.clicked.connect(hsm.trig_initialize)

        self.homing_button.clicked.connect(hsm.trig_home)

        # utility buttons
        # self.joystick_calibration_button.clicked.connect(self.quick_test)
        # self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(hsm.trig_emergency_shutdown)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        hsm.trig_emergency_shutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def update(self):
        if hsm.is_s_initialized():
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)


class Controller:

    def __init__(self):

        self.mode = 0

        self.display = hardware_manager.display

        # add a callback to the HSM to be notified about state changes
        hsm.state_changed_callbacks.append(self.on_hsm_state_change)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.joystick_button_callbacks.append(self.cb_joystick_button_change)
        hardware_manager.rgb_button_callbacks.append(self.cb_rgb_button_change)

    def cb_rotary_selector_switch(self, new_mode):
        print('Controller: switched from mode ' + str(self.mode) +  ' to mode ' + str(new_mode))
        if new_mode == 0:
            hsm.trig_jog()
        elif new_mode == 1:
            hsm.trig_polar_joystick()
        elif new_mode == 2:
            hsm.trig_rectilinear_joystick()
        elif new_mode == 3:
            hsm.trig_mirror_slider()
        elif new_mode == 4:
            hsm.trig_sequencer()
        # update
        self.mode = new_mode

    def cb_joystick_button_change(self, value):
        # only if button is pressed
        if value:
            # different behavior depending on current mode

            # polar joystick mode
            if hsm.is_s_operational.s_polar_joystick_control():
                motion_controller.toggle_polar_joystick_axes_set()
                print('Toggled joystick axes.')

    def cb_rgb_button_change(self, value):
        # only if button is pressed
        if value:
            # different behavior depending on current mode

            # jog mode
            if hsm.is_s_operational.s_jog_control():
                motion_controller.toggle_jog_axis()
                print('Toggled jog axis.')

            # polar joystick mode
            elif hsm.is_s_operational.s_polar_joystick_control():
                print('Setting marker.')
                marker = motion_controller.get_marker()
                print(marker)
                motion_controller.target_marker = marker

            # mirror slider mode
            elif hsm.is_s_operational.s_mirror_slider():
                motion_controller.front_linear_mirror(hardware_manager.wheel.counter)

            # sequencer mode
            elif hsm.is_s_operational.s_sequencer_control():
                motion_controller.synchronized_move_from_here_to_target(10)

    def on_hsm_state_change(self):
        # different behavior depending on current mode

        # jog mode
        if hsm.is_s_operational.s_jog_control():
            self.display.clear()
            self.display.mode('Jogging')
            self.display.show()

        # polar joystick mode
        elif hsm.is_s_operational.s_polar_joystick_control():
            self.display.clear()
            self.display.mode('Polar Joystick')
            self.display.show()

        # rectilinear joystick mode
        elif hsm.is_s_operational.s_rectilinear_joystick_control():
            self.display.clear()
            self.display.mode('Rect. Joystick')
            self.display.show()

        # mirror slider mode
        elif hsm.is_s_operational.s_mirror_slider():
            self.display.clear()
            self.display.mode('Mirror Slider')
            self.display.show()
            # wheel is used to dial in the duration of the slide
            wheel = hardware_manager.wheel
            wheel.counter = 10
            wheel.max = 30
            wheel.min = 5

        # sequencer mode
        elif hsm.is_s_operational.s_sequencer_control():
            self.display.clear()
            self.display.mode('Sequencer')
            self.display.show()

        # other
        else:
            self.display.clear()
            self.display.mode('Other')
            self.display.show()