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
        self.joystick_calibration_button.clicked.connect(hardware_manager.joystick.calibrate)
        # self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(hsm.trig_emergency_shutdown)

        self.update()

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

        self.speed_display.text = str(motion_controller.user_speed) + ' m/s'


class Controller:

    def __init__(self):

        self.mode = 0
        self.mode_label = 'Uninitialized'

        self.display = hardware_manager.display

        # add a callback to the HSM to be notified about state changes
        hsm.state_changed_callbacks.append(self.on_hsm_state_change)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.joystick_button_callbacks.append(self.cb_joystick_button_change)
        hardware_manager.rgb_button_callbacks.append(self.cb_rgb_button_change)
        hardware_manager.wheel_callbacks.append(self.cb_wheel_counter_change)

        self.speed = 0.0

    def cb_rotary_selector_switch(self, new_mode):
        print('Controller: switched from mode ' + str(self.mode) +  ' to mode ' + str(new_mode))
        if new_mode == 0:
            hsm.trig_jog()
        elif new_mode == 1:
            hsm.joystick_variant = 'polar'
            hsm.trig_joystick()
        elif new_mode == 2:
            hsm.joystick_variant = 'rectilinear'
            hsm.trig_joystick()
        elif new_mode == 3:
            hsm.sequencer_variant = 'a_b'
            hsm.trig_sequencer()
        elif new_mode == 4:
            hsm.sequencer_variant = 'front_linear'
            hsm.trig_sequencer()
        elif new_mode == 5:
            hsm.sequencer_variant = 'orbiter'
            hsm.trig_sequencer()
        # update
        self.mode = new_mode

    def cb_joystick_button_change(self, value):
        # only if button is pressed
        if value:
            # different behavior depending on current mode

            # polar joystick mode
            if hsm.is_s_operational.s_joystick_control():
                if hsm.joystick_variant == 'polar':
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
            elif hsm.is_s_operational.s_joystick_control():
                print('Setting marker.')
                marker = motion_controller.get_marker()
                print(marker)
                motion_controller.target_marker = marker

            # sequencer mode
            elif hsm.is_s_operational.s_sequencer_control(allow_substates=True):
                hsm.trig_proceed()

    def on_hsm_state_change(self):
        # different behavior depending on current mode

        # jog mode
        if hsm.is_s_operational.s_jog_control():
            self.mode_label = 'Jogging'
            self.update_display()

        # joystick modes
        elif hsm.is_s_operational.s_joystick_control():
            if hsm.joystick_variant == 'polar':
                self.mode_label = 'Polar Joystick'
            elif hsm.joystick_variant == 'rectilinear':
                self.mode_label = 'Rect. Joystick'
            self.update_display()

        # sequencer modes
        elif hsm.is_s_operational.s_sequencer_control(allow_substates=True):
            if hsm.sequencer_variant == 'a_b':
                self.mode_label = 'A to B'
            elif hsm.sequencer_variant == 'front_linear':
                self.mode_label = 'Front Linear'
            elif hsm.sequencer_variant == 'orbiter':
                self.mode_label = 'Orbiter'
            self.update_display()

            if hsm.is_s_operational.s_sequencer_control.s_at_setup():
                # wheel is used to dial in the duration of the slide
                wheel = hardware_manager.wheel
                wheel.scale = 0.01
                wheel.min = 0.01
                wheel.counter = 0.1
                wheel.max = 0.3


        # other
        else:
            self.mode_label = 'Other'
            self.update_display()

    def update_display(self):
        self.display.clear()
        self.display.mode(self.mode_label)
        self.display.show()

    def cb_wheel_counter_change(self, counter):
        motion_controller.user_speed = counter
        print('Speed is now: ' + str(counter))
