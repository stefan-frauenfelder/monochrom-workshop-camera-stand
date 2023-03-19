import sys
import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets, uic

from hardware import hardware_manager
from hsm import hsm
from motion_control import motion_controller


class HmiParameter:
    def __init__(self, name, labels, target, value_label, value_range, value_increment, value_fraction_digits):
        self.name = name
        self.labels = labels
        self.target = target
        self.value_label = value_label
        self.value_range = value_range
        self.value_increment = value_increment
        self.value_fraction_digits = value_fraction_digits


class HmiParameterSet:
    def __init__(self):
        self.parameter = []
        self.selected_index = 0


class Hmi(QtWidgets.QMainWindow):

    # all parameters of the HMI are collected in a dictionary
    # its items are parameter sets
    parameter_space = {}

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(hsm.trig_initialize)

        self.homing_button.clicked.connect(hsm.trig_home)

        self.soft_key_C.clicked.connect(self.switch_parameter)

        # utility buttons
        self.joystick_calibration_button.clicked.connect(hardware_manager.joystick.calibrate)
        # self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(hsm.trig_emergency_shutdown)
        self.sys_exit_button.clicked.connect(sys.exit)

        # styling

        self.tab_bar.setStyleSheet(
            'QTabWidget::pane {border: 0px #000}'
            'QTabBar {background-color: #000}'
            'QTabBar::tab {height: 78px; width: 50px; color: #bbb; background-color: #222; border:2px solid #444; border-top-left-radius: 20px; border-bottom-left-radius: 20px}'
            'QTabBar::tab:selected {background-color: #000; border: 2px solid #fff; border-right: 0px #000}'
            'QTabBar::tab:!selected {color: #999; margin-left: 7px; width: 43px; border-right-color: #fff}')

        self.setStyleSheet(
            'QMainWindow {background-color: #000}'
            'QTabWidget {background-color: #000}'
            'QLabel {background-color: #000; color: #999}'
            'QPushButton {'
            'background-color: #000;'
            'color: #fff;'
            'border-style: outset;'
            'border-width: 2px;'
            'border-color: #ddd;'
            'border-radius: 10px;'
            '}'
            'QPushButton:pressed {border-style: inset; background-color: #222;}'
            'QPushButton:disabled {color: #666; border-color: #666}'
        )

        self.mode = 0

        # add a callbacks to the HSM to be notified about state changes
        hsm.before_any_state_change_callbacks.append(self.before_hsm_state_change)
        hsm.after_any_state_change_callbacks.append(self.after_hsm_state_change)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.joystick_button_callbacks.append(self.cb_joystick_button_change)
        hardware_manager.rgb_button_callbacks.append(self.cb_rgb_button_change)
        hardware_manager.wheel_callbacks.append(self.cb_wheel_counter_change)

        hardware_manager.soft_key_3_callbacks.append(self.switch_parameter)

        sequencer_parameter_set = HmiParameterSet()
        sequencer_parameter_set.parameter.append(HmiParameter(
            name='speed',
            labels=[self.speed_label, self.speed_value, self.speed_unit],
            target=motion_controller.user_speed,
            value_label=self.speed_value,
            value_range=(0.05, 1.0),
            value_increment=0.05,
            value_fraction_digits=2
        ))
        sequencer_parameter_set.parameter.append(HmiParameter(
            name='deflection',
            labels=[self.deflection_label, self.deflection_value, self.deflection_unit],
            target=motion_controller.front_linear_distance,
            value_label=self.deflection_value,
            value_range=(0.01, 1.0),
            value_increment=0.01,
            value_fraction_digits=2
        ))
        # add the set to the space
        self.parameter_space['sequencer_parameter'] = sequencer_parameter_set

        self.current_parameter_set: HmiParameterSet = self.parameter_space['sequencer_parameter']

        self.speed = 0.0

    def switch_parameter(self, level=1):
        if level:  # button is pressed
            num_param_in_set = len(self.current_parameter_set.parameter)
            previous_selected_index = self.current_parameter_set.selected_index
            self.current_parameter_set.selected_index = (previous_selected_index + 1) % num_param_in_set
            for i in range(num_param_in_set):
                hmi_parameter: HmiParameter = self.current_parameter_set.parameter[i]
                if i == self.current_parameter_set.selected_index:
                    # change labels of parameters to white
                    for label in hmi_parameter.labels:
                        label.setStyleSheet('QLabel{color: #fff}')
                    self.configure_wheel_for_parameter(hmi_parameter)
                elif i == previous_selected_index:
                    for label in hmi_parameter.labels:
                        label.setStyleSheet('QLabel{color: #999}')

    def configure_wheel_for_parameter(self, param):
        parameter: HmiParameter = param
        wheel = hardware_manager.wheel
        wheel.min = parameter.value_range[0]
        wheel.max = parameter.value_range[1]
        wheel.scale = parameter.value_increment
        wheel.counter = parameter.target

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        hsm.trig_emergency_shutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def cb_rotary_selector_switch(self, index):
        print('Controller: switched from mode ' + str(self.mode) +  ' to mode ' + str(index))
        if index == 0:
            hsm.trig_jog()
            self.tab_bar.setCurrentIndex(0)
        elif index == 1:
            hsm.joystick_variant = 'polar'
            hsm.trig_joystick()
            self.tab_bar.setCurrentIndex(1)
        elif index == 2:
            hsm.sequencer_variant = 'a_b'
            hsm.trig_sequencer()
            self.tab_bar.setCurrentIndex(2)
        elif index == 3:
            hsm.sequencer_variant = 'a_b'
            hsm.trig_sequencer()
            self.tab_bar.setCurrentIndex(3)
        elif index == 4:
            hsm.sequencer_variant = 'a_b'
            hsm.trig_sequencer()
            self.tab_bar.setCurrentIndex(4)
        # update
        self.mode = index

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

    def before_hsm_state_change(self, state_name):
        pass

    def after_hsm_state_change(self, state_name):
        # different behavior depending on current mode

        # jog mode
        if hsm.is_s_operational.s_jog_control():
            self.mode_label = 'Jogging'

        # joystick modes
        elif hsm.is_s_operational.s_joystick_control():
            if hsm.joystick_variant == 'polar':
                self.mode_label = 'Polar Joystick'
            elif hsm.joystick_variant == 'rectilinear':
                self.mode_label = 'Rect. Joystick'

        # sequencer modes
        elif hsm.is_s_operational.s_sequencer_control(allow_substates=True):
            if hsm.sequencer_variant == 'a_b':
                self.mode_label = 'A to B'
            elif hsm.sequencer_variant == 'front_linear':
                self.mode_label = 'Front Linear'
            elif hsm.sequencer_variant == 'orbiter':
                self.mode_label = 'Orbiter'

            if hsm.is_s_operational.s_sequencer_control.s_at_setup():
                # wheel is used to dial in the duration of the slide
                pass
        # other
        else:
            self.mode_label = 'Other'

        if hsm.is_s_initialized():
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)

    def cb_wheel_counter_change(self, counter):
        index = self.current_parameter_set.selected_index
        parameter: HmiParameter = self.current_parameter_set.parameter[index]
        fraction_digits = parameter.value_fraction_digits
        parameter.target = round(counter, fraction_digits)
        parameter.value_label.setText(f"{counter:.{fraction_digits}f}")
