import sys
import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets, uic

from hardware import hardware_manager
from hsm import hsm
from motion_control import motion_controller
import mainwindow


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


class Tab:

    tab_bar: QtWidgets.QTabWidget = None

    def __init__(self,
                 display_name,
                 short_display_name,
                 slot=None,
                 parameter_set=None,
                 hsm_trigger=None,
                 soft_key_a_callback=None,
                 soft_key_b_callback=None,
                 soft_key_c_callback=None,
                 rgb_0_button_callback=None,
                 joystick_button_callback=None,
                 init_function=None):

        self.display_name = display_name
        self.short_display_name = short_display_name
        self.slot = slot  # range: 0...4
        self.parameter_set: HmiParameterSet = parameter_set
        self.hsm_trigger = hsm_trigger
        self.soft_key_a_callback = soft_key_a_callback
        self.soft_key_b_callback = soft_key_b_callback
        self.soft_key_c_callback = soft_key_c_callback
        self.rgb_0_button_callback = rgb_0_button_callback
        self.joystick_button_callback = joystick_button_callback
        self.init_function = init_function
        self._active = False

    def activate(self):
        # can only activate if self has a slot
        if self.is_enabled:
            if self.hsm_trigger:
                # call the trigger function which puts the HSM in the required state
                self.hsm_trigger()
            hardware_manager.soft_key_1_callbacks[0] = self.soft_key_a_callback
            hardware_manager.soft_key_2_callbacks[0] = self.soft_key_b_callback
            hardware_manager.soft_key_3_callbacks[0] = self.soft_key_c_callback
            hardware_manager.rgb_0_button_callbacks[0] = self.rgb_0_button_callback
            hardware_manager.joystick_button_callbacks[0] = self.joystick_button_callback
            if self.init_function:
                self.init_function()
            self._active = True
            # tell the (parent) QT tab bar to make this tab visible/active
            self.tab_bar.setCurrentIndex(self.slot)

    def deactivate(self):
        hardware_manager.soft_key_1_callbacks[0] = None
        hardware_manager.soft_key_2_callbacks[0] = None
        hardware_manager.soft_key_3_callbacks[0] = None
        hardware_manager.rgb_0_button_callbacks[0] = None
        hardware_manager.joystick_button_callbacks[0] = None
        self._active = False

    @property
    def is_active(self):
        return self._active

    @property
    def is_enabled(self):
        return self.slot is not None

    def disable(self):
        self.slot = None


class Hmi(QtWidgets.QMainWindow, mainwindow.Ui_MainWindow):

    # all parameters of the HMI are collected in a dictionary
    # its items are parameter sets
    parameter_space = {}

    def __init__(self):
        super().__init__()

        # this method is inherited from mainwindow.Ui_MainWindow and loads all the UI elements
        self.setupUi(self)

        self.setWindowTitle("Camera Motion Control")

        self.initialize_button.clicked.connect(hsm.trig_initialize)

        self.homing_button.clicked.connect(hsm.trig_home)

        self.soft_key_C_sequence_tab.clicked.connect(self.switch_parameter)
        self.soft_key_C_joystick_tab.clicked.connect(motion_controller.reset_rotor_reference)

        # utility buttons
        self.joystick_calibration_button.clicked.connect(hardware_manager.joystick.calibrate)
        # self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(hsm.trig_emergency_shutdown)
        self.sys_exit_button.clicked.connect(sys.exit)

        # Styling of graphical user interface

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

        self.tabs = []

        # Callback registration for static callbacks

        # add a callbacks to the HSM to be notified about state changes
        hsm.before_any_state_change_callbacks.append(self.before_hsm_state_change)
        hsm.after_any_state_change_callbacks.append(self.after_hsm_state_change)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.wheel_callbacks.append(self.cb_wheel_counter_change)

        # Parameterization

        speed_parameter = HmiParameter(
            name='speed',
            labels=[self.sequence_speed_label, self.sequence_speed_value, self.sequence_speed_unit],
            target=motion_controller.camera_speed,
            value_label=self.sequence_speed_value,
            value_range=(0.001, 0.5),
            value_increment=0.001,
            value_fraction_digits=3
        )
        deflection_parameter = HmiParameter(
            name='deflection',
            labels=[self.sequence_deflection_label, self.sequence_deflection_value, self.sequence_deflection_unit],
            target=motion_controller.total_target_path_length,
            value_label=self.sequence_deflection_value,
            value_range=(0.01, 1.2),
            value_increment=0.01,
            value_fraction_digits=2
        )
        duration_parameter = HmiParameter(
            name='duration',
            labels=[self.lin_duration_label, self.lin_duration_value, self.lin_duration_unit],
            target=motion_controller.linear_interpolation_duration,
            value_label=self.lin_duration_value,
            value_range=(1, 100),
            value_increment=1,
            value_fraction_digits=0
        )

        sequencer_parameter_set = HmiParameterSet()
        sequencer_parameter_set.parameter.extend([speed_parameter, deflection_parameter])

        lin_parameter_set = HmiParameterSet()
        lin_parameter_set.parameter.append(duration_parameter)

        # Create the tabs

        Tab.tab_bar = self.tab_bar  # class property, set once for all tabs

        self.tabs.append(Tab(
            display_name='Jogging',
            short_display_name='JOG',
            slot=0,
            parameter_set=None,
            hsm_trigger=hsm.trig_jog,
        ))
        self.tabs.append(Tab(
            display_name='Joystick',
            short_display_name='STICK',
            slot=1,
            parameter_set=None,
            hsm_trigger=hsm.trig_joystick,
            soft_key_c_callback=motion_controller.reset_rotor_reference,
            joystick_button_callback=self.switch_joystick_axes,
        ))
        self.tabs.append(Tab(
            display_name='Linear interpolation',
            short_display_name='LIN',
            slot=2,
            parameter_set=lin_parameter_set,
            hsm_trigger=hsm.trig_sequencer,
            soft_key_a_callback=motion_controller.set_start_marker,
            soft_key_b_callback=motion_controller.set_target_marker,
            soft_key_c_callback=self.switch_parameter,
            rgb_0_button_callback=hsm.trig_proceed,
            joystick_button_callback=self.switch_joystick_axes,
            init_function=self.lin_init,
        ))
        self.tabs.append(Tab(
            display_name='Sequencer',
            short_display_name='SEQ',
            slot=3,
            parameter_set=sequencer_parameter_set,
            hsm_trigger=hsm.trig_sequencer,
            soft_key_c_callback=self.switch_parameter,
            rgb_0_button_callback=hsm.trig_proceed,
            joystick_button_callback=self.switch_joystick_axes,
            init_function=self.sequencer_init,
        ))
        self.tabs.append(Tab(
            display_name='Settings',
            short_display_name='SYS',
            slot=4,
            parameter_set=None,
            hsm_trigger=hsm.trig_sequencer,
        ))

    def switch_parameter(self):
        if self.active_parameter_set:
            num_param_in_set = len(self.active_parameter_set.parameter)
            previous_selected_index = self.active_parameter_set.selected_index
            self.active_parameter_set.selected_index = (previous_selected_index + 1) % num_param_in_set
            self.update_parameter()

    def update_parameter(self):
        if self.active_parameter_set:
            for i in range(len(self.active_parameter_set.parameter)):
                hmi_parameter: HmiParameter = self.active_parameter_set.parameter[i]
                if i == self.active_parameter_set.selected_index:
                    # change labels of parameters to white
                    for label in hmi_parameter.labels:
                        label.setStyleSheet('QLabel{color: #fff}')
                    self.configure_wheel_for_parameter(hmi_parameter)
                else:
                    for label in hmi_parameter.labels:
                        label.setStyleSheet('QLabel{color: #999}')

    def configure_wheel_for_parameter(self, param):
        parameter: HmiParameter = param
        wheel = hardware_manager.wheel
        wheel.counter = parameter.target[0]
        wheel.min = parameter.value_range[0]
        wheel.max = parameter.value_range[1]
        wheel.scale = parameter.value_increment

    @property
    def active_tab(self):
        for tab in self.tabs:
            if tab.is_active:
                return tab
        return None

    @property
    def active_parameter_set(self):
        if self.active_tab:
            return self.active_tab.parameter_set  # can be None as well

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        hsm.trig_emergency_shutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def lin_init(self):
        hsm.sequencer_variant = 'linear_interpolation'

    def sequencer_init(self):
        hsm.sequencer_variant = 'front_linear'

    def cb_rotary_selector_switch(self, index):
        print('Controller: switched to mode ' + str(index))
        # deactivate the currently active tab (only one must be active at all time)
        if self.active_tab:
            self.active_tab.deactivate()
        for tab in self.tabs:
            if tab.slot == index:
                tab.activate()  # this triggers the HSM state transition and updates shows the tab in the GUI
                self.update_parameter()  # this highlights the last selected parameter and configures the wheel
                break

    def switch_joystick_axes(self):
        if hsm.joystick_variant == 'polar':
            motion_controller.toggle_polar_joystick_axes_set()

    def before_hsm_state_change(self, state_name):
        pass

    def after_hsm_state_change(self, state_name):
        # different behavior depending on current mode

        if hsm.is_s_initialized():
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)

    def cb_wheel_counter_change(self, counter):
        if self.active_parameter_set:
            index = self.active_parameter_set.selected_index
            parameter: HmiParameter = self.active_parameter_set.parameter[index]
            fraction_digits = parameter.value_fraction_digits
            parameter.target[0] = round(counter, fraction_digits)
            parameter.value_label.setText(f"{counter:.{fraction_digits}f}")
