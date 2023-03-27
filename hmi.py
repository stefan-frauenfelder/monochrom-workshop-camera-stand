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
                 soft_key_c_callback=None):

        self.display_name = display_name
        self.short_display_name = short_display_name
        self.slot = slot  # range: 0...4
        self.parameter_set: HmiParameterSet = parameter_set
        self.hsm_trigger = hsm_trigger
        self.soft_key_a_callback = soft_key_a_callback
        self.soft_key_b_callback = soft_key_b_callback
        self.soft_key_c_callback = soft_key_c_callback
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
            self._active = True
            # tell the (parent) QT tab bar to make this tab visible/active
            self.tab_bar.setCurrentIndex(self.slot)

    def deactivate(self):
        hardware_manager.soft_key_1_callbacks[0] = None
        hardware_manager.soft_key_2_callbacks[0] = None
        hardware_manager.soft_key_3_callbacks[0] = None
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

        self.mode = 0

        self.tabs = []

        # Callback registration for static callbacks

        # add a callbacks to the HSM to be notified about state changes
        hsm.before_any_state_change_callbacks.append(self.before_hsm_state_change)
        hsm.after_any_state_change_callbacks.append(self.after_hsm_state_change)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.joystick_button_callbacks.append(self.cb_joystick_button_change)
        hardware_manager.rgb_0_button_callbacks.append(self.cb_rgb_0_button_change)
        hardware_manager.wheel_callbacks.append(self.cb_wheel_counter_change)

        # hardware_manager.soft_key_3_callbacks.append(self.switch_parameter)

        # Parameterization

        sequencer_parameter_set = HmiParameterSet()
        sequencer_parameter_set.parameter.append(HmiParameter(
            name='speed',
            labels=[self.speed_label, self.speed_value, self.speed_unit],
            target=motion_controller.camera_speed,
            value_label=self.speed_value,
            value_range=(0.001, 0.5),
            value_increment=0.001,
            value_fraction_digits=3
        ))
        sequencer_parameter_set.parameter.append(HmiParameter(
            name='deflection',
            labels=[self.deflection_label, self.deflection_value, self.deflection_unit],
            target=motion_controller.total_target_path_length,
            value_label=self.deflection_value,
            value_range=(0.01, 1.2),
            value_increment=0.01,
            value_fraction_digits=2
        ))
        # add the set to the space
        self.parameter_space['sequencer_parameter'] = sequencer_parameter_set

        # Create the tabs

        Tab.tab_bar = self.tab_bar  # class property, set once for all tabs

        self.tabs.append(Tab(
            display_name='Jogging',
            short_display_name='JOG',
            slot=0,
            parameter_set=None,
            hsm_trigger=hsm.trig_jog
        ))
        self.tabs.append(Tab(
            display_name='Joystick',
            short_display_name='STICK',
            slot=1,
            parameter_set=None,
            hsm_trigger=hsm.trig_joystick,
            soft_key_c_callback=motion_controller.reset_rotor_reference
        ))
        self.tabs.append(Tab(
            display_name='A-B',
            short_display_name='A-B',
            slot=2,
            parameter_set=None,
            hsm_trigger=hsm.trig_sequencer
        ))
        self.tabs.append(Tab(
            display_name='Sequencer',
            short_display_name='SEQ',
            slot=3,
            parameter_set=sequencer_parameter_set,
            hsm_trigger=hsm.trig_sequencer,
            soft_key_c_callback=self.switch_parameter
        ))
        self.tabs.append(Tab(
            display_name='Settings',
            short_display_name='SYS',
            slot=4,
            parameter_set=None,
            hsm_trigger=hsm.trig_sequencer
        ))

    def switch_parameter(self):
        num_param_in_set = len(self.active_parameter_set.parameter)
        previous_selected_index = self.active_parameter_set.selected_index
        self.active_parameter_set.selected_index = (previous_selected_index + 1) % num_param_in_set
        for i in range(num_param_in_set):
            hmi_parameter: HmiParameter = self.active_parameter_set.parameter[i]
            if i == self.active_parameter_set.selected_index:
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
            return self.active_tab.parameter_set

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        hsm.trig_emergency_shutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def cb_rotary_selector_switch(self, index):
        print('Controller: switched to mode ' + str(index))
        # deactivate the currently active tab (only one must be active at all time)
        if self.active_tab:
            self.active_tab.deactivate()
        for tab in self.tabs:
            if tab.slot == index:
                tab.activate()  # this triggers the HSM state transition and updates shows the tab in the GUI
                break

    def cb_joystick_button_change(self, value):
        # only if button is pressed
        if value:
            # different behavior depending on current mode

            # polar joystick mode
            if hsm.is_s_operational.s_joystick_control():
                if hsm.joystick_variant == 'polar':
                    motion_controller.toggle_polar_joystick_axes_set()
                    print('Toggled joystick axes.')

    def cb_rgb_0_button_change(self, value):
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
        if self.active_parameter_set:
            index = self.active_parameter_set.selected_index
            parameter: HmiParameter = self.active_parameter_set.parameter[index]
            fraction_digits = parameter.value_fraction_digits
            parameter.target[0] = round(counter, fraction_digits)
            parameter.value_label.setText(f"{counter:.{fraction_digits}f}")
