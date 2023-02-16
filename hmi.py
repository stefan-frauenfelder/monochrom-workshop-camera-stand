import time
import pigpio

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

from hardware import hardware_manager
from mechanics_fsm import mechanics_fsm
from motion_control import motion_controller


class StateObserver(object):
    def __init__(self, name, owner):
        self.name = name
        self.owner = owner

    def on_enter_state(self, target, event):
        print(f'Mechanics FSM entering state {target.name} triggered by {event}')
        self.owner.update()


class View(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("Camera Motion Control")

        # create an observer which can observe an FSM to observe the mechanics FSM
        mechanics_fsm_observer = StateObserver('views_mechanics_observer', self)
        # add the observer to the mechanics FSM, it will call View's update method on every state change
        mechanics_fsm.add_observer(mechanics_fsm_observer)

        self.initialize_button.clicked.connect(mechanics_fsm.e_initialize)

        self.homing_button.clicked.connect(mechanics_fsm.e_home)

        # utility buttons
        self.joystick_calibration_button.clicked.connect(self.quick_test)
        self.quick_test_button.clicked.connect(self.quick_test)
        self.estop_button.clicked.connect(mechanics_fsm.e_emergency_shutdown)

    def closeEvent(self, event):
        print("Window close command issued, passing e_exit event to FSM.")
        mechanics_fsm.e_eshutdown()
        # close the window and exit the application
        event.accept()

    def quick_test(self):
        pass

    def update(self):

        if mechanics_fsm.current_state.name == 'Initialized':
            self.homing_button.setEnabled(True)
        else:
            self.homing_button.setEnabled(False)


class Controller:

    def __init__(self):

        self.mode = 0

        # create an observer which can observe an FSM to observe the mechanics FSM
        mechanics_fsm_observer = StateObserver('controllers_mechanics_observer', self)
        # add the observer to the mechanics FSM, it will call Controller's update method on every state change
        mechanics_fsm.add_observer(mechanics_fsm_observer)

        hardware_manager.rotary_selector_callbacks.append(self.cb_rotary_selector_switch)
        hardware_manager.rgb_button_callbacks.append(self.cb_joystick_button_change)

    def cb_rotary_selector_switch(self, new_mode):
        print('Controller: switched from mode ' + str(self.mode) +  ' to mode ' + str(new_mode))
        if new_mode == 0:
            mechanics_fsm.e_joystick_control()
        elif self.mode == 0:
            mechanics_fsm.e_idle()
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



