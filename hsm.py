import logging

from motion_control import motion_controller

from transitions.extensions import HierarchicalMachine
from transitions.extensions.nesting import NestedState
NestedState.separator = '↦'


class Hsm(object):

    def __init__(self, name):

        logging.basicConfig(level=logging.INFO)

        sequencer_states = [
            's_at_setup',
            's_moving_to_start',
            's_at_start',
            's_moving_forward',
            's_moving_backwards',
            's_at_target',
            's_moving_to_setup'
        ]

        # add a (sub-sub-) state machine
        self.sequencer = HierarchicalMachine(
            states=sequencer_states,
            initial='s_at_setup',
            send_event=True,
            ignore_invalid_triggers=True,
            after_state_change='state_changed_updater')

        operational_states = [
            's_jog_control',
            's_polar_joystick_control',
            's_rectilinear_joystick_control',
            's_mirror_slider',
            {
                'name': 's_sequencer_control',
                'initial': 's_at_setup',
                'children': self.sequencer
            }
        ]

        # add a (sub-) state machine
        self.operator = HierarchicalMachine(
            states=operational_states,
            initial='s_polar_joystick_control',
            send_event=True,
            ignore_invalid_triggers=True,
            after_state_change='state_changed_updater')

        # mode transitions
        self.operator.add_transition(trigger='trig_jog', source='*', dest='s_jog_control')
        self.operator.add_transition(trigger='trig_polar_joystick', source='*', dest='s_polar_joystick_control')
        self.operator.add_transition(trigger='trig_rectilinear_joystick', source='*', dest='s_rectilinear_joystick_control')
        self.operator.add_transition(trigger='trig_mirror_slider', source='*', dest='s_mirror_slider')
        self.operator.add_transition(trigger='trig_sequencer', source='*', dest='s_sequencer_control')

        root_states = [
            's_uninitialized',
            's_initializing',
            's_initialized',
            's_homing',
            's_emergency_stop',
            's_emergency_shutdown',
            {
                'name': 's_operational',
                'initial': 's_polar_joystick_control',
                'children': self.operator
            }
        ]

        # add the root state machine
        self.system = HierarchicalMachine(
            model=self,
            states=root_states,
            initial='s_uninitialized',
            send_event=True,
            ignore_invalid_triggers=True,
            after_state_change='state_changed_updater')

        # top level transitions
        self.system.add_transition(trigger='trig_initialize', source='s_uninitialized', dest='s_initializing')
        self.system.add_transition(trigger='trig_done', source='s_initializing', dest='s_initialized')
        self.system.add_transition(trigger='trig_home', source='s_initialized', dest='s_homing')
        self.system.add_transition(trigger='trig_done', source='s_homing', dest='s_operational')
        self.system.add_transition(trigger='trig_emergency_stop', source='*', dest='s_emergency_stop')
        self.system.add_transition(trigger='trig_emergency_shutdown', source='*', dest='s_emergency_shutdown')

        self.name = name
        self.state_changed_callbacks = []

        # definition of callbacks

        # startup and initialization sequence
        self.system.on_enter(state_name='s_initializing', callback='initializing_sequence')
        self.system.on_enter(state_name='s_homing', callback='homing_sequence')

        # polar joystick control
        self.operator.on_enter(state_name='s_polar_joystick_control', callback='start_polar_joystick_control')
        self.operator.on_exit(state_name='s_polar_joystick_control', callback='stop_polar_joystick_control')

        # rectilinear joystick control
        self.operator.on_enter(state_name='s_rectilinear_joystick_control', callback='start_rectilinear_joystick_control')
        self.operator.on_exit(state_name='s_rectilinear_joystick_control', callback='stop_rectilinear_joystick_control')

        # jog control
        self.operator.on_enter(state_name='s_jog_control', callback='start_jog_control')
        self.operator.on_exit(state_name='s_jog_control', callback='stop_jog_control')

        # emergency behavior
        self.system.on_enter(state_name='s_emergency_stop', callback='emergency_stop_sequence')
        self.system.on_enter(state_name='s_emergency_shutdown', callback='emergency_shutdown_sequence')

    def state_changed_updater(self, event):
        # state_name =
        print('State changed.')
        if self.state_changed_callbacks:
            for callback in self.state_changed_callbacks:
                callback()

    def start_polar_joystick_control(self, event):
        motion_controller.start_polar_joystick_control()

    def stop_polar_joystick_control(self, event):
        motion_controller.stop_continuous_control_thread()

    def start_rectilinear_joystick_control(self, event):
        motion_controller.start_rectilinear_joystick_control()

    def stop_rectilinear_joystick_control(self, event):
        motion_controller.stop_continuous_control_thread()

    def start_jog_control(self, event):
        motion_controller.start_jog_control()

    def stop_jog_control(self, event):
        motion_controller.stop_continuous_control_thread()

    def initializing_sequence(self, event):
        motion_controller.initialize_steppers()
        self.trig_done()

    def homing_sequence(self, event):
        motion_controller.homing_run()
        self.trig_done()

    def test(self):
        print('Yes, works.')

    def emergency_stop_sequence(self, event):
        print(f'\033[91mWARNING: Emergency stop triggered!')
        motion_controller.emergency_stop()

    def emergency_shutdown_sequence(self, event):
        print(f'\033[91mWARNING: Emergency shutdown sequence triggered!')
        motion_controller.emergency_shutdown()


# create the finite state machine
hsm = Hsm('The single HSM')
