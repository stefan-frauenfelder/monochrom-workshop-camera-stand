import logging

from motion_control import motion_controller

from transitions.extensions import HierarchicalMachine
from transitions.extensions.nesting import NestedState
NestedState.separator = '↦'


class Hsm(object):

    def __init__(self, name):

        # logging.basicConfig(level=logging.INFO)

        self.joystick_variant = 'polar'
        self.sequencer_variant = 'front_linear'

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

        # sequencer transitions
        self.sequencer.add_transition(trigger='trig_proceed', source='s_at_setup', dest='s_moving_to_start')
        self.sequencer.add_transition(trigger='trig_proceed', source='s_at_start', dest='s_moving_forward')
        self.sequencer.add_transition(trigger='trig_proceed', source='s_at_target', dest='s_moving_to_setup')

        self.sequencer.add_transition(trigger='trig_go_back', source='s_at_target', dest='s_moving_backwards')
        self.sequencer.add_transition(trigger='trig_go_back', source='s_at_start', dest='s_moving_to_setup')

        self.sequencer.add_transition(trigger='trig_reached', source='s_moving_to_start', dest='s_at_start')
        self.sequencer.add_transition(trigger='trig_reached', source='s_moving_forward', dest='s_at_target')
        self.sequencer.add_transition(trigger='trig_reached', source='s_moving_backwards', dest='s_at_start')
        self.sequencer.add_transition(trigger='trig_reached', source='s_moving_to_setup', dest='s_at_setup')

        # sequencer callbacks
        self.sequencer.on_enter(state_name='s_at_setup', callback='cb_on_enter_s_at_setup')
        self.sequencer.on_enter(state_name='s_at_start', callback='cb_on_enter_s_at_start')
        self.sequencer.on_enter(state_name='s_at_target', callback='cb_on_enter_s_at_target')
        self.sequencer.on_enter(state_name='s_moving_to_start', callback='cb_on_enter_s_moving_to_start')
        self.sequencer.on_enter(state_name='s_moving_to_setup', callback='cb_on_enter_s_moving_to_setup')
        self.sequencer.on_enter(state_name='s_moving_forward', callback='cb_on_enter_s_moving_forward')
        self.sequencer.on_enter(state_name='s_moving_backwards', callback='cb_on_enter_s_moving_backwards')

        operational_states = [
            's_jog_control',
            's_joystick_control',
            {
                'name': 's_sequencer_control',
                'initial': 's_at_setup',
                'children': self.sequencer
            }
        ]

        # add a (sub-) state machine
        self.operator = HierarchicalMachine(
            states=operational_states,
            initial='s_joystick_control',
            send_event=True,
            ignore_invalid_triggers=True,
            after_state_change='state_changed_updater')

        # mode transitions
        self.operator.add_transition(trigger='trig_jog', source='*', dest='s_jog_control')
        self.operator.add_transition(trigger='trig_joystick', source='*', dest='s_joystick_control')
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
                'initial': 's_joystick_control',
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
            before_state_change='cb_before_any_state_change',
            after_state_change='cb_after_any_state_change')

        # top level transitions
        self.system.add_transition(trigger='trig_initialize', source='s_uninitialized', dest='s_initializing')
        self.system.add_transition(trigger='trig_done', source='s_initializing', dest='s_initialized')
        self.system.add_transition(trigger='trig_home', source='s_initialized', dest='s_homing')
        self.system.add_transition(trigger='trig_done', source='s_homing', dest='s_operational')
        self.system.add_transition(trigger='trig_emergency_stop', source='*', dest='s_emergency_stop')
        self.system.add_transition(trigger='trig_emergency_shutdown', source='*', dest='s_emergency_shutdown')

        self.name = name
        self.after_any_state_change_callbacks = []
        self.before_any_state_change_callbacks = []

        # definition of callbacks

        # startup and initialization sequence
        self.system.on_enter(state_name='s_initializing', callback='initializing_sequence')
        self.system.on_enter(state_name='s_homing', callback='homing_sequence')

        # joystick control
        self.operator.on_enter(state_name='s_joystick_control', callback='start_joystick_control')
        self.operator.on_exit(state_name='s_joystick_control', callback='stop_joystick_control')

        # jog control
        self.operator.on_enter(state_name='s_jog_control', callback='start_jog_control')
        self.operator.on_exit(state_name='s_jog_control', callback='stop_jog_control')

        # sequencer control
        self.operator.on_enter(state_name='s_sequencer_control', callback='start_sequencer_control')
        self.operator.on_exit(state_name='s_sequencer_control', callback='stop_sequencer_control')

        # emergency behavior
        self.system.on_enter(state_name='s_emergency_stop', callback='emergency_stop_sequence')
        self.system.on_enter(state_name='s_emergency_shutdown', callback='emergency_shutdown_sequence')

    def cb_after_any_state_change(self, event):
        state_string = self.state
        print('State changed to ' + state_string + '.')
        if self.after_any_state_change_callbacks:
            for callback in self.after_any_state_change_callbacks:
                callback(state_string)

    def cb_before_any_state_change(self, event):
        state_string = self.state
        print('State changing from ' + state_string + '.')
        if self.before_any_state_change_callbacks:
            for callback in self.before_any_state_change_callbacks:
                callback(state_string)

    # sequencer callbacks

    def cb_on_enter_s_at_setup(self, event):
        pass

    def cb_on_enter_s_at_start(self, event):
        pass

    def cb_on_enter_s_at_target(self, event):
        pass

    def cb_on_enter_s_moving_to_start(self, event):
        motion_controller.move_to_front_linear_start_position(1.0)
        self.trig_reached()

    def cb_on_enter_s_moving_to_setup(self, event):
        motion_controller.synchronized_move_to_marker(motion_controller.current_sequence_setup_marker)
        self.trig_reached()

    def cb_on_enter_s_moving_forward(self, event):
        motion_controller.front_linear_motion(total_deflection=1.0)
        self.trig_reached()

    def cb_on_enter_s_moving_backwards(self, event):
        self.trig_reached()

    def start_jog_control(self, event):
        motion_controller.start_jog_control()

    def stop_jog_control(self, event):
        motion_controller.stop_continuous_control_thread()

    def start_joystick_control(self, event):
        if self.joystick_variant == 'polar':
            motion_controller.start_polar_joystick_control()
        elif self.joystick_variant == 'rectilinear':
            motion_controller.start_rectilinear_joystick_control()

    def stop_joystick_control(self, event):
        motion_controller.stop_continuous_control_thread()

    def start_sequencer_control(self, event):
        pass

    def stop_sequencer_control(self, event):
        pass

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
