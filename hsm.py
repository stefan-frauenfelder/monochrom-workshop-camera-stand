from statemachine import StateMachine, State

from motion_control import motion_controller

from transitions.extensions import HierarchicalMachine


class Hsm(object):

    states = [
        'uninitialized',
        'initializing',
        'initialized',
        'homing',
        'emergencyStop',
        'emergencyShutdown',
        {
            'name': 'operational',
            'initial': 'joystickControl',
            'children': [
                'joystickControl',
                'jogControl',
                {
                    'name': 'sequencerControl',
                    'initial': 'atSetup',
                    'children': [
                        'atSetup',
                        'movingToStart',
                        'atStart',
                        'movingForward',
                        'movingBackwards',
                        'atTarget',
                        'movingToSetup'
                    ]
                }
            ]}
    ]

    def __init__(self, name):

        self.name = name
        self.state_changed_callbacks = []

        # add a state machine behavior
        self.machine = HierarchicalMachine(
            model=self,
            states=Hsm.states,
            initial='uninitialized',
            send_event=True,
            ignore_invalid_triggers=True,
            after_state_change='state_changed_updater')

        # definition of transitions

        # top level transitions
        self.machine.add_transition(trigger='initialize', source='uninitialized', dest='initializing')
        self.machine.add_transition(trigger='done', source='initializing', dest='initialized')
        self.machine.add_transition(trigger='home', source='initialized', dest='homing')
        self.machine.add_transition(trigger='done', source='homing', dest='operational')
        self.machine.add_transition(trigger='emergency_stop', source='*', dest='emergencyStop')
        self.machine.add_transition(trigger='emergency_shutdown', source='*', dest='emergencyShutdown')

        # mode transitions
        self.machine.add_transition(trigger='joystick', source=['operational_jogControl', 'operational_sequencerControl'], dest='operational_joystickControl')
        self.machine.add_transition(trigger='sequence', source=['operational_jogControl', 'operational_joystickControl'], dest='operational_sequencerControl')
        self.machine.add_transition(trigger='jog', source=['operational_joystickControl', 'operational_sequencerControl'], dest='operational_jogControl')

        # definition of callbacks

        # startup and initialization sequence
        self.machine.on_enter(state_name='initializing',
                              callback='initializing_sequence')
        self.machine.on_enter(state_name='homing',
                              callback='homing_sequence')

        # joystick control
        self.machine.on_enter(state_name='operational_joystickControl',
                              callback='start_joystick_control')
        self.machine.on_exit(state_name='operational_joystickControl',
                             callback='stop_joystick_control')

        # jog control
        self.machine.on_enter(state_name='operational_jogControl',
                              callback='start_jog_control')
        self.machine.on_exit(state_name='operational_jogControl',
                             callback='stop_jog_control')

        # emergency behavior
        self.machine.on_enter(state_name='emergencyStop',
                              callback='emergency_stop_sequence')
        self.machine.on_enter(state_name='emergencyShutdown',
                              callback='emergency_shutdown_sequence')

    def state_changed_updater(self, event):
        # state_name =
        print('State changed.')
        if self.state_changed_callbacks:
            for callback in self.state_changed_callbacks:
                callback()

    def start_joystick_control(self, event):
        motion_controller.start_joystick_control()

    def stop_joystick_control(self, event):
        motion_controller.stop_joystick_control()

    def start_jog_control(self, event):
        motion_controller.start_jog_control()

    def stop_jog_control(self, event):
        motion_controller.stop_jog_control()

    def initializing_sequence(self, event):
        motion_controller.initialize_steppers()
        self.done()

    def homing_sequence(self, event):
        motion_controller.homing_run()
        self.done()

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
