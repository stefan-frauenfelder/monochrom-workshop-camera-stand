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
            ignore_invalid_triggers=True,
            after_state_change='state_changed')

        # top level transitions
        self.machine.add_transition(trigger='initialize', source='uninitialized', dest='initializing')
        self.machine.add_transition(trigger='done', source='initializing', dest='initialized')
        self.machine.add_transition(trigger='home', source='initialized', dest='homing')
        self.machine.add_transition(trigger='done', source='homing', dest='operational')
        self.machine.add_transition(trigger='emergency_stop', source='*', dest='emergencyStop')
        self.machine.add_transition(trigger='emergency_shutdown', source='*', dest='emergencyShutdown')

        self.machine.on_enter(state_name='operational_joystickControl', callback='motion_controller.start_joystick_control')
        self.machine.on_exit(state_name='operational_joystickControl', callback='motion_controller.stop_joystick_control')

    def state_changed(self):
        print('State changed.')
        for callback in self.state_changed_callbacks:
            callback()

    def on_enter_initializing(self):
        motion_controller.initialize_steppers()
        self.done()

    def on_enter_homing(self):
        motion_controller.homing_run()
        self.done()

    def test(self):
        print('Yes, works.')

    def on_enter_emergencyStop(self):
        print(f'\033[91mWARNING: Emergency stop triggered!')
        motion_controller.emergency_stop()

    def on_enter_emergencyShutdown(self):
        print(f'\033[91mWARNING: Emergency shutdown sequence triggered!')
        motion_controller.emergency_shutdown()


# create the finite state machine
hsm = Hsm('The single HSM')
