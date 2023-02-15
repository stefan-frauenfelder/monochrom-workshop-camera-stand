from statemachine import StateMachine, State

from motion_control import motion_controller


class MechanicsFsm(StateMachine):

    def __init__(self):
        super().__init__()

    # definition of states
    s_initial = State('Initial', initial=True)
    s_initializing = State('Initializing')
    s_initialized = State('Initialized')
    s_homing = State('Homing')
    s_idle = State('Idle')
    s_jog_control = State('Jog Control')
    s_joystick_control = State('Joystick Control')
    s_coordinated_move = State('Coordinated Move')
    s_emergency_stop = State('Emergency Stop')
    s_emergency_shutdown = State('Emergency Shutdown')
    s_quick_test = State('Quick Test')

    # definition of events, triggering transitions
    e_initialize = s_initial.to(s_initializing)
    e_initialized = s_initializing.to(s_initialized)
    e_home = s_initialized.to(s_homing)
    e_homed = s_homing.to(s_idle)
    e_move = s_idle.to(s_coordinated_move)

    e_idle = \
        s_jog_control.to(s_idle) | \
        s_joystick_control.to(s_idle) | \
        s_coordinated_move.to(s_idle) | \
        s_emergency_stop.to(s_idle)

    e_jog = \
        s_idle.to(s_jog_control) | \
        s_jog_control.to(s_idle)

    e_joystick_control = \
        s_idle.to(s_joystick_control) | \
        s_joystick_control.to(s_idle)

    e_quick_test = \
        s_idle.to(s_quick_test) | \
        s_quick_test.to(s_idle)

    e_emergency_stop = \
        s_initial.to(s_emergency_stop) | \
        s_initializing.to(s_emergency_stop) | \
        s_initialized.to(s_emergency_stop) | \
        s_homing.to(s_emergency_stop) | \
        s_idle.to(s_emergency_stop) | \
        s_joystick_control.to(s_emergency_stop) | \
        s_jog_control.to(s_emergency_stop) | \
        s_coordinated_move.to(s_emergency_stop) | \
        s_quick_test.to(s_emergency_stop)

    e_emergency_shutdown = \
        s_initial.to(s_emergency_shutdown) | \
        s_initializing.to(s_emergency_shutdown) | \
        s_initialized.to(s_emergency_shutdown) | \
        s_homing.to(s_emergency_shutdown) | \
        s_idle.to(s_emergency_shutdown) | \
        s_joystick_control.to(s_emergency_shutdown) | \
        s_jog_control.to(s_emergency_shutdown) | \
        s_coordinated_move.to(s_emergency_shutdown) | \
        s_quick_test.to(s_emergency_shutdown) | \
        s_emergency_stop.to(s_emergency_shutdown)

    def on_enter_s_initializing(self):
        motion_controller.initialize_steppers()
        self.e_initialized()

    def on_enter_s_homing(self):
        motion_controller.homing_run()
        self.e_homed()

    def on_enter_s_jog_control(self):
        motion_controller.set_jogging_axis('arm')
        motion_controller.start_jogging()

    def on_exit_s_jog_control(self):
        motion_controller.stop_jogging()

    def on_enter_s_joystick_control(self):
        motion_controller.start_joysticking()

    def on_exit_s_joystick_control(self):
        motion_controller.stop_joysticking()

    def on_enter_s_emergency_stop(self):
        print(f'\033[91mWARNING: Emergency stop triggered!')
        motion_controller.emergency_stop()

    def on_enter_s_emergency_shutdown(self):
        print(f'\033[91mWARNING: Emergency shutdown sequence triggered!')
        motion_controller.emergency_shutdown()

    def on_enter_s_quick_test(self):
        pass

    def on_exit_s_quick_test(self):
        pass


# create the finite state machine
mechanics_fsm = MechanicsFsm()
