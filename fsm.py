from statemachine import StateMachine, State
import threading

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
    s_eshutdown = State('Emergency Shutdown')
    # s_final = State('Final', final=True)

    s_jogging = State("Jogging")

    # moving_to_start = State('Moving to start position')
    # ready = State('Ready for target move')
    # executing_move = State('Executing target move')
    # done = State('Done with target move')
    # returning_to_neutral = State('Returning to neutral position')

    # definition of events, triggering transitions
    e_initialize = s_initial.to(s_initializing)
    e_initialized = s_initializing.to(s_initialized)
    e_home = s_initialized.to(s_homing)
    e_homed = s_homing.to(s_idle)

    e_eshutdown = s_initial.to(s_eshutdown) | s_idle.to(s_eshutdown) | s_initialized.to(s_eshutdown) | s_initializing.to(s_eshutdown) | s_homing.to(s_eshutdown)

    e_jog = s_idle.to(s_jogging)
    e_stop_jog = s_jogging.to(s_idle)

    # get_ready = s_idle.to(moving_to_start)
    # reached_start = moving_to_start.to(ready)
    # go = ready.to(executing_move)
    # reached_target = executing_move.to(done)
    # go_home = done.to(s_idle)

    def on_enter_s_initializing(self):
        motion_controller.initialize_steppers()
        self.e_initialized()

    def on_enter_s_homing(self):
        motion_controller.homing_run()
        self.e_homed()

    def on_enter_s_jogging(self):
        motion_controller.set_jogging_axis('arm')
        motion_controller.start_jogging()

    def on_exit_s_jogging(self):
        motion_controller.stop_jogging()

    def on_enter_s_eshutdown(self):
        print(f'\033[91mWARNING: Emergency shutdown sequence triggered!')
        motion_controller.emergency_shutdown()


# create the finite state machine
mechanics_fsm = MechanicsFsm()