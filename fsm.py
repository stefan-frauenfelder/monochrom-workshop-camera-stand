from statemachine import StateMachine, State
import threading

from motion_control import motion_controller


class CameraMotionControlFsm(StateMachine):

    def __init__(self):
        super().__init__()

    # definition of states
    s_initial = State('Initial', initial=True)
    s_initialized = State('Initialized')
    s_idle = State('Idle')
    s_final = State('Final', final=True)

    s_jogging = State("Jogging arm")

    s_joystick_calibration = State('Joystick calibration')

    # moving_to_start = State('Moving to start position')
    # ready = State('Ready for target move')
    # executing_move = State('Executing target move')
    # done = State('Done with target move')
    # returning_to_neutral = State('Returning to neutral position')

    # definition of events (transitions)
    e_initialize = s_initial.to(s_initialized)
    e_home = s_initialized.to(s_idle)
    e_exit = s_initial.to(s_final) | s_idle.to(s_final) | s_initialized.to(s_final)
    e_jog = s_idle.to(s_jogging)
    e_stop_jog = s_jogging.to(s_idle)
    e_calibrate_joystick = s_initial.to(s_joystick_calibration)
    e_joystick_calibrated = s_joystick_calibration.to(s_initial)

    # get_ready = s_idle.to(moving_to_start)
    # reached_start = moving_to_start.to(ready)
    # go = ready.to(executing_move)
    # reached_target = executing_move.to(done)
    # go_home = done.to(s_idle)

    # def before_cycle(self, event_data=None):
    #     message = event_data.kwargs.get("message", "")
    #     message = ". " + message if message else ""
    #     return "Running {} from {} to {}{}".format(
    #         event_data.event,
    #         event_data.transition.source.id,
    #         event_data.transition.target.id,
    #         message,
    #     )

    def on_e_initialize(self):
        motion_controller.initialize_steppers()

    def on_e_home(self):
        motion_controller.homing_run()

    def on_enter_s_jogging_arm(self):
        motion_controller.start_jogging('arm')

    def on_exit_s_jogging(self):
        motion_controller.stop_jogging()

    def on_e_exit(self):
        motion_controller.shutdown()


# create the finite state machine
fsm = CameraMotionControlFsm()