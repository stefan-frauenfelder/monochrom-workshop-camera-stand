from statemachine import StateMachine, State

from motion_control import motion_controller


class SequencerFsm(StateMachine):

    def __init__(self):
        super().__init__()

    # definition of states
    s_at_setup = State('At setup', initial=True)
    s_moving_to_start = State('Moving to start')
    s_at_start = State('At start')
    s_moving_forward = State('Moving forward')
    s_at_target = State('At target')
    s_moving_backwards = State('Moving backwards')
    s_moving_to_setup = State('Moving to setup')

    e_move_to_start = s_at_setup.to(s_moving_to_start)
    e_reached_start = s_moving_to_start.to(s_at_start) | s_moving_backwards.to(s_at_start)
    e_move_forward = s_at_start.to(s_moving_forward)
    e_reached_target = s_moving_forward.to(s_at_target)
    e_move_backwards = s_at_target.to(s_moving_backwards)
    e_move_to_setup = s_at_target.to(s_moving_to_setup) | s_at_start.to(s_moving_to_setup)
    e_reached_setup = s_moving_to_setup.to(s_at_setup)

    e_abort = \
        s_at_setup.to(s_at_setup) | \
        s_moving_to_start.to(s_at_setup) | \
        s_at_start.to(s_at_setup) | \
        s_moving_forward.to(s_at_setup) | \
        s_at_target.to(s_at_setup) | \
        s_moving_backwards.to(s_at_setup) | \
        s_moving_to_setup.to(s_at_setup)


# create the finite state machine
sequencer_fsm = SequencerFsm()
