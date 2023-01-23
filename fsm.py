from statemachine import StateMachine, State


class CameraMotionControlFsm(StateMachine):

    def __init__(self, coordinator):
        super().__init__()
        self._coordinator = coordinator
        self._view = None

    def set_view(self, view):
        self._view = view

    # definition of states
    initial = State('Initial', initial=True)
    initialized = State('Initialized')
    idle = State('Idle')
    final = State('Final', final=True)
    jogging_arm = State("Jogging arm")

    initialize = initial.to(initialized)
    home = initialized.to(idle)
    exit = initial.to(final) | idle.to(final) | initialized.to(final)
    jog_arm = idle.to(jogging_arm)

    # def before_cycle(self, event_data=None):
    #     message = event_data.kwargs.get("message", "")
    #     message = ". " + message if message else ""
    #     return "Running {} from {} to {}{}".format(
    #         event_data.event,
    #         event_data.transition.source.id,
    #         event_data.transition.target.id,
    #         message,
    #     )

    def on_initialize(self):
        self._coordinator.initialize_steppers()

    def on_home(self):
        self._coordinator.homing_run()

    def on_enter_initialized(self):
        self._view.homing_button.setEnabled(True)

    def on_enter_jogging_arm(self):
        self._coordinator.start_jogging('arm')

    def on_exit_jogging_arm(self):
        self._coordinator.stop_jogging()

    def on_exit(self):
        self._coordinator.shutdown()
