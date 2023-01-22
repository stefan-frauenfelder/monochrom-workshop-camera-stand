from statemachine import StateMachine, State


class CameraMotionControlFsm(StateMachine):

    def __init__(self, controller):
        super().__init__()
        self._controller = controller
        self._view = None

    def set_view(self, view):
        self._view = view

    # definition of states
    initial = State('Initial', initial=True)
    initialized = State('Initialized')
    idle = State('Idle')
    final = State('Final', final=True)
    # jogging_arm = State("Jogging arm")

    initialize = initial.to(initialized)
    home = initialized.to(idle)
    exit = initial.to(final) | idle.to(final) | initialized.to(final)

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
        self._controller.initialize_steppers()

    def on_home(self):
        self._controller.homing_run()

    def on_enter_initialized(self):
        self._view.homing_button.setEnabled(True)

    def on_exit(self):
        self._controller.shutdown()
