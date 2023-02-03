import time
import serial
import threading
import json
import pigpio

import sequent_ports

from nanotec import *
from motion_control import *

# globals for GPIO pin assignment

# port extension cards
SEQUENT_INTERUPT_GPIO = 5


class Coordinator():

    def __init__(self):

        # setup the serial port for RS485 communication to the stepper motors
        serial_port = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=1)
        # a lock for the commander to enable multiple threads using the same single hardware resourse
        commander_lock = threading.Lock()
        # create a single commander using the single serial port
        self.commander = Commander(ser=serial_port, lock=commander_lock)
        # axes are empty for now and are created upon user request
        self.axes = None
        self.controller = None
        # setup the hardware opto-isolated input and relay outputs cards
        self.io_card = sequent_ports.SequentPorts(SEQUENT_INTERUPT_GPIO)
        # create a couple of events which manage flags that allow to abort threads
        self._jogging_flag = threading.Event()
        self._joystick_calibration_flag = threading.Event()

    def set_controller(self, controller):
        self.controller = controller

    def set_adc(self, adc):
        self.adc = adc

    def initialize_steppers(self):
        # create all the stepper instances using the stepper configuration files
        # Horizontal axis
        arm = LocatedStepper(self.commander, self.io_card, json.loads(open("stepper_config/arm_config.json").read()))
        # Vertical axis
        lift = LocatedStepper(self.commander, self.io_card, json.loads(open("stepper_config/lift_config.json").read()))
        # rotation of the arm
        rotor = LocatedStepper(self.commander, self.io_card, json.loads(open("stepper_config/rotor_config.json").read()))
        # paning of the camera
        pan = LocatedStepper(self.commander, self.io_card, json.loads(open("stepper_config/pan_config.json").read()))
        # tilting of the camera
        tilt = LocatedStepper(self.commander, self.io_card, json.loads(open("stepper_config/tilt_config.json").read()))

        self.axes = {
            'arm': arm,
            'lift': lift,
            'rotor': rotor,
            'pan': pan,
            'tilt': tilt
        }

        # power up the steppers
        for axis in self.axes.values():
            # power up the individual axes with a slight delay to avoid power drop
            axis.power_up()
            time.sleep(0.2)

        # initialize the steppers with a default configuration
        for axis in self.axes.values():
            axis.initialize()

        print('All motors powered up and initialized.')

        # motion_controller.run_front_linear_sequence(distance=0.6, duration=30, step_frequency=10, start_s=0.3, stop_s=-0.3)

        # motion_controller.run_circular_sequence(distance=0.7, radius=0.3, duration=30, step_frequency=10, start_angle=1, stop_angle=2 * math.gpio - 1)

    def homing_run(self):
        # define individual threads for all axis to run homing in parallel
        find_limits_arm_thread = threading.Thread(target=self.axes['arm'].find_linear_stepper_limits)
        find_limits_lift_thread = threading.Thread(target=self.axes['lift'].find_linear_stepper_limits)
        # the lambda prevents the function from being interpreted here already. Don't ask me how. I have no clue.
        find_limits_rotor_thread = threading.Thread(target=lambda: self.axes['rotor'].find_rotor_origin(limit_switch='external', direction=Direction.negative))
        find_origin_pan_thread = threading.Thread(target=lambda: self.axes['pan'].find_rotational_stepper_origin(limit_switch='internal', direction=Direction.negative))
        find_origin_tilt_thread = threading.Thread(target=lambda: self.axes['tilt'].find_rotational_stepper_origin(limit_switch='internal', direction=Direction.positive))
        # find limits of major axes
        find_limits_arm_thread.start()
        find_limits_lift_thread.start()
        find_limits_rotor_thread.start()
        # wait for completion
        find_limits_arm_thread.join()
        find_limits_lift_thread.join()
        find_limits_rotor_thread.join()
        # find limits of minor axes
        find_origin_pan_thread.start()
        find_origin_tilt_thread.start()
        # wait for completion
        find_origin_pan_thread.join()
        find_origin_tilt_thread.join()
        # set fake limits because there are no limit switches yet
        self.axes['pan'].set_fake_rotational_stepper_limits(math.pi)
        self.axes['tilt'].set_fake_rotational_stepper_limits(math.pi)
        self.axes['rotor'].set_fake_rotational_stepper_limits(math.pi / 4)

        # init only here since motion controller assumes homed axes
        self.init_motion_controller()

    def activate_joystick_mode(self):
        # limit speeds for joystick mode
        self.axes['arm'].speed = 0.05
        self.axes['lift'].speed = 0.05
        self.axes['rotor'].speed = 0.1
        self.axes['pan'].speed = 0.1
        self.axes['tilt'].speed = 0.1

        for axis in self.axes.values():
            axis.mode = 'joystick_mode'
            axis.run()

        input("Press Enter to stop and continue.")

        for axis in self.axes.values():
            axis.stop()
            axis.mode = 'relative_positioning'

    def shutdown(self):
        # power down all motors
        for axis in self.axes.values():
            axis.shutdown()
            time.sleep(0.1)
        print('Steppers powered down.')

    def start_jogging(self, axis_name):
        # get the axis by name
        axis = self.axes[axis_name]
        # set the jogging flag to true. It is checked in the while loop of jogging
        self._jogging_flag.set()
        # create a new thread and start it
        thread = threading.Thread(target=lambda: self.motion_controller.jog(axis=axis, wheel=self.controller.wheel, flag=self._jogging_flag))
        thread.start()

    def stop_jogging(self):
        # clearing the jogging flag will cause the previously started jogging thread to return
        self._jogging_flag.clear()

    def init_motion_controller(self):
        self.motion_controller = MotionController(self.axes)

    def calibrate_joystick(self):
        # set the flag to true. It is checked in the while loop of calibration
        self._joystick_calibration_flag.set()
        # create a new thread and start it
        thread = threading.Thread(target=lambda: self.controller.calibrate_joystick())
        thread.start()

    def stop_calibrating_joystick(self):
        # clearing the flag will cause the previously started calibration thread to return
        self._joystick_calibration_flag.clear()


