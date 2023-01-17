import time
import serial
import threading
import json

import sequent_ports

from nanotec import *
from motion_control import *

# globals for GPIO pin assignment

# port extension cards
SEQUENT_INTERUPT_GPIO = 5
# Jog wheel
JOG_WHEEL_A_GPIO = 23
JOG_WHEEL_B_GPIO = 24
# Switches
ENTER_SW_GPIO = 17


class Controller():

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

        # setup the hardware optoisolated input and relay outpus cards
        self.io_card = sequent_ports.SequentPorts(SEQUENT_INTERUPT_GPIO)

        # Rotary encoder jog wheel
        self.wheel = Rotary(clk_gpio=JOG_WHEEL_A_GPIO, dt_gpio=JOG_WHEEL_B_GPIO, sw_gpio=ENTER_SW_GPIO)
        self.wheel.setup_rotary(
            min=0,
            max=1000,
            scale=1,
            debounce=0,
            rotary_callback=self.rotary_callback
        )

    def rotary_callback(self, counter):
        print("Counter value: ", counter)

    def create_stepper_instances(self):
        # create all the stepper instances using the stepper configuration files
        # Horizontal axis
        arm = LocatedStepper(self.commander, self.io_card, json.loads(open("arm_config.json").read()))

        # Vertical axis
        lift = LocatedStepper(self.commander, self.io_card, json.loads(open("lift_config.json").read()))

        # rotation of the arm
        rotor = LocatedStepper(self.commander, self.io_card, json.loads(open("rotor_config.json").read()))

        # paning of the camera
        pan = LocatedStepper(self.commander, self.io_card, json.loads(open("pan_config.json").read()))

        # tilting of the camera
        tilt = LocatedStepper(self.commander, self.io_card, json.loads(open("tilt_config.json").read()))

        print('All motors powered up and initialized.')

        self.axes = {
            'arm': arm,
            'lift': lift,
            'rotor': rotor,
            'pan': pan,
            'tilt': tilt
        }

    def initialize(self):

        self.create_stepper_instances()

        input("Press Enter to activate joystick mode...")

        activate_joystick_mode(self.axes)

        input("Press Enter to start the homing sequence.")

        self.homing_run()

        motion_controller = MotionController(axes_dict)

        motion_controller.jog_mode(axis_name='arm', wheel=self.wheel)

        # motion_controller.run_front_linear_sequence(distance=0.6, duration=30, step_frequency=10, start_s=0.3, stop_s=-0.3)

        # motion_controller.run_circular_sequence(distance=0.7, radius=0.3, duration=30, step_frequency=10, start_angle=1, stop_angle=2 * math.pi - 1)

    def homing_run(self):

        # auxilary functions
        def find_rotor_origin():
            rotor.find_rotor_origin(limit_switch='external', direction=Direction.negative)

        def find_pan_origin():
            pan.find_rotational_stepper_origin(limit_switch='internal', direction=Direction.negative)

        def find_tilt_origin():
            tilt.find_rotational_stepper_origin(limit_switch='internal', direction=Direction.positive)

        find_limits_arm_thread = threading.Thread(target=arm.find_linear_stepper_limits)
        find_limits_lift_thread = threading.Thread(target=lift.find_linear_stepper_limits)
        find_limits_rotor_thread = threading.Thread(target=find_rotor_origin)

        find_limits_arm_thread.start()
        find_limits_lift_thread.start()
        find_limits_rotor_thread.start()

        find_limits_arm_thread.join()
        find_limits_lift_thread.join()
        find_limits_rotor_thread.join()

        find_origin_pan_thread = threading.Thread(target=find_pan_origin)
        find_origin_tilt_thread = threading.Thread(target=find_tilt_origin)

        find_origin_pan_thread.start()
        find_origin_tilt_thread.start()

        find_origin_pan_thread.join()
        find_origin_tilt_thread.join()

        pan.set_fake_rotational_stepper_limits(math.pi)
        tilt.set_fake_rotational_stepper_limits(math.pi)
        rotor.set_fake_rotational_stepper_limits(math.pi / 4)

    def activate_joystick_mode(axes_dict):

        arm.speed = 0.05
        lift.speed = 0.05
        rotor.speed = 0.1
        pan.speed = 0.1
        tilt.speed = 0.1

        for axis in axes_dict.values():
            axis.mode = 'joystick_mode'
            axis.run()

        input("Press Enter to stop and continue.")

        for axis in axes_dict.values():
            axis.stop()
            axis.mode = 'relative_positioning'

    def shutdown(self):
        # power down all motors
        for i in range(1, 6):
            self.io_card.set_output(i, 0)
            time.sleep(0.1)
        print('Steppers powered down.')
