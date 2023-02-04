import time
import math
import threading
import serial
import json
import pigpio

import sequent_ports

from nanotec import *
from motion_math import *
from hardware import wheel

default_linear_speed = 0.1
default_rotor_speed = 0.1
default_pan_tilt_speed = 0.8

# port extension cards
SEQUENT_INTERRUPT_GPIO = 5


class MotionController:

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

        # setup the hardware opto-isolated input and relay outputs cards
        self.io_card = sequent_ports.SequentPorts(SEQUENT_INTERRUPT_GPIO)
        # create a couple of events which manage flags that allow to abort threads
        self._jogging_flag = threading.Event()
        self._joystick_calibration_flag = threading.Event()

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
        thread = threading.Thread(target=lambda: self.jog(axis=axis, wheel=self.controller.wheel, flag=self._jogging_flag))
        thread.start()

    def stop_jogging(self):
        # clearing the jogging flag will cause the previously started jogging thread to return
        self._jogging_flag.clear()


    def disarm_all(self):
        # iterate through axes
        for axis in self.axes.values():
            axis.armed = False

    def emergency_stop(self):
        # iterate through axes
        for axis in self.axes.values():
            # immediately stop axis
            axis.immediate_stop()

    def parallel_run(self):
        # create a list of threads
        threads = []
        # iterate through axes
        for axis in self.axes.values():
            # check if it was configured for a run (travel was set but run was not called)
            if axis.armed:
                # create a new (blocking) thread, add it to the list, and start it
                thread = threading.Thread(target=axis.blocking_run)
                threads.append(thread)
                thread.start()
        # join all threads
        for thread in threads:
            thread.join()

    def two_simple_motions(self):

        # move arm to the starting point
        self.axes['arm'].goto_absolute_position(position=0.5, speed=default_linear_speed)
        self.axes['arm'].blocking_run()

        self.axes['lift'].goto_absolute_position(position=1, speed=default_linear_speed)
        self.axes['lift'].blocking_run()

        input("Press Enter to continue...")

        self.axes['arm'].goto_absolute_position(position=0.7, speed=default_linear_speed)
        self.axes['lift'].goto_absolute_position(position=1.2, speed=default_linear_speed)

        input("Press Enter to continue...")

        self.parallel_run()

        input("Press Enter to continue...")

    def move_to_start_angle(self, distance, radius, start_angle):

        # lift the pen from the paper
        self.axes['lift'].goto_absolute_position(position=1.2, speed=default_linear_speed)
        self.axes['arm'].goto_absolute_position(position=0.4, speed=default_linear_speed)
        self.parallel_run()

        input("Press Enter to arm motors...")

        # move arm to the starting point
        arm_start_position = circular_motion_arm_position(start_angle, distance, radius)
        self.axes['arm'].goto_absolute_position(position=arm_start_position, speed=default_linear_speed)
        # move rotor to starting point
        rotor_starting_angle = circular_motion_rotor_angle(start_angle, distance, radius)
        self.axes['rotor'].goto_absolute_position(position=rotor_starting_angle, speed=default_rotor_speed)
        # move pan to starting point
        pan_starting_angle = circular_motion_pan_angle(start_angle, distance, radius)
        self.axes['pan'].goto_absolute_position(position=pan_starting_angle, speed=default_pan_tilt_speed)
        # move the lift to above target height
        self.axes['lift'].goto_absolute_position(position=0.9, speed=default_linear_speed)
        # point the pen down
        self.axes['tilt'].goto_absolute_position(position=-0.5, speed=default_pan_tilt_speed)

        input("Press Enter to start parallel run...")
        self.parallel_run()

        # lower the pen to the paper
        self.axes['lift'].goto_absolute_position(position=0.851, speed=default_linear_speed)
        self.axes['lift'].blocking_run()

    def back_up(self):

        # lift the pen from the paper
        self.axes['lift'].goto_absolute_position(position=0.87, speed=default_linear_speed)
        self.axes['lift'].blocking_run()

        self.axes['lift'].goto_absolute_position(position=1, speed=default_linear_speed)
        # go to neutral
        self.axes['tilt'].goto_absolute_position(position=0, speed=default_pan_tilt_speed)
        self.axes['pan'].goto_absolute_position(position=0, speed=default_pan_tilt_speed)
        self.axes['rotor'].goto_absolute_position(position=0, speed=default_rotor_speed)
        # retract the arm
        self.axes['arm'].goto_absolute_position(position=0.3, speed=default_linear_speed)
        # execute
        self.parallel_run()

    def go_neutral(self):
        # go to neutral
        self.axes['tilt'].goto_absolute_position(position=0, speed=default_pan_tilt_speed)
        self.axes['pan'].goto_absolute_position(position=0, speed=default_pan_tilt_speed)
        self.axes['rotor'].goto_absolute_position(position=0, speed=default_rotor_speed)
        # retract the arm
        self.axes['arm'].goto_absolute_position(position=0.3, speed=default_linear_speed)
        # execute
        self.parallel_run()

    def circular_motion(self, distance, radius, duration, step_frequency, start_angle, stop_angle):

        input("Press Enter to start circular motion...")

        # get arm, rotor, and pan ready for circular motion using speed mode
        self.axes['arm'].mode = "speed_mode"
        self.axes['arm'].ramp_type = "jerkfree"
        self.axes['arm'].jerk = 5

        self.axes['rotor'].mode = "speed_mode"
        self.axes['rotor'].ramp_type = "jerkfree"
        self.axes['rotor'].jerk = 5

        self.axes['pan'].mode = "speed_mode"
        self.axes['pan'].ramp_type = "jerkfree"
        self.axes['pan'].jerk = 5

        step_periode = 1.0 / step_frequency

        start_time = time.time()

        loop_count = 0
        t = 0

        k = (stop_angle - start_angle) / duration

        try:
            while t < duration:

                # alpha = k * t

                t = time.time() - start_time

                self.axes['arm'].signed_speed = circular_motion_arm_speed(t, k, start_angle, distance, radius)

                self.axes['rotor'].signed_speed = circular_motion_rotor_speed(t, k, start_angle, distance, radius)

                self.axes['pan'].signed_speed = circular_motion_pan_speed(t, k, start_angle, distance, radius)

                if loop_count == 0:
                    self.axes['arm'].run()
                    self.axes['rotor'].run()
                    self.axes['pan'].run()

                loop_count += 1

                # time.sleep(step_periode)

        except SoftLimitViolationException:
            print('Warning: Aborting movement due to soft limit violation.')
            self.emergency_stop()

        else:  # everything went smoothly
            self.axes['arm'].stop()
            self.axes['rotor'].stop()
            self.axes['pan'].stop()

            sleep_time = loop_count * step_periode

            print('Sleep time was ' + str(sleep_time))

    def front_linear_motion(self, distance, duration, step_frequency, start_s, stop_s):

        input("Press Enter to start linear motion...")

        # get arm, rotor, and pan ready for circular motion using speed mode
        self.axes['arm'].mode = "speed_mode"
        self.axes['arm'].ramp_type = "jerkfree"
        self.axes['arm'].jerk = 5

        self.axes['rotor'].mode = "speed_mode"
        self.axes['rotor'].ramp_type = "jerkfree"
        self.axes['rotor'].jerk = 5

        self.axes['pan'].mode = "speed_mode"
        self.axes['pan'].ramp_type = "jerkfree"
        self.axes['pan'].jerk = 5

        step_periode = 1.0 / step_frequency

        start_time = time.time()

        loop_count = 0
        t = 0

        k = (stop_s - start_s) / duration

        try:
            while t < duration:

                # alpha = k * t

                t = time.time() - start_time

                self.axes['arm'].signed_speed = front_linear_motion_arm_speed(t, k, start_s, distance)

                self.axes['rotor'].signed_speed = front_linear_motion_rotor_pan_speed(t, k, start_s, distance)

                self.axes['pan'].signed_speed = - front_linear_motion_rotor_pan_speed(t, k, start_s, distance)

                if loop_count == 0:
                    self.axes['arm'].run()
                    self.axes['rotor'].run()
                    self.axes['pan'].run()

                loop_count += 1

                # time.sleep(step_periode)

        except SoftLimitViolationException:
            print('Warning: Aborting movement due to soft limit violation.')
            self.emergency_stop()

        else:  # everything went smoothly
            self.axes['arm'].stop()
            self.axes['rotor'].stop()
            self.axes['pan'].stop()

            sleep_time = loop_count * step_periode

            print('Loop count was ' + str(loop_count))

    def move_to_front_linear_start_position(self, distance, start_s):
        # move arm to the starting point
        arm_start_position = front_linear_motion_arm_position(start_s, distance)
        self.axes['arm'].goto_absolute_position(position=arm_start_position, speed=default_linear_speed)
        # move rotor to starting point
        rotor_starting_angle = front_linear_motion_rotor_pan_angle(start_s, distance)
        self.axes['rotor'].goto_absolute_position(position=rotor_starting_angle, speed=default_rotor_speed)
        # move pan to starting point
        pan_starting_angle = - rotor_starting_angle
        self.axes['pan'].goto_absolute_position(position=pan_starting_angle, speed=default_pan_tilt_speed)
        # execute
        self.parallel_run()

    # Sequences

    def run_circular_sequence(self, distance, radius, duration, step_frequency, start_angle, stop_angle):

        self.move_to_start_angle(distance, radius, start_angle)

        time.sleep(0.2)

        self.circular_motion(distance, radius, duration, step_frequency, start_angle, stop_angle)

        time.sleep(0.2)

        self.back_up()

    def run_front_linear_sequence(self, distance, duration, step_frequency, start_s, stop_s):

        self.move_to_front_linear_start_position(distance, start_s)
        time.sleep(0.2)
        self.front_linear_motion(distance, duration, step_frequency, start_s, stop_s)
        time.sleep(0.2)
        self.go_neutral()

    def jog(self, axis, wheel, flag):

        current_position = axis.absolute_position

        near_limit = axis.near_absolute_limit
        far_limit = axis.far_absolute_limit

        wheel.counter = int(1000 * current_position)
        wheel.min = int(1000 * near_limit)
        wheel.max = int(1000 * far_limit)

        axis.mode = "speed_mode"
        axis.ramp_type = "jerkfree"
        axis.jerk = 20

        position_reached = True

        while flag.is_set():

            distance = round(float(wheel.counter) / 1000, 3) - round(axis.absolute_position, 3)

            if not (distance == 0):

                axis.signed_speed = distance

                if position_reached:  # was reached before, now new need for motion
                    axis.run()

                position_reached = False

            else:  # distance is 0
                if not position_reached:  # was moving before
                    axis.stop()
                    position_reached = True

                time.sleep(0.5)

        # if you are here, flag was cleared from outside
        axis.stop()


# create the one motion controller which will be imported by other modules
motion_controller = MotionController()
