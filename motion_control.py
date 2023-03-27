import time
import math
import threading
import serial
import json
import pigpio

from nanotec import *
from motion_math import *

from hardware import hardware_manager

default_linear_speed = 0.1
default_rotor_speed = 0.1
default_pan_tilt_speed = 0.8


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
        # _axes are empty for now and are created upon user request
        self._axes = None

        self._wheel = hardware_manager.wheel

        # set up the hardware opto-isolated input and relay outputs cards
        self.io_card = hardware_manager.sequent_ports

        # create an event which manages a flag that allows to abort a running thread
        self._continuous_control_flag = threading.Event()
        self._continuous_control_thread_id = False

        self._jog_axis = None

        self._polar_joystick_axes_set = 'arm-lift-rotor'

        self.start_marker = {}
        self.target_marker = {}

        self.current_sequence_setup_marker = {}
        self.current_sequence_start_marker = {}
        self.current_sequence_target_marker = {}

        # parameters

        # the resulting total speed of the camera
        self.camera_speed = [0.1]

        # the total length of the target path the camera travels during the target motion, from start to stop, including acceleration distance
        self.total_target_path_length = [0.1]

        # the perpendicular distance of the camera path from the robot origin
        self.perpendicular_target_path_distance_from_origin = [0.5]

    def get_marker(self):
        marker = {}
        for key in self._axes:
            # unpack
            axis: LocatedStepper = self._axes[key]
            # store current position
            marker[key] = axis.absolute_position
        return marker

    def synchronized_move_to_marker(self, marker):
        """ perform a synchronized move from the current location to the marker """
        # get some empty dicts for later
        duration = {}
        distance = {}
        start_location = {}
        common_duration = 0

        # only a subset of axis may have to move, so go through keys in marker
        for key in marker:
            # unpack
            axis: LocatedStepper = self._axes[key]
            start_location[key] = axis.absolute_position
            # calculate distance and minimal duration
            distance[key] = marker[key] - start_location[key]
            duration[key] = abs(distance[key] / axis.maximum_speed)

        # find the longest duration
        for key in duration:
            common_duration = max(common_duration, duration[key])

        # calculate speeds based on common duration
        for key in marker:
            # unpack
            axis: LocatedStepper = self._axes[key]
            # calculate speed based on distance and common duration
            speed = distance[key] / common_duration
            # arm the axis
            axis.jerk = 5
            axis.goto_absolute_position(position=marker[key], speed=speed)

        # start all axes
        self.parallel_run()

    def reset_rotor_reference(self):
        rotor: LocatedStepper = self._axes['rotor']
        rotor.set_position_offset_to_current_position()

    def toggle_jog_axis(self):
        # stop jogging to switch axis
        self.stop_continuous_control_thread()
        if self._axes:
            if not self._jog_axis:
                self._jog_axis = self._axes['arm']
            elif self._jog_axis == self._axes['arm']:
                self._jog_axis = self._axes['lift']
            elif self._jog_axis == self._axes['lift']:
                self._jog_axis = self._axes['rotor']
            elif self._jog_axis == self._axes['rotor']:
                self._jog_axis = self._axes['pan']
            elif self._jog_axis == self._axes['pan']:
                self._jog_axis = self._axes['tilt']
            elif self._jog_axis == self._axes['tilt']:
                self._jog_axis = self._axes['arm']
            else:
                raise ValueError('Error setting jogging axes.')
        # restart jogging
        self.start_jog_control()

    def toggle_polar_joystick_axes_set(self):
        # stop joystick to switch axes set
        self.stop_continuous_control_thread()
        # switch axes set
        if self._polar_joystick_axes_set == 'arm-lift-rotor':
            self._polar_joystick_axes_set = 'pan-tilt'
        else:
            self._polar_joystick_axes_set = 'arm-lift-rotor'
        # start again
        self.start_polar_joystick_control()

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

        self._axes = {
            'arm': arm,
            'lift': lift,
            'rotor': rotor,
            'pan': pan,
            'tilt': tilt
        }

        # power up the steppers
        for axis in self._axes.values():
            # power up the individual _axes with a slight delay to avoid power drop
            axis.power_up()
            time.sleep(0.2)

        # initialize the steppers with a default configuration
        for axis in self._axes.values():
            axis.initialize()

        print('All motors powered up and initialized.')

        # motion_controller.run_front_linear_sequence(distance=0.6, duration=30, step_frequency=10, start_s=0.3, stop_s=-0.3)

        # motion_controller.run_circular_sequence(distance=0.7, radius=0.3, duration=30, step_frequency=10, start_angle=1, stop_angle=2 * math.gpio - 1)

    def homing_run(self):
        # define individual threads for all axis to run homing in parallel
        find_limits_arm_thread = threading.Thread(target=self._axes['arm'].find_linear_stepper_limits)
        find_limits_lift_thread = threading.Thread(target=self._axes['lift'].find_linear_stepper_limits)
        # the lambda prevents the function from being interpreted here already. Don't ask me how. I have no clue.
        find_limits_rotor_thread = threading.Thread(target=lambda: self._axes['rotor'].find_rotor_origin(limit_switch='external', direction=Direction.negative))
        find_origin_pan_thread = threading.Thread(target=lambda: self._axes['pan'].find_rotational_stepper_origin(limit_switch='internal', direction=Direction.negative))
        find_origin_tilt_thread = threading.Thread(target=lambda: self._axes['tilt'].find_rotational_stepper_origin(limit_switch='internal', direction=Direction.positive))
        # find limits of major _axes
        find_limits_arm_thread.start()
        find_limits_lift_thread.start()
        find_limits_rotor_thread.start()
        # wait for completion
        find_limits_arm_thread.join()
        find_limits_lift_thread.join()
        find_limits_rotor_thread.join()
        # find limits of minor _axes
        find_origin_pan_thread.start()
        find_origin_tilt_thread.start()
        # wait for completion
        find_origin_pan_thread.join()
        find_origin_tilt_thread.join()
        # set fake limits because there are no limit switches yet
        self._axes['pan'].set_fake_rotational_stepper_limits()
        self._axes['tilt'].set_fake_rotational_stepper_limits()
        self._axes['rotor'].set_fake_rotational_stepper_limits()

    def activate_all_steppers_hardware_analog_joystick_mode(self):
        # limit speeds for joystick mode
        self._axes['arm'].speed = 0.05
        self._axes['lift'].speed = 0.05
        self._axes['rotor'].speed = 0.1
        self._axes['pan'].speed = 0.1
        self._axes['tilt'].speed = 0.1

        for axis in self._axes.values():
            axis.mode = 'joystick_mode'
            axis.run()

        input("Press Enter to stop and continue.")

        for axis in self._axes.values():
            axis.stop()
            axis.mode = 'relative_positioning'

    def shutdown(self):
        # power down all motors
        for axis in self._axes.values():
            axis.shutdown()
            time.sleep(0.1)
        print('Steppers powered down.')

    def start_jog_control(self):
        print('Starting jog control')
        # the first time jogging is started, an axis needs to be set
        if not self._jog_axis:
            self._jog_axis = self._axes['arm']
        self.start_continuous_control_thread(self.jog)

    def start_polar_joystick_control(self):
        print('Starting polar joystick control')
        self.start_continuous_control_thread(self.polar_joystick_control)

    def start_rectilinear_joystick_control(self):
        print('Starting rectilinear joystick control')
        self.start_continuous_control_thread(self.rectilinear_joystick_control)

    def start_continuous_control_thread(self, target):
        print('Starting continuous control thread')
        # set the flag to true. It is checked in the while loop of every continuous control target function
        self._continuous_control_flag.set()
        # make sure there is no active thread
        if self._continuous_control_thread_id:
            if self._continuous_control_thread_id.is_alive():
                raise RuntimeError('Another continuous thread is still alive.')
        # start a new thread
        self._continuous_control_thread_id = threading.Thread(target=lambda: target(flag=self._continuous_control_flag))
        self._continuous_control_thread_id.start()

    def stop_continuous_control_thread(self):
        print('Stopping continuous control thread')
        # clearing the flag will cause the previously started thread to return
        self._continuous_control_flag.clear()
        self._continuous_control_thread_id.join(timeout=5)

    def disarm_all(self):
        # iterate through _axes
        for axis in self._axes.values():
            axis.armed = False

    def emergency_stop(self):
        # iterate through _axes
        for axis in self._axes.values():
            # immediately stop axis
            axis.immediate_stop()
        # stop any continuous control thread
        self.stop_continuous_control_thread()

    def emergency_shutdown(self):
        # power down all motors
        for axis in self._axes.values():
            axis.shutdown()
        print('Steppers powered down.')
        # stop any continuous control thread
        self.stop_continuous_control_thread()

    def parallel_run(self):
        # create a list of threads
        threads = []
        # iterate through _axes
        for axis in self._axes.values():
            # check if it was configured for a run (travel was set but run was not called)
            if axis.armed:
                # create a new (blocking) thread, add it to the list, and start it
                thread = threading.Thread(target=axis.blocking_run)
                threads.append(thread)
                thread.start()
        # join all threads
        for thread in threads:
            thread.join()

    def move_to_start_angle(self, distance, radius, start_angle):

        # lift the pen from the paper
        self._axes['lift'].goto_absolute_position(position=1.2, speed=default_linear_speed)
        self._axes['arm'].goto_absolute_position(position=0.4, speed=default_linear_speed)
        self.parallel_run()

        input("Press Enter to arm motors...")

        # move arm to the starting point
        arm_start_position = circular_motion_arm_position(start_angle, distance, radius)
        self._axes['arm'].goto_absolute_position(position=arm_start_position, speed=default_linear_speed)
        # move rotor to starting point
        rotor_starting_angle = circular_motion_rotor_angle(start_angle, distance, radius)
        self._axes['rotor'].goto_absolute_position(position=rotor_starting_angle, speed=default_rotor_speed)
        # move pan to starting point
        pan_starting_angle = circular_motion_pan_angle(start_angle, distance, radius)
        self._axes['pan'].goto_absolute_position(position=pan_starting_angle, speed=default_pan_tilt_speed)
        # move the lift to above target height
        self._axes['lift'].goto_absolute_position(position=0.9, speed=default_linear_speed)
        # point the pen down
        self._axes['tilt'].goto_absolute_position(position=-0.5, speed=default_pan_tilt_speed)

        input("Press Enter to start parallel run...")
        self.parallel_run()

        # lower the pen to the paper
        self._axes['lift'].goto_absolute_position(position=0.851, speed=default_linear_speed)
        self._axes['lift'].blocking_run()

    def circular_motion(self, distance, radius, duration, step_frequency, start_angle, stop_angle):

        input("Press Enter to start circular motion...")

        # get arm, rotor, and pan ready for circular motion using speed mode
        self._axes['arm'].mode = "speed_mode"
        self._axes['arm'].ramp_type = "jerkfree"
        self._axes['arm'].jerk = 5

        self._axes['rotor'].mode = "speed_mode"
        self._axes['rotor'].ramp_type = "jerkfree"
        self._axes['rotor'].jerk = 5

        self._axes['pan'].mode = "speed_mode"
        self._axes['pan'].ramp_type = "jerkfree"
        self._axes['pan'].jerk = 5

        step_periode = 1.0 / step_frequency

        start_time = time.time()

        loop_count = 0
        t = 0

        k = (stop_angle - start_angle) / duration

        try:
            while t < duration:

                # alpha = k * t

                t = time.time() - start_time

                self._axes['arm'].signed_speed = circular_motion_arm_speed(t, k, start_angle, distance, radius)

                self._axes['rotor'].signed_speed = circular_motion_rotor_speed(t, k, start_angle, distance, radius)

                self._axes['pan'].signed_speed = circular_motion_pan_speed(t, k, start_angle, distance, radius)

                if loop_count == 0:
                    self._axes['arm'].run()
                    self._axes['rotor'].run()
                    self._axes['pan'].run()

                loop_count += 1

                # time.sleep(step_periode)

        except SoftLimitViolationException:
            print('Warning: Aborting movement due to soft limit violation.')
            self.emergency_stop()

        else:  # everything went smoothly
            self._axes['arm'].stop()
            self._axes['rotor'].stop()
            self._axes['pan'].stop()

            sleep_time = loop_count * step_periode

            print('Sleep time was ' + str(sleep_time))

    def speed_scaling(self, nominal_speed, t, duration, acceleration=0.05):
        if nominal_speed < 0:
            acceleration = -acceleration
        acceleration_duration = nominal_speed / acceleration
        if t < acceleration_duration:  # during acceleration
            return acceleration * t
        else:  # during steady speed
            return nominal_speed

    # front linear motion methods

    def move_to_front_linear_start_position(self):

        # the current pan angle defines the axis of perpendicularity and is used to 'zero' the rotor
        deviation_angle = self._axes['pan'].absolute_position
        self._axes['rotor'].absolute_position = - deviation_angle

        # using the new rotor origin, store the marker of the setup position
        self.current_sequence_setup_marker = self.get_marker()

        # calculate the perpendicular distance of the camera from the origin
        arm_extension = self._axes['arm'].absolute_position
        distance, current_deflection = rectilinear_camera_coordinates(- deviation_angle, arm_extension)
        # store for the target move
        self.perpendicular_target_path_distance_from_origin[0] = distance

        # calculate the start position using distance and the given path length
        deflection_from_center = self.total_target_path_length[0] / 2.0
        arm_start_position = front_linear_motion_arm_position(deflection_from_center, distance)
        rotor_starting_angle = front_linear_motion_rotor_pan_angle(deflection_from_center, distance)

        # store this in the start marker
        self.current_sequence_start_marker = {'arm': arm_start_position,
                                              'rotor': rotor_starting_angle,
                                              'pan': - rotor_starting_angle}

        # move
        self.synchronized_move_to_marker(self.current_sequence_start_marker)

    def front_linear_motion(self):

        nominal_speed = self.camera_speed[0]

        axes = [self._axes['arm'], self._axes['rotor'], self._axes['pan']]
        distance = self.perpendicular_target_path_distance_from_origin[0]
        path_length = self.total_target_path_length[0]

        nominal_duration = abs(path_length / nominal_speed)
        # initial deflection is positive, move forward is from left to right
        initial_deflection = path_length / 2.0

        for axis in axes:
            axis.mode = "speed_mode"
            axis.ramp_type = "jerkfree"
            axis.jerk = 20

        start_time = time.time()
        stopped = True
        t = 0
        # speed needs to be negative for move from left to right
        nominal_speed = - path_length / nominal_duration

        # acceleration for k-scaling
        acceleration = 0.05

        # calculate how much longer the move will take to reach the same distance with k-scaling (lower avg speed)
        acceleration_duration = abs(nominal_speed / acceleration)
        duration = nominal_duration + acceleration_duration / 2.0

        try:
            while t < duration:

                t = time.time() - start_time
                speed = self.speed_scaling(nominal_speed, t, duration, acceleration)
                self._axes['arm'].signed_speed = front_linear_motion_arm_speed(t, speed, initial_deflection, distance)

                t = time.time() - start_time
                speed = self.speed_scaling(nominal_speed, t, duration, acceleration)
                self._axes['rotor'].signed_speed = front_linear_motion_rotor_pan_speed(t, speed, initial_deflection, distance)

                t = time.time() - start_time
                speed = self.speed_scaling(nominal_speed, t, duration, acceleration)
                self._axes['pan'].signed_speed = - front_linear_motion_rotor_pan_speed(t, speed, initial_deflection, distance)

                if stopped:
                    for axis in axes:
                        axis.run()
                    stopped = False

        except SoftLimitViolationException:
            print('Warning: Aborting movement due to soft limit violation.')
            self.emergency_stop()

        else:  # everything went smoothly
            for axis in axes:
                axis.stop()

    # orbiter motion methods

    def jog(self, flag):
        # preferences
        jog_wheel_scaling = 200

        axis = self._jog_axis

        current_position = axis.absolute_position

        near_limit = axis.near_absolute_limit
        far_limit = axis.far_absolute_limit

        self._wheel.counter = int(jog_wheel_scaling * current_position)
        self._wheel.min = int(jog_wheel_scaling * near_limit)
        self._wheel.max = int(jog_wheel_scaling * far_limit)

        axis.mode = "speed_mode"
        axis.ramp_type = "jerkfree"
        axis.jerk = 20

        position_reached = True

        while flag.is_set():

            distance = round(float(self._wheel.counter) / jog_wheel_scaling, 3) - round(axis.absolute_position, 3)

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

    def polar_joystick_control(self, flag):

        if self._polar_joystick_axes_set == 'arm-lift-rotor':
            axes = [self._axes['arm'], self._axes['lift'], self._axes['rotor']]
        elif self._polar_joystick_axes_set == 'pan-tilt':
            axes = [self._axes['pan'], self._axes['tilt']]
        else:
            raise ValueError('Not a valid axes set. Valid is arm-lift-rotor or pan-tilt.')

        for axis in axes:
            axis.mode = "speed_mode"
            axis.ramp_type = "jerkfree"
            axis.jerk = 20

        # every axis needs its own stopped flag
        stopped = []
        for i in range(len(axes)):
            stopped.append(True)

        while flag.is_set():
            # read the joystick
            position_tuple = hardware_manager.joystick.get_position()
            joystick_position = [position_tuple[1], position_tuple[0], - position_tuple[2]]

            for i in range(len(axes)):
                if abs(joystick_position[i]) > 0:  # joystick position demands movement
                    # set speed from joystick position
                    axes[i].limited_speed = joystick_position[i] * axes[i].joystick_speed

                    if stopped[i]:  # was stopped before
                        axes[i].run()  # start again
                        stopped[i] = False
                else:  # joystick position is neutral
                    if not stopped[i]:  # was moving before
                        axes[i].stop()  # stop
                        stopped[i] = True

        # if you are here, flag was cleared from outside
        for axis in axes:
            axis.stop()

    def rectilinear_joystick_control(self, flag):

        axes = [self._axes['arm'], self._axes['rotor'], self._axes['pan']]

        for axis in axes:
            axis.mode = "speed_mode"
            axis.ramp_type = "jerkfree"
            axis.jerk = 20

        stopped = True

        while flag.is_set():

            try:
                # read the joystick
                position_tuple = hardware_manager.joystick.get_position()
                joystick_speed_setting = position_tuple[1]

                if abs(joystick_speed_setting) > 0:  # joystick position demands movement

                    # a new period of movement starts
                    start_time = time.time()
                    rotor_angle = self._axes['rotor'].absolute_position
                    arm_extension = self._axes['arm'].absolute_position
                    (distance, start_s) = rectilinear_camera_coordinates(rotor_angle, arm_extension)

                    while abs(joystick_speed_setting) > 0 & flag.is_set():  # joystick position demands movement

                        # update the joystick value inside the while loop
                        position_tuple = hardware_manager.joystick.get_position()
                        joystick_speed_setting = position_tuple[1]

                        # k is the rectilinear speed of the camera in m/s
                        k = joystick_speed_setting / 10

                        t = time.time() - start_time

                        self._axes['arm'].limited_speed = front_linear_motion_arm_speed(t, k, start_s, distance)
                        self._axes['rotor'].limited_speed = front_linear_motion_rotor_pan_speed(t, k, start_s, distance)
                        self._axes['pan'].limited_speed = - front_linear_motion_rotor_pan_speed(t, k, start_s, distance)

                        if stopped:
                            # start all axes
                            for axis in axes:
                                axis.run()
                            stopped = False

                else:  # joystick position is neutral
                    if not stopped:
                        # stop all axes
                        for axis in axes:
                            axis.stop()
                        stopped = True

            except SoftLimitViolationException:
                print('Warning: Aborting movement due to soft limit violation.')
                self.emergency_stop()

        # if you are here, flag was cleared from outside
        for axis in axes:
            axis.stop()


# create the one motion controller which will be imported by other modules
motion_controller = MotionController()
