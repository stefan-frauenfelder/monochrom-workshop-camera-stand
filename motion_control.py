
import time
import math
import threading

from nanotec import *
from motion_math import *

default_linear_speed = 0.1
default_rotor_speed = 0.1
default_pan_tilt_speed = 0.8


class MotionController():

    def __init__(self, axes_dict):

        self.axes = axes_dict
        # better safe than sorry
        self.disarm_all()

    def disarm_all(self):
        # iterate through axes
        for axis in self.axes.values():
            axis.armed = False

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

        first_loop = 1
        t = 0

        k = (stop_angle - start_angle) / duration

        while t < duration:

            # alpha = k * t

            t = time.time() - start_time

            self.axes['arm'].signed_speed = circular_motion_arm_speed(t, k, start_angle, distance, radius)

            self.axes['rotor'].signed_speed = circular_motion_rotor_speed(t, k, start_angle, distance, radius)

            self.axes['pan'].signed_speed = circular_motion_pan_speed(t, k, start_angle, distance, radius)

            if first_loop:
                self.axes['arm'].run()
                self.axes['rotor'].run()
                self.axes['pan'].run()
            first_loop = 0

            time.sleep(step_periode)

        self.axes['arm'].stop()
        self.axes['rotor'].stop()
        self.axes['pan'].stop()

    def run_circular_sequence(self, distance, radius, duration, step_frequency, start_angle, stop_angle):

        self.move_to_start_angle(distance, radius, start_angle)

        time.sleep(0.2)

        self.circular_motion(distance, radius, duration, step_frequency, start_angle, stop_angle)

        time.sleep(0.2)

        self.back_up()
