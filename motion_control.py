
import time
import math
import threading

from nanotec import *
from camera_stand_math import *

default_linear_speed = 0.05
default_rotor_speed = 0.1


def flat_circle_run(distance=0.8, radius=0.1, duration=20, step_frequency=10, start_angle=0, stop_angle=2 * math.pi):

    input("Press Enter to continue...")

    # move arm to the starting point
    arm_start_position = circular_motion_arm_position(start_angle, distance, radius)
    print('Arm start position: ' + str(arm_start_position))
    arm.goto_absolute_position(position=arm_start_position, speed=default_linear_speed)
    arm.blocking_run()

    input("Press Enter to continue...")

    # move rotor to starting point
    rotor_starting_angle = circular_motion_rotor_angle(start_angle, distance, radius)
    rotor.move(rotor_starting_angle, default_rotor_speed)
    rotor.blocking_run()

    input("Press Enter to continue...")

    # move the lift to above target height
    lift.goto_absolute_position(position=1.0, speed=default_linear_speed)
    lift.blocking_run()

    input("Press Enter to continue...")

    # point the pen down
    tilt.move(angle=-math.pi / 2, speed=6)
    tilt.blocking_run()

    input("Press Enter to continue...")

    # lower the pen to the paper
    lift.goto_absolute_position(position=0.82, speed=default_linear_speed)
    lift.blocking_run()

    input("Press Enter to continue...")

    # get arm and rotor ready for circular motion using speed mode
    arm.mode = "speed_mode"
    arm.ramp_type = "jerkfree"
    arm.jerk = 5

    rotor.mode = "speed_mode"
    rotor.ramp_type = "jerkfree"
    rotor.jerk = 5

    step_periode = 1.0 / step_frequency

    start_time = time.time()

    first_loop = 1
    t = 0

    k = (stop_angle - start_angle) / duration

    while t < duration:

        # alpha = k * t

        t = time.time() - start_time

        rotor.signed_speed = circular_motion_rotor_speed(t, k, start_angle, distance, radius)

        arm.signed_speed = circular_motion_arm_speed(t, k, start_angle, distance, radius)

        if first_loop:
            arm.run()
            rotor.run()
        first_loop = 0

        time.sleep(step_periode)

    arm.stop()
    rotor.stop()

    time.sleep(1)

    input("Press Enter to continue...")

    # lift the pen from the paper
    lift.goto_absolute_position(position=1.0, speed=default_linear_speed)
    lift.blocking_run()

    input("Press Enter to continue...")

    # raise the pen straight
    tilt.move(angle=math.pi / 2, speed=6)
    tilt.blocking_run()


def move_to_start_angle(distance, radius, start_angle):

    # move arm to the starting point
    arm_start_position = circular_motion_arm_position(start_angle, distance, radius)
    arm.goto_absolute_position(position=arm_start_position, speed=default_linear_speed)
    arm.blocking_run()

    input("Press Enter to continue...")

    # move rotor to starting point
    rotor_starting_angle = circular_motion_rotor_angle(start_angle, distance, radius)
    rotor.move(rotor_starting_angle, default_rotor_speed)
    rotor.blocking_run()


def circular_motion(distance, radius, duration, step_frequency, start_angle, stop_angle):

    # get arm, rotor, and pan ready for circular motion using speed mode
    arm.mode = "speed_mode"
    arm.ramp_type = "jerkfree"
    arm.jerk = 5

    rotor.mode = "speed_mode"
    rotor.ramp_type = "jerkfree"
    rotor.jerk = 5

    pan.mode = "speed_mode"
    pan.ramp_type = "jerkfree"
    pan.jerk = 5

    step_periode = 1.0 / step_frequency

    start_time = time.time()

    first_loop = 1
    t = 0

    k = (stop_angle - start_angle) / duration

    while t < duration:

        # alpha = k * t

        t = time.time() - start_time

        arm.signed_speed = circular_motion_arm_speed(t, k, start_angle, distance, radius)

        rotor.signed_speed = circular_motion_rotor_speed(t, k, start_angle, distance, radius)

        pan.signed_speed = circular_motion_pan_speed(t, k, start_angle, distance, radius)

        if first_loop:
            arm.run()
            rotor.run()
            pan.run()
        first_loop = 0

        time.sleep(step_periode)

    arm.stop()
    rotor.stop()
    pan.stop()
