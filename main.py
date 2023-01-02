
import serial
import time
import sys
import math
import RPi.GPIO as GPIO
import threading

import sequent_ports

from bitstring import BitArray

from nanotec import *
from camera_stand_math import *


def flat_circle_run(distance=0.8, radius=0.1, duration=20, step_frequency=10, start_angle=0, stop_angle=2 * math.pi):

    default_linear_speed = 0.05
    default_rotor_speed = 0.1

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


def init():

    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    # # Rotary encoder inputs
    # GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode)

    return


# def rotation_decode(Enc_A):
#     global counter
#     # time.sleep(0.002)
#     Switch_A = GPIO.input(Enc_A)
#     Switch_B = GPIO.input(Enc_B)

#     if (Switch_A == 1) and (Switch_B == 0):
#         counter += 1
#         print("direction -> ", counter)
#         while Switch_B == 0:
#             Switch_B = GPIO.input(Enc_B)
#         while Switch_B == 1:
#             Switch_B = GPIO.input(Enc_B)
#         return

#     elif (Switch_A == 1) and (Switch_B == 1):
#         counter -= 1
#         print("direction <- ", counter)
#         while Switch_A == 1:
#             Switch_A = GPIO.input(Enc_A)
#         return
#     else:
#         return


def linear_axes_limit_run():

    arm.ramp_type = "jerkfree"
    arm.jerk = 5

    lift.ramp_type = "jerkfree"
    lift.jerk = 5

    print("Finding limits of all axes...")

    find_limits_arm_thread = threading.Thread(target=arm.find_limits)
    find_limits_lift_thread = threading.Thread(target=lift.find_limits)

    find_limits_arm_thread.start()
    find_limits_lift_thread.start()

    find_limits_arm_thread.join()
    find_limits_lift_thread.join()

    print("Limits of all axes set.")


def rotary_axes_origin_run():

    # rotor.find_origin(limit_switch='external', direction=RotationalDirection.cw)

    def find_pan_origin():
        pan.find_origin(direction=RotationalDirection.cw)

    def find_tilt_origin():
        tilt.find_origin(direction=RotationalDirection.ccw)

    find_origin_pan_thread = threading.Thread(target=find_pan_origin)
    find_origin_tilt_thread = threading.Thread(target=find_tilt_origin)

    find_origin_pan_thread.start()
    find_origin_tilt_thread.start()

    find_origin_pan_thread.join()
    find_origin_tilt_thread.join()


if __name__ == '__main__':

    init()

    SEQUENT_INTN_GPIO = 5

    io_card = sequent_ports.SequentPorts(SEQUENT_INTN_GPIO)

    try:

        print(sys.executable)
        print(sys.version)

        serial_port = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=1)

        commander_lock = threading.Lock()

        the_commander = Commander(ser=serial_port, lock=commander_lock)  # create the commander

        # Horizontal axis
        arm = LocatedLinearStepper(commander=the_commander,
                                   io_card=io_card,
                                   motor_address=1,
                                   name='arm',
                                   power_relay=1,
                                   inverse_direction=False,
                                   safe_length=0.6,
                                   position_offset=0.258,
                                   near_soft_limit_port=1,
                                   far_soft_limit_port=0)

        # Vertical axis
        lift = LocatedLinearStepper(commander=the_commander,
                                    io_card=io_card,
                                    motor_address=2,
                                    name='lift',
                                    power_relay=2,
                                    inverse_direction=True,
                                    safe_length=0.6,
                                    position_offset=0.790,
                                    near_soft_limit_port=3,
                                    far_soft_limit_port=2)

        # Rotor
        radians_per_motor_revolution_of_rotor = 2 * math.pi / 25  # this is the gear ratio of the worm drive

        rotor = OrientedRotationalStepper(commander=the_commander,
                                          io_card=io_card,
                                          motor_address=3,
                                          name='rotor',
                                          power_relay=3,
                                          angle_per_motor_revolution=radians_per_motor_revolution_of_rotor)

        rotor.ramp_type = "jerkfree"
        rotor.jerk = 1

        pan = OrientedRotationalStepper(commander=the_commander,
                                        io_card=io_card,
                                        motor_address=4,
                                        name='pan',
                                        power_relay=4,
                                        inverse_direction=True,
                                        angle_per_motor_revolution=2 * math.pi)

        pan.ramp_type = "jerkfree"
        pan.jerk = 5

        tilt = OrientedRotationalStepper(commander=the_commander,
                                         io_card=io_card,
                                         motor_address=5,
                                         name='tilt',
                                         power_relay=5,
                                         inverse_direction=True,
                                         angle_per_motor_revolution=2 * math.pi)

        tilt.ramp_type = "jerkfree"
        tilt.jerk = 5

        # rotor.find_origin(limit_switch='external', direction=RotationalDirection.cw)

        time.sleep(2)

        linear_axes_limit_run()
        rotary_axes_origin_run()

        time.sleep(2)

        flat_circle_run(distance=0.8, radius=0.1, duration=20, step_frequency=10, start_angle=0.5, stop_angle=2 * math.pi - 0.5)

        time.sleep(2)

    except KeyboardInterrupt:
        # here you put any code you want to run before the program
        # exits when you press CTRL+C
        print('Exiting upon KeyboardInterrupt.')

    # except:
        # this catches ALL other exceptions including errors.
        # You won't get any error messages for debugging
        # so only use it once your code is working
        # print('Exiting upon error.')

    finally:

        # power down all motors
        for i in range(1, 6):
            io_card.set_output(i, 0)
            time.sleep(0.2)
        print('Motors powered down.')

        GPIO.cleanup()  # this ensures a clean exit
        print('IO ports cleaned up.')

# end of main
