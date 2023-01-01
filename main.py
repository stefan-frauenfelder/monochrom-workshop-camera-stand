
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

# def linear_ramp(LinearMotor, speed_step_frequency=10, cycle_duration=8, max_speed=0.1):

#     speed_step_periode = 1.0 / speed_step_frequency

#     num_speed_steps = int((cycle_duration / 4.0) / speed_step_periode)

#     horizontal_slider.run()

#     for half_cycle in range(2):

#         horizontal_slider.direction = half_cycle

#         for speed_step in range(num_speed_steps):
#             horizontal_slider.speed = max_speed * (speed_step / num_speed_steps)
#             time.sleep(speed_step_periode)

#         for speed_step in range(num_speed_steps, 0, -1):
#             horizontal_slider.speed = max_speed * (speed_step / num_speed_steps)
#             time.sleep(speed_step_periode)

#     horizontal_slider.stop()


# def circle_run(step_frequency=10, cycle_duration=8, max_speed=0.1):

#     step_periode = 1.0 / step_frequency

#     num_steps_for_cycle = int(cycle_duration / step_periode)

#     horizontal_slider.run()
#     vertical_slider.run()

#     for step in range(num_steps_for_cycle):

#         angle = step / num_steps_for_cycle * 2.0 * math.pi

#         horizontal_speed_scaling = math.sin(angle)
#         vertical_speed_scaling = math.cos(angle)

#         if horizontal_speed_scaling > 0:
#             horizontal_slider.direction = 0
#         else:
#             horizontal_slider.direction = 1
#             horizontal_speed_scaling = -horizontal_speed_scaling

#         horizontal_slider.speed = horizontal_speed_scaling * max_speed

#         if vertical_speed_scaling > 0:
#             vertical_slider.direction = 0
#         else:
#             vertical_slider.direction = 1
#             vertical_speed_scaling = -vertical_speed_scaling

#         vertical_slider.speed = vertical_speed_scaling * max_speed

#         time.sleep(step_periode)

#     horizontal_slider.stop()
#     vertical_slider.stop()


def flat_circle_run(distance=0.6, radius=0.1, duration=20, step_frequency=10):

    tilt.move(angle=-math.pi / 2, speed=6)
    tilt.blocking_run()

    lift.move(distance=-0.1, speed=0.03)
    lift.blocking_run()

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

    while t < duration:

        t = time.time() - start_time

        k = math.pi * 2 / duration

        alpha = k * t

        rotor.signed_speed = circular_motion_rotor_speed(t, k, distance, radius)

        arm.signed_speed = circular_motion_arm_speed(t, k, distance, radius)

        if first_loop:
            arm.run()
            rotor.run()
        first_loop = 0

        time.sleep(step_periode)

    arm.stop()
    rotor.stop()

    time.sleep(1)

    lift.move(distance=0.1, speed=0.03)
    lift.blocking_run()

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


def get_normalized_bit(value, bit_index):
    return (value >> bit_index) & 1


if __name__ == '__main__':

    try:

        print(sys.executable)
        print(sys.version)

        # create global thread-safe events for motion surveilance
        arm_motion = threading.Event()
        lift_motion = threading.Event()
        rotor_motion = threading.Event()
        pan_motion = threading.Event()
        tilt_motion = threading.Event()

        init()

        serial_port = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=1)

        commander_lock = threading.Lock()

        the_commander = Commander(ser=serial_port, lock=commander_lock)  # create the commander

        # # start motion surveilance in a separate thread
        # motion_surveilance_thread = threading.Thread(target=start_motion_surveilance)
        # motion_surveilance_thread.start()

        # opto_inputs = sequent_inputs.get_opto_all(bus=config.sm_bus, bus_lock=config.sm_bus_lock, stack=2)

        ports = sequent_ports.SequentPorts()

        opto_inputs = ports.read_inputs()

        ports.set_output(relay=2, switch_state=0)

        # # Horizontal axis
        # arm = LocatedLinearStepper(commander=the_commander,
        #                            motor_address=1,
        #                            name='arm',
        #                            power_relay=1,
        #                            motion_event=arm_motion,
        #                            inverse_direction=False,
        #                            safe_length=0.6,
        #                            position_offset=0.260)

        # # Vertical axis
        # lift = LocatedLinearStepper(commander=the_commander,
        #                             motor_address=2,
        #                             name='lift',
        #                             power_relay=2,
        #                             motion_event=lift_motion,
        #                             inverse_direction=True,
        #                             safe_length=0.6,
        #                             position_offset=0.740)

        # # Rotor
        # radians_per_motor_revolution_of_rotor = 2 * math.pi / 25  # this is the gear ratio of the worm drive

        # rotor = OrientedRotationalStepper(commander=the_commander,
        #                                   motor_address=3,
        #                                   name='rotor',
        #                                   power_relay=3,
        #                                   motion_event=rotor_motion,
        #                                   angle_per_motor_revolution=radians_per_motor_revolution_of_rotor)

        # rotor.ramp_type = "jerkfree"
        # rotor.jerk = 1

        # pan = OrientedRotationalStepper(commander=the_commander,
        #                                 motor_address=4,
        #                                 name='pan',
        #                                 power_relay=4,
        #                                 motion_event=pan_motion,
        #                                 inverse_direction=True,
        #                                 angle_per_motor_revolution=2 * math.pi)

        # pan.ramp_type = "jerkfree"
        # pan.jerk = 5

        # tilt = OrientedRotationalStepper(commander=the_commander,
        #                                  motor_address=5,
        #                                  name='tilt',
        #                                  power_relay=5,
        #                                  motion_event=tilt_motion,
        #                                  inverse_direction=True,
        #                                  angle_per_motor_revolution=2 * math.pi)

        # tilt.ramp_type = "jerkfree"
        # tilt.jerk = 5

        # # linear_axes_limit_run()
        # rotary_axes_origin_run()

        # flat_circle_run(distance=0.5, radius=0.1, duration=20, step_frequency=10)

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

        # # power down all motors
        # for i in range(1, 6):
        #     #switch_relay(config.sm_bus, i, 0)
        #     time.sleep(0.2)
        # print('Motors powered down.')

        # # GPIO.cleanup()  # this ensures a clean exit
        pass
        # print('IO ports cleaned up.')

# end of main
