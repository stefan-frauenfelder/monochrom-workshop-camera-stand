
import serial
import time
import sys
import math
import RPi.GPIO as GPIO
import threading

import sequent_ports

from bitstring import BitArray

from nanotec import *
from motion_control import *
from camera_stand_math import *


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


def homing_run():

    find_limits_arm_thread = threading.Thread(target=arm.find_linear_stepper_limits)
    find_limits_lift_thread = threading.Thread(target=lift.find_linear_stepper_limits)

    find_limits_arm_thread.start()
    find_limits_lift_thread.start()

    find_limits_arm_thread.join()
    find_limits_lift_thread.join()

    # rotor.find_origin(limit_switch='external', direction=Direction.negative)

    def find_pan_origin():
        pan.find_rotational_stepper_origin(limit_switch='internal', direction=Direction.negative)

    def find_tilt_origin():
        tilt.find_rotational_stepper_origin(limit_switch='internal', direction=Direction.positive)

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
        arm = LocatedStepper(commander=the_commander,
                             io_card=io_card,
                             motor_address=1,
                             name='arm',
                             power_relay=1,
                             steps_per_motor_revolution=200,
                             micro_steps_per_step=8,
                             si_unit_per_motor_revolution=0.12,
                             inverse_direction=False,
                             approx_safe_travel=0.6,
                             position_offset=0.258,
                             near_soft_limit_port=1,
                             far_soft_limit_port=0)

        # Vertical axis
        lift = LocatedStepper(commander=the_commander,
                              io_card=io_card,
                              motor_address=2,
                              name='lift',
                              power_relay=2,
                              steps_per_motor_revolution=200,
                              micro_steps_per_step=8,
                              si_unit_per_motor_revolution=0.12,
                              inverse_direction=True,
                              approx_safe_travel=0.6,
                              position_offset=0.812,
                              near_soft_limit_port=3,
                              far_soft_limit_port=2)

        # Rotor
        radians_per_motor_revolution_of_rotor = 2 * math.pi / 25  # this is the gear ratio of the worm drive

        rotor = OrientedStepper(commander=the_commander,
                                io_card=io_card,
                                motor_address=3,
                                name='rotor',
                                power_relay=3,
                                inverse_direction=True,
                                si_unit_per_motor_revolution=radians_per_motor_revolution_of_rotor)

        rotor.ramp_type = "jerkfree"
        rotor.jerk = 1

        pan = OrientedStepper(commander=the_commander,
                              io_card=io_card,
                              motor_address=4,
                              name='pan',
                              power_relay=4,
                              inverse_direction=False,
                              si_unit_per_motor_revolution=(2 * math.pi))

        tilt = OrientedStepper(commander=the_commander,
                               io_card=io_card,
                               motor_address=5,
                               name='tilt',
                               power_relay=5,
                               inverse_direction=False,
                               si_unit_per_motor_revolution=(2 * math.pi))

        time.sleep(2)

        homing_run()

        time.sleep(2)

        # flat_circle_run(travel=0.8, radius=0.1, duration=20, step_frequency=10, start_angle=0.5, stop_angle=2 * math.pi - 0.5)

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
