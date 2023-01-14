
import serial
import time
import sys
import math
import threading
import json
import pigpio

import sequent_ports

from nanotec import *
from motion_control import *

from rotary import Rotary

pi = math.pi

gpio = pigpio.pi()

# New rotary encoder stuff


def rotary_callback(counter):
    print("Counter value: ", counter)


def init():

    # GPIO.setwarnings(True)
    # GPIO.setmode(GPIO.BCM)

    # # Rotary encoder inputs
    # GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode)

    return


# def rotation_decode(Enc_A):
#     global counter
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

    def find_rotor_origin():
        rotor.find_rotor_origin(limit_switch='external', direction=Direction.negative)

    find_limits_arm_thread = threading.Thread(target=arm.find_linear_stepper_limits)
    find_limits_lift_thread = threading.Thread(target=lift.find_linear_stepper_limits)
    find_limits_rotor_thread = threading.Thread(target=find_rotor_origin)

    find_limits_arm_thread.start()
    find_limits_lift_thread.start()
    find_limits_rotor_thread.start()

    find_limits_arm_thread.join()
    find_limits_lift_thread.join()
    find_limits_rotor_thread.join()

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


if __name__ == '__main__':

    SEQUENT_INTN_GPIO = 5

    # Rotary encoder inputs
    # Enc_A = 23
    # Enc_B = 24

    init()

    counter = 10

    wheel = Rotary(clk_gpio=23, dt_gpio=24, sw_gpio=17)

    wheel.setup_rotary(
        min=0,
        max=1000,
        scale=1,
        debounce=0,
        rotary_callback=rotary_callback
    )

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

        commander = Commander(ser=serial_port, lock=commander_lock)  # create the commander

        # create all the stepper instances using the stepper configuration files

        # Horizontal axis
        arm = LocatedStepper(commander, io_card, json.loads(open("arm_config.json").read()))

        # Vertical axis
        lift = LocatedStepper(commander, io_card, json.loads(open("lift_config.json").read()))

        # rotation of the arm
        rotor = LocatedStepper(commander, io_card, json.loads(open("rotor_config.json").read()))

        # paning of the camera
        pan = LocatedStepper(commander, io_card, json.loads(open("pan_config.json").read()))

        # tilting of the camera
        tilt = LocatedStepper(commander, io_card, json.loads(open("tilt_config.json").read()))

        print('All motors powered up and initialized.')

        time.sleep(2)

        axes_dict = {
            'arm': arm,
            'lift': lift,
            'rotor': rotor,
            'pan': pan,
            'tilt': tilt
        }

        input("Press Enter to activate joystick mode...")

        activate_joystick_mode(axes_dict)

        input("Press Enter to start the homing sequence.")

        homing_run()

        motion_controller = MotionController(axes_dict)

        motion_controller.jog_mode(axis_name='arm', wheel=wheel)

        # motion_controller.run_front_linear_sequence(distance=0.6, duration=30, step_frequency=10, start_s=0.3, stop_s=-0.3)

        # motion_controller.run_circular_sequence(distance=0.7, radius=0.3, duration=30, step_frequency=10, start_angle=1, stop_angle=2 * math.pi - 1)

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
            time.sleep(0.1)
        print('Motors powered down.')

        # GPIO.cleanup()  # this ensures a clean exit
        print('IO ports cleaned up.')

# end of main
