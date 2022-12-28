

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


def flat_circle_run(distance=0.5, radius=0.05, duration=20, step_frequency=10):

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

        alpha = math.pi * 2 * t / duration

        angular_speed_value = angular_speed(alpha, distance, radius)

        arm.signed_speed = arm_extension_speed(alpha, distance, radius)

        if angular_speed_value > 0:
            rotor.step_direction = 0
        else:
            rotor.step_direction = 1
            angular_speed_value = -angular_speed_value

        rotor.speed = angular_speed_value

        if first_loop:
            arm.run()
            rotor.run()
        first_loop = 0

        time.sleep(step_periode)

    arm.stop()
    rotor.stop()


def init():

    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    # Rotary encoder inputs
    GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.setup(Stepper_power, GPIO.OUT)

    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode)

    return


def rotation_decode(Enc_A):
    global counter
    # time.sleep(0.002)
    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)

    if (Switch_A == 1) and (Switch_B == 0):
        counter += 1
        print("direction -> ", counter)
        while Switch_B == 0:
            Switch_B = GPIO.input(Enc_B)
        while Switch_B == 1:
            Switch_B = GPIO.input(Enc_B)
        return

    elif (Switch_A == 1) and (Switch_B == 1):
        counter -= 1
        print("direction <- ", counter)
        while Switch_A == 1:
            Switch_A = GPIO.input(Enc_A)
        return
    else:
        return


if __name__ == '__main__':

    import serial
    import time
    import sys
    import math
    import RPi.GPIO as GPIO
    import threading

    from nanotec import *
    from camera_stand_math import *

    try:
        print(sys.executable)
        print(sys.version)

        # Rotary encoder inputs
        Enc_A = 23
        Enc_B = 24

        # Optoisolated GPIO inputs for optical limit switches
        Arm_near_soft_limit = 18
        Arm_far_soft_limit = 17
        Lift_far_soft_limit = 19
        Lift_near_soft_limit = 20

        # power supply for stepper motors
        Stepper_power = 21

        init()

        GPIO.output(Stepper_power, GPIO.HIGH)  # turn on the power for the steppers

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
                                   motor_address=1,
                                   name='arm',
                                   inverse_direction=False,
                                   safe_length=0.6,
                                   near_soft_limit_gpio=Arm_near_soft_limit,
                                   far_soft_limit_gpio=Arm_far_soft_limit,
                                   position_offset=0.260)

        # Vertical axis
        lift = LocatedLinearStepper(commander=the_commander,
                                    motor_address=2,
                                    name='lift',
                                    inverse_direction=True,
                                    safe_length=0.6,
                                    near_soft_limit_gpio=Lift_near_soft_limit,
                                    far_soft_limit_gpio=Lift_far_soft_limit,
                                    position_offset=0.740)

        arm.ramp_type = "jerkfree"
        arm.jerk = 5

        lift.ramp_type = "jerkfree"
        lift.jerk = 5

        print("Finding limits of all axes...")

        find_limits_arm_thread = threading.Thread(target=arm.find_limits)
        # find_limits_lift_thread = threading.Thread(target=lift.find_limits)

        find_limits_arm_thread.start()
        # find_limits_lift_thread.start()

        find_limits_arm_thread.join()
        # find_limits_lift_thread.join()

        print("Limits of all axes set.")

        # # 0.02m stopping distance @ jerk 5 and speed 0.1m/s
        # # 0.07m stopping distance @ jerk 5 and speed 0.2m/s

        print("All done.")

        arm.goto_absolute_position(position=0.4, speed=0.1)

        arm.run()
        while not arm.is_ready:
            time.sleep(0.5)
        arm.stop()

        # Rotor

        radians_per_motor_revolution = 2 * math.pi / 25  # this is the gear ratio of the worm drive

        rotor = RotationStepper(commander=the_commander,
                                motor_address=3,
                                name='rotor',
                                angle_per_motor_revolution=radians_per_motor_revolution)

        rotor.ramp_type = "jerkfree"
        rotor.jerk = 1

        flat_circle_run()

        GPIO.output(Stepper_power, GPIO.LOW)  # turn off the power for the steppers

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
        GPIO.cleanup()  # this ensures a clean exit
        print('IO ports are cleaned up.')

# end of main
