

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


def flat_circle_run(distance=0.5, radius=0.02, duration=20, step_frequency=10):

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
            rotor.direction = 0
        else:
            rotor.direction = 1
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

    # Optoisolated GPIO inputs for optical limit switches
    GPIO.setup(Soft_limit_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Soft_limit_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Soft_limit_top, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Soft_limit_bottom, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.setup(Stepper_power, GPIO.OUT)

    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode)

    GPIO.add_event_detect(Soft_limit_right, GPIO.RISING, callback=soft_limit_violation)
    GPIO.add_event_detect(Soft_limit_left, GPIO.RISING, callback=soft_limit_violation)
    GPIO.add_event_detect(Soft_limit_top, GPIO.RISING, callback=soft_limit_violation)
    GPIO.add_event_detect(Soft_limit_bottom, GPIO.RISING, callback=soft_limit_violation)

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


def soft_limit_violation(port):
    print("Soft limit violation")
    # horizontal_slider.stop()
    # rotor.stop()


# Press the green button in the gutter to run the script.
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

        counter = 10

        # Rotary encoder inputs
        Enc_A = 23
        Enc_B = 24

        # Optoisolated GPIO inputs for optical limit switches
        Soft_limit_right = 17
        Soft_limit_left = 18
        Soft_limit_top = 19
        Soft_limit_bottom = 20

        # power supply for stepper motors
        Stepper_power = 21

        init()

        time.sleep(1)

        GPIO.output(Stepper_power, GPIO.HIGH)  # turn on the power for the steppers

        time.sleep(2)  # wait for steppers to be ready

        serial_port = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=1)

        commander_lock = threading.Lock()

        the_commander = Commander(ser=serial_port, lock=commander_lock)  # create the commander

        # Horizontal axis
        arm = LocatedLinearStepper(commander=the_commander, motor_address=1, inverse_direction=False, position_offset=0.152, name='arm')

        # Vertical axis
        lift = LocatedLinearStepper(commander=the_commander, motor_address=2, inverse_direction=True, position_offset=0.740, name='lift')

        arm.ramp_type = "jerkfree"
        arm.jerk = 5

        lift.ramp_type = "jerkfree"
        lift.jerk = 5

        def reference_arm():
            arm.reference_run()

        def reference_lift():
            lift.reference_run()

        reference_arm_thread = threading.Thread(target=reference_arm)
        reference_lift_thread = threading.Thread(target=reference_lift)

        print("Referencing all axes...")

        reference_arm_thread.start()
        reference_lift_thread.start()

        reference_arm_thread.join()
        reference_lift_thread.join()

        print("All axes referenced.")

        time.sleep(1)

        def move_arm():

            arm.absolute_position = 0.3
            arm.signed_speed = 0.2
            arm.run()
            while not arm.is_ready:
                time.sleep(1)
            arm.stop()

            print("Arm now positioned at ", str(arm.absolute_position))

        # horizontal_slider.run()

        # # 0.02m stopping distance @ jerk 5 and speed 0.1m/s
        # # 0.07m stopping distance @ jerk 5 and speed 0.2m/s

        def move_lift():

            lift.absolute_position = 1.00       # meters above floor
            lift.signed_speed = 0.2
            lift.run()

            while not lift.is_ready:
                time.sleep(1)
            lift.stop()

            print("Lift now positioned at ", str(lift.absolute_position))

        t1 = threading.Thread(target=move_arm)
        t2 = threading.Thread(target=move_lift)

        t1.start()
        t2.start()

        print("Waiting for moves to finish...")

        t1.join()
        t2.join()

        time.sleep(2)  # wait two secs before powering down the steppers

        t = arm.move(distance=0.1, speed=0.1)

        t.join()

        print("All done.")

        GPIO.output(Stepper_power, GPIO.LOW)  # turn off the power for the steppers

        # Rotor

        # radians_per_motor_revolution = 2 * math.pi / 25  # this is the gear ratio of the worm drive

        # rotor = RotationMotor(commander=the_commander, motor_address=3, angle_per_motor_revolution=radians_per_motor_revolution, direction_modifier="default")
        # rotor.ramp_type = "jerkfree"
        # rotor.jerk = 1

        # flat_circle_run()

        # horizontal_slider.run()
        # vertical_slider.run()
        # rotor.run()

        # try:
        #     while True:
        #         time.sleep(1)
        # except KeyboardInterrupt:
        #     GPIO.cleanup()

        # # horizontal_slider.run()
        # # vertical_slider.run()
        # # rotor.run()

        # # start_time = time.time()

        # # circle_run()

        # # end_time = time.time()

        # # elapsed_time = end_time - start_time

        # # print('Execution time:', elapsed_time, 'seconds')

    except KeyboardInterrupt:
        # here you put any code you want to run before the program
        # exits when you press CTRL+C
        print('Exiting upon KeyboardInterrupt.')

    #except:
        # this catches ALL other exceptions including errors.
        # You won't get any error messages for debugging
        # so only use it once your code is working
        print('Exiting upon error.')

    finally:
        GPIO.cleanup()  # this ensures a clean exit
        print('IO ports are cleaned up.')

# end of main
