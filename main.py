

def linear_ramp(LinearMotor, speed_step_frequency=10, cycle_duration=8, max_speed=0.1):

    speed_step_periode = 1.0 / speed_step_frequency

    num_speed_steps = int((cycle_duration / 4.0) / speed_step_periode)

    horizontal_slider.run()

    for half_cycle in range(2):

        horizontal_slider.direction = half_cycle

        for speed_step in range(num_speed_steps):
            horizontal_slider.speed = max_speed * (speed_step / num_speed_steps)
            time.sleep(speed_step_periode)

        for speed_step in range(num_speed_steps, 0, -1):
            horizontal_slider.speed = max_speed * (speed_step / num_speed_steps)
            time.sleep(speed_step_periode)

    horizontal_slider.stop()


def circle_run(step_frequency=10, cycle_duration=8, max_speed=0.1):

    step_periode = 1.0 / step_frequency

    num_steps_for_cycle = int(cycle_duration / step_periode)

    horizontal_slider.run()
    vertical_slider.run()

    for step in range(num_steps_for_cycle):

        angle = step / num_steps_for_cycle * 2.0 * math.pi

        horizontal_speed_scaling = math.sin(angle)
        vertical_speed_scaling = math.cos(angle)

        if horizontal_speed_scaling > 0:
            horizontal_slider.direction = 0
        else:
            horizontal_slider.direction = 1
            horizontal_speed_scaling = -horizontal_speed_scaling

        horizontal_slider.speed = horizontal_speed_scaling * max_speed

        if vertical_speed_scaling > 0:
            vertical_slider.direction = 0
        else:
            vertical_slider.direction = 1
            vertical_speed_scaling = -vertical_speed_scaling

        vertical_slider.speed = vertical_speed_scaling * max_speed

        time.sleep(step_periode)

    horizontal_slider.stop()
    vertical_slider.stop()


def init():
    print("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(Enc_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    import serial
    import time
    import sys
    import math
    import RPi.GPIO as GPIO

    from nanotec import *

    print(sys.executable)
    print(sys.version)

    counter = 10

    Enc_A = 23
    Enc_B = 24

    try:
        init()
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()

    # serial_port = serial.Serial(port='/dev/ttyUSB0',
    #                             baudrate=115200,
    #                             bytesize=serial.EIGHTBITS,
    #                             parity=serial.PARITY_NONE,
    #                             stopbits=serial.STOPBITS_ONE,
    #                             timeout=1)

    # the_commander = Commander()  # create the singleton commander
    # the_commander.ser = serial_port  # hand the serial port to the commander to use

    # # the motor for the horizontal motion
    # horizontal_slider = LinearMotor(commander=the_commander, motor_address=1, distance_per_revolution=0.12)

    # vertical_slider = LinearMotor(commander=the_commander, motor_address=2, distance_per_revolution=0.12)

    # horizontal_slider.speed = 0.005     # m/s
    # horizontal_slider.direction = 0     # 1 is out, 0 is in
    # horizontal_slider.mode = "speed_mode"
    # horizontal_slider.ramp_type = "jerkfree"
    # horizontal_slider.jerk = 5

    # vertical_slider.speed = 0.005     # m/s
    # vertical_slider.direction = 0     # 1 is out, 0 is in
    # vertical_slider.mode = "speed_mode"
    # vertical_slider.ramp_type = "jerkfree"
    # vertical_slider.jerk = 5

    # start_time = time.time()

    # circle_run()

    # end_time = time.time()

    # elapsed_time = end_time - start_time

    # print('Execution time:', elapsed_time, 'seconds')
