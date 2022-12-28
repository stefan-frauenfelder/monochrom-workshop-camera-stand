import time
import threading
import math
import RPi.GPIO as GPIO
from enum import Enum

LinearDirection = Enum('LinearDirection', 'positive negative')
RotationalDirection = Enum('RotationalDirection', 'cw ccw')


class Commander():
    def __init__(self, ser, lock):
        self._ser = ser
        self._lock = lock

    def __del__(self):
        self._ser.close()  # close port

    def write_command(self, address, command):
        # the calling thread aquires the lock and blocks other threads from using the commander until done
        with self._lock:

            self._ser.write(b'#' + (str(address)).encode('UTF-8') + command + b'\r')
            answer = self._ser.read_until(b'\r')  # read until '\r' appears
            answer = answer[1:].rstrip(b'\r')
            print('Invoced ' + command.decode('UTF-8') + ' for motor ' + str(address) + ' received answer ' + answer.decode('UTF-8').rstrip('\r'))  # print
            return answer


class NanotecStepper():
    def __init__(self, commander, motor_address=1, name='DefaultName', steps_per_motor_revolution=200, micro_steps_per_step=8):
        self.commander = commander
        self._motor_address = motor_address
        self._steps_per_motor_revolution = steps_per_motor_revolution
        self._micro_steps_per_step = micro_steps_per_step
        self._name = name

        self._command_letters = {
            "position_mode": b'p',
            "step_mode": b'g',
            "travel_distance": b's',
            "initial_step_frequency": b'u',
            "maximum_step_frequency": b'o',
            "second_maximum_step_frequency": b'n',
            "acceleration_ramp": b'b',
            "break_ramp": b'B',
            "direction_of_rotation": b'd',
            "reversal_of_direction_of_rotation_for_repeat_records": b't',
            "repetitions": b'W',
            "pause": b'P',
            "record_number_of_continuation_record": b'N',
            "maximum_jerk_for_acceleration_ramp": b':b',
            "maximum_jerk_for_break_ramp": b':B',
            "ramp_type": b':ramp_mode',
            "joystick_dead_range": b'=',
            "joystick_filter": b'f',
            "limit_switch_behavior": b'l',
            "input_1": b':port_in_a',
            "input_2": b':port_in_b',
            "input_3": b':port_in_c',
            "input_4": b':port_in_d',
            "input_5": b':port_in_e',
            "input_6": b':port_in_f',
            "step_position": b'C',
            "is_referenced": b':is_referenced',
            "status": b'$',
            "reset_error": b'D',
            "factory_reset": b'~'
        }
        self._ram_record = {
            "step_mode": self._micro_steps_per_step,   # number of microsteps per step
            "travel_distance": 1,
            "initial_step_frequency": 1,
            "maximum_step_frequency": 1,
            "second_maximum_step_frequency": 1,
            "acceleration_ramp": 10000,  # irrelevantly high, maximum jerk is limiting factor instead
            "break_ramp": 10000,         # irrelevantly high, maximum jerk is limiting factor instead
            "direction_of_rotation": 0,
            "reversal_of_direction_of_rotation_for_repeat_records": 0,
            "repetitions": 1,
            "pause": 0,
            "record_number_of_continuation_record": 0,
            "maximum_jerk_for_acceleration_ramp": 1,
            "maximum_jerk_for_break_ramp": 1,
            "joystick_dead_range": 5,
            "joystick_filter": 16,
            "limit_switch_behavior": 17442,  # default   alternative: 9250,  # default + ext.lim = stop
            "input_1": 7,  # external reference switch
            "input_2": 7   # external reference switch

        }

        for key in self._ram_record:
            command = self._command_letters[key] + (str(self._ram_record[key]).encode('UTF-8'))
            self.commander.write_command(self._motor_address, command)
            # print('Initializing ' + key + ' of motor ' + str(self._motor_address) + ' to ' + str(self._ram_record[key]))

        self.mode = "relative_positioning"
        self.ramp_type = "jerkfree"

    def step_speed(self, value):
        if value < 1:
            print('WARNING: Increasing speed value of ' + str(value) + ' to 1.')
            value = max(value, 1)
        command = self._command_letters["maximum_step_frequency"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    step_speed = property(None, step_speed)

    def step_distance(self, value):
        command = self._command_letters["travel_distance"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    step_distance = property(None, step_distance)

    def step_direction(self, value):
        command = self._command_letters["direction_of_rotation"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    step_direction = property(None, step_direction)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        if value == "relative_positioning":
            number = 1
        elif value == "absolute_positioning":
            number = 2
        elif value == "internal_reference_run":
            number = 3
        elif value == "external_reference_run":
            number = 4
        elif value == "speed_mode":
            number = 5
        elif value == "joystick_mode":
            number = 12
        else:
            raise ValueError('Unsupported mode.')
        command = self._command_letters["position_mode"] + (str(number).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
        self._mode = value

    @property
    def micro_steps_per_step(self):
        return self._micro_steps_per_step

    @micro_steps_per_step.setter
    def micro_steps_per_step(self, value):
        command = self._command_letters["step_mode"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
        self._micro_steps_per_step = value

    def ramp_type(self, value):
        if value == "trapezoid":
            number = 0
        elif value == "sinusoid":
            number = 1
        elif value == "jerkfree":
            number = 2
        else:
            number = 0
        command = self._command_letters["ramp_type"] + (str(number).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    ramp_type = property(None, ramp_type)

    def jerk(self, value):
        command = self._command_letters["maximum_jerk_for_acceleration_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
        command = self._command_letters["maximum_jerk_for_break_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    jerk = property(None, jerk)

    @property
    def step_position(self):
        command = self._command_letters["step_position"]
        position_answer_byte = self.commander.write_command(self._motor_address, command)
        position_answer_str = position_answer_byte.decode('UTF-8')
        if "+" in position_answer_str:
            position_str = position_answer_str[2:]
        else:
            position_str = position_answer_str[1:]
        return int(position_str)

    @property
    def is_referenced(self):
        command = self._command_letters["is_referenced"]
        position_answer_byte = self.commander.write_command(self._motor_address, command)
        position_answer_str = position_answer_byte.decode('UTF-8')
        if "1" in position_answer_str[1:]:
            return True
        else:
            return False

    @property
    def is_ready(self):
        command = self._command_letters["status"]
        status_answer_byte = self.commander.write_command(self._motor_address, command)
        status_answer_str = status_answer_byte.decode('UTF-8').split("$")
        status_answer_int = int(status_answer_str[1])
        if status_answer_int & 1:
            return True
        elif status_answer_int & 4:
            self.reset_position_error()
            return False
        else:
            return False

    @property
    def motor_address(self):
        return self._motor_address

    @property
    def steps_per_motor_revolution(self):
        return self._steps_per_motor_revolution

    def run(self):
        # start the motor with the current settings
        self.commander.write_command(self._motor_address, b'A')

    def stop(self):
        # stop the motor with the current ramp
        self.commander.write_command(self._motor_address, b'S1')

    def immediate_stop(self):
        # stop the motor with the steep stop ramp
        self.commander.write_command(self._motor_address, b'S0')

    def reset_position_error(self):
        # reset the position error
        print('WARNING: Reseting position error of motor ' + str(self._motor_address) + '!')
        self.commander.write_command(self._motor_address, self._command_letters["reset_error"])

    def reset_to_factory_setting(self):
        # reset the motor to factory settings
        self.commander.write_command(self._motor_address, self._command_letters["factory_reset"])
        time.sleep(1)  # the controller needs a second to accept new commands and should be powered off and back on


class PhysicalLinearStepper(NanotecStepper):

    def __init__(self, commander, motor_address, name='DefaultName', steps_per_motor_revolution=200, micro_steps_per_step=8, distance_per_motor_revolution=0.12):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step)

        self._distance_per_motor_revolution = distance_per_motor_revolution

    def distance(self, value):
        # convert from physical distance in meters to (micro) steps of the motor
        self.step_distance = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._distance_per_motor_revolution)
    distance = property(None, distance)

    def direction(self, value):
        # set the turning direction of the motor according to the cw or ccw input direction and the direction modifier
        if value == LinearDirection.positive:
            self.step_direction = 1
        elif value == LinearDirection.negative:
            self.step_direction = 0
        else:
            raise ValueError('Direction of linear motor needs to be positive or negative.')
    direction = property(None, direction)

    def speed(self, value):
        # convert from absolute (always positive) physical speed in meters per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._distance_per_motor_revolution)
    speed = property(None, speed)

    @property
    def position(self):
        return float(self.step_position) / self.micro_steps_per_step / self.steps_per_motor_revolution * self._distance_per_motor_revolution


class OrientedLinearStepper(PhysicalLinearStepper):

    def __init__(self, commander, motor_address, name='DefaultName', steps_per_motor_revolution=200, micro_steps_per_step=8, distance_per_motor_revolution=0.12, inverse_direction=False, safe_length=0.6):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step, distance_per_motor_revolution)

        self._inverse_direction = inverse_direction
        self._origin_is_set = False
        self._safe_length = safe_length

    def signed_distance(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = LinearDirection.positive
                self.distance = value
            else:
                self.direction = LinearDirection.negative
                self.distance = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = LinearDirection.negative
                self.distance = value
            else:
                self.direction = LinearDirection.positive
                self.distance = -value
    signed_distance = property(None, signed_distance)

    def signed_speed(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = LinearDirection.positive
                self.speed = value
            else:
                self.direction = LinearDirection.negative
                self.speed = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = LinearDirection.negative
                self.speed = value
            else:
                self.direction = LinearDirection.positive
                self.speed = -value
    signed_speed = property(None, signed_speed)

    def signed_position(self):
        if not self._inverse_direction:  # direction is default
            return self.position
        else:  # direction is inverse
            return -self.position
    signed_position = property(signed_position, None)

    def move(self, distance, speed=False):

        self.mode = "relative_positioning"
        if speed:
            self.signed_speed = speed
        self.signed_distance = distance

    def wait_for_ready(self):
        # this is blocking and normally only called in its own thread
        while not self.is_ready:
            time.sleep(0.5)
        self.stop()  # not sure this is needed

    # utility functions

    def find_origin(self):

        if not self.is_referenced:

            # store values to reapply later
            previous_mode = self.mode
            previous_micro_steps_per_step = self.micro_steps_per_step

            print('Finding origin of ' + self._name + ' axis.')

            self.mode = "external_reference_run"
            self.micro_steps_per_step = 1   # full step mode
            self.distance = 2 * self._safe_length  # m
            self.signed_speed = -0.02       # m/s

            self.run()
            while not self.is_referenced:
                time.sleep(0.5)
            self.stop()  # in case the motor was already referenced and it is still running

            # switch ro suitable settings to move within soft limits
            self.micro_steps_per_step = previous_micro_steps_per_step
            # move to within soft limits
            self.move(distance=self._safe_length / 3, speed=0.2)
            self.run()
            while not self.is_ready:
                time.sleep(0.5)
            self.stop()

            # restore previous settings
            self.mode = previous_mode

            self._origin_is_set = True

            print('Origin of ' + self._name + ' axis set. Now positioned at ' + str(self.signed_position))

        else:
            # must not happen
            raise ValueError('Find origin must not be called for an already referenced motor. Power cycle the motor before every new run of this software.')


class FiniteLinearStepper(OrientedLinearStepper):

    def __init__(self, commander, motor_address, name='DefaultName',
                 steps_per_motor_revolution=200, micro_steps_per_step=8,
                 distance_per_motor_revolution=0.12, inverse_direction=False, safe_length=0.6,
                 near_soft_limit_gpio=False, far_soft_limit_gpio=False):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step, distance_per_motor_revolution, inverse_direction, safe_length)

        self._near_soft_limit_gpio = near_soft_limit_gpio
        self._far_soft_limit_gpio = far_soft_limit_gpio

        self._near_soft_limit_location = False
        self._far_soft_limit_location = False

        self._is_off_near_soft_limit = False
        self._is_off_far_soft_limit = False

        self._safe_travel_distance = False

        self._safety_margin = 0.01  # the margin between the soft limit and the safe zone

        self._is_limited = False

        GPIO.setup(self._near_soft_limit_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._far_soft_limit_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self._near_soft_limit_gpio, GPIO.BOTH, callback=self.near_soft_limit_event)
        GPIO.add_event_detect(self._far_soft_limit_gpio, GPIO.BOTH, callback=self.far_soft_limit_event)

    def near_soft_limit_event(self, channel):
        if GPIO.input(channel):    # edge was rising
            self.near_soft_limit_enter()
        else:                                   # edge was falling
            self.near_soft_limit_leave()

    def far_soft_limit_event(self, channel):
        if GPIO.input(channel):     # edge was rising
            self.far_soft_limit_enter()
        else:                                   # edge was falling
            self.far_soft_limit_leave()

    def near_soft_limit_enter(self):
        print('Entering near soft limit of ' + self._name + ' axis.')
        self._is_off_near_soft_limit = True
        self.off_limits()

    def near_soft_limit_leave(self):
        print('Leaving near soft limit of ' + self._name + ' axis.')
        self._is_off_near_soft_limit = False

    def far_soft_limit_enter(self):
        print('Entering far soft limit of ' + self._name + ' axis.')
        self._is_off_far_soft_limit = True
        self.off_limits()

    def far_soft_limit_leave(self):
        print('Leaving far soft limit of ' + self._name + ' axis.')
        self._is_off_far_soft_limit = False

    def off_limits(self):
        if self._is_limited:
            self.stop()
            print('WARNING: ' + self._name + ' axis stopped due to soft limit violation.')

    def find_limits(self):
        if not self.is_referenced:
            self.find_origin()

        # move to off near limit
        self.move(distance=- 2 * self._safe_length, speed=0.1)
        self.run()
        while not self._is_off_near_soft_limit:
            time.sleep(0.1)
        self.stop()

        # slowly go back to within limit
        self.move(distance=2 * self._safe_length, speed=0.01)
        self.run()
        while self._is_off_near_soft_limit:
            time.sleep(0.01)
        self.stop()

        # set the near limit
        current_position = self.signed_position
        self._near_soft_limit_location = current_position
        print('Set near soft limit location of ' + self._name + ' axis to ' + str(current_position) + '.')

        # move to near far limit
        self.move(distance=self._safe_length, speed=0.2)
        self.run()
        while not self.is_ready:
            time.sleep(0.5)
        self.stop()

        # move beyond far limit
        self.move(distance=self._safe_length, speed=0.1)
        self.run()
        while not self._is_off_far_soft_limit:
            time.sleep(0.1)
        self.stop()

        # slowly go back to within limit
        self.move(distance=-self._safe_length, speed=0.01)
        self.run()
        while self._is_off_far_soft_limit:
            time.sleep(0.01)
        self.stop()

        # set the far limit
        current_position = self.signed_position
        self._far_soft_limit_location = current_position
        print('Set far soft limit location of ' + self._name + ' axis to ' + str(current_position) + '.')

        self._safe_travel_distance = self._far_soft_limit_location - self._near_soft_limit_location - 2 * self._safety_margin

        # move to center
        self.move(distance=- self._safe_travel_distance / 2, speed=0.2)
        self.run()
        while not self.is_ready:
            time.sleep(0.5)
        self.stop()

        self._is_limited = True

        print('Safe travel distance of ' + self._name + ' axis is ' + str(self._safe_travel_distance) + '.')
        print('Soft limit guard of ' + self._name + ' axis activated.')

    def get_limited_position(self):
        return self.signed_position - self._safety_margin - self._near_soft_limit_location

    def set_limited_position(self, value):
        self.signed_distance = value + self._safety_margin + self._near_soft_limit_location

    limited_position = property(get_limited_position, set_limited_position)


class LocatedLinearStepper(FiniteLinearStepper):

    def __init__(self, commander, motor_address, name='DefaultName',
                 steps_per_motor_revolution=200, micro_steps_per_step=8,
                 distance_per_motor_revolution=0.12, inverse_direction=False, safe_length=0.6,
                 near_soft_limit_gpio=False, far_soft_limit_gpio=False,
                 position_offset=0):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step, distance_per_motor_revolution, inverse_direction, safe_length, near_soft_limit_gpio, far_soft_limit_gpio)

        self._position_offset = position_offset

    def get_absolute_position(self):
        return self.limited_position + self._position_offset

    def set_absolute_position(self, value):
        self.limited_position = value - self._position_offset

    absolute_position = property(get_absolute_position, set_absolute_position)

    def goto_absolute_position(self, position, speed):
        if position > self._position_offset and position < (self._position_offset + self._safe_travel_distance):
            self.mode = "absolute_positioning"
            if speed:
                self.signed_speed = speed
            self.absolute_position = position
        else:
            raise ValueError('Position to go to is outside safe travel distance.')


class PhysicalRotationalStepper(NanotecStepper):

    def __init__(self, commander, motor_address, name='DefaultName', steps_per_motor_revolution=200, micro_steps_per_step=8, angle_per_motor_revolution=0.2):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step)

        self._angle_per_motor_revolution = angle_per_motor_revolution

    def angle(self, value):
        # convert from physical angle in radians to (micro) steps of the motor
        self.step_distance = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._angle_per_motor_revolution)
    angle = property(None, angle)

    def direction(self, value):
        # set the turning direction of the motor according to the cw or ccw argument
        if value == RotationalDirection.cw:
            self.step_direction = 1  # Todo: this may be wrong!
        elif value == RotationalDirection.ccw:
            self.step_direction = 0
        else:
            raise ValueError('Direction of rotation needs to be either cw or ccw.')
    direction = property(None, direction)

    def speed(self, value):
        # convert from physical speed in radians per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._angle_per_motor_revolution)
    speed = property(None, speed)

    @property
    def position(self):
        return float(self.step_position) / self.micro_steps_per_step / self.steps_per_motor_revolution * self._angle_per_motor_revolution


class OrientedRotationalStepper(PhysicalRotationalStepper):

    def __init__(self, commander, motor_address, name='DefaultName', steps_per_motor_revolution=200, micro_steps_per_step=8, angle_per_motor_revolution=0.2, inverse_direction=False, safe_angle=3):
        super().__init__(commander, motor_address, name, steps_per_motor_revolution, micro_steps_per_step, angle_per_motor_revolution)

        self._inverse_direction = inverse_direction
        self._origin_is_set = False
        self._safe_angle = safe_angle

    def signed_angle(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = RotationalDirection.ccw
                self.angle = value
            else:
                self.direction = RotationalDirection.cw
                self.angle = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = RotationalDirection.cw
                self.angle = value
            else:
                self.direction = RotationalDirection.ccw
                self.angle = -value
    signed_angle = property(None, signed_angle)

    def signed_speed(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = RotationalDirection.ccw
                self.speed = value
            else:
                self.direction = RotationalDirection.cw
                self.speed = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = RotationalDirection.cw
                self.speed = value
            else:
                self.direction = RotationalDirection.ccw
                self.speed = -value
    signed_speed = property(None, signed_speed)

    def signed_position(self):
        if not self._inverse_direction:  # direction is default
            return self.position
        else:  # direction is inverse
            return -self.position
    signed_position = property(signed_position, None)

    def move(self, angle, speed=False):

        self.mode = "relative_positioning"
        if speed:
            self.signed_speed = speed
        self.signed_angle = angle

    def find_origin(self, direction=RotationalDirection.cw):

        if not self.is_referenced:

            # store values to reapply later
            previous_mode = self.mode

            print('Finding origin of ' + self._name + ' axis.')

            if direction == RotationalDirection.ccw:  # approaching origin from negative angles
                sign_modifier = 1
            elif direction == RotationalDirection.cw:  # approaching origin from positive angles
                sign_modifier = -1
            else:
                raise ValueError('Direction of rotation to find origin needs to be either cw or ccw.')

            # assumption: rotational axis is started up in about neutral position

            # backup from neutral startup position by a quarter rotation
            self.mode = "relative_positioning"
            self.speed = 1
            self.signed_angle = sign_modifier * -math.pi / 4
            self.run()
            while not self.is_ready:
                time.sleep(0.5)
            self.stop()

            # approach origin slowly
            self.mode = 'internal_reference_run'
            self.speed = 0.1
            self.signed_angle = sign_modifier * 2 * math.pi
            self.run()
            while not self.is_referenced:
                time.sleep(0.5)
            self.stop()  # in case the motor was already referenced and it is still running

            # restore previous settings
            self.mode = previous_mode

            self._origin_is_set = True

            print('Origin of ' + self._name + ' axis set. Now positioned at ' + str(self.signed_position))

        else:
            # must not happen
            raise ValueError('Find origin must not be called for an already referenced motor. Power cycle the motor before every new run of this software.')
