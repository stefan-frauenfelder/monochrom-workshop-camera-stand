import time
import threading
import math
import RPi.GPIO as GPIO
from enum import Enum

import sequent_ports

Direction = Enum('Direction', 'positive negative')


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


class SoftLimitViolationException(Exception):
    "Raised when a stepper moves outside its safe travel zone"
    pass


class NanotecStepper():

    _command_letters = {
        "position_mode": b'p',
        "step_mode": b'g',
        "travel": b's',
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

    _defaults = {
        "travel": 1,
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

    def __init__(self, commander, io_card, stepper_config):

        self.stepper_config = stepper_config

        self.commander = commander
        self._io_card = io_card

        self._motor_address = self.stepper_config['busAddress']
        self._steps_per_motor_revolution = self.stepper_config['stepsPerMotorRevolution']
        self._micro_steps_per_step = self.stepper_config['microStepsPerStep']
        self.name = self.stepper_config['name']
        self._power_relay = self.stepper_config['powerRelay']
        self.armed = False

    def initialize(self):

        for key in self._defaults:
            command = self._command_letters[key] + (str(self._defaults[key]).encode('UTF-8'))
            self.commander.write_command(self._motor_address, command)
            # print('Initializing ' + key + ' of motor ' + str(self._motor_address) + ' to ' + str(self._defaults[key]))

        self.micro_steps_per_step = self._micro_steps_per_step
        self.mode = "relative_positioning"
        self.ramp_type = "jerkfree"

    def power_up(self):
        self._io_card.set_output(self._power_relay, 1)

    def shutdown(self):
        print('Powering down ' + self.name + ' axis.')
        self._io_card.set_output(self._power_relay, 0)

    def step_speed(self, value):
        if value < 1:
            print('WARNING: Increasing speed value of ' + str(value) + ' to 1.')
            value = max(value, 1)
        command = self._command_letters["maximum_step_frequency"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
    step_speed = property(None, step_speed)

    def step_travel(self, value):
        command = self._command_letters["travel"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self._motor_address, command)
        self.armed = True
    step_travel = property(None, step_travel)

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
        self.armed = False

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
        time.sleep(1)  # the coordinator needs a second to accept new commands and should be powered off and back on

    def wait_for_ready(self, polling_time=0.5):
        # this is blocking and normally only called in its own thread
        time.sleep(polling_time)
        while not self.is_ready:
            time.sleep(polling_time)
        self.stop()  # not sure this is needed

    def blocking_run(self, polling_time=0.5):
        self.run()
        self.wait_for_ready(polling_time)


class PhysicalStepper(NanotecStepper):

    def __init__(self, commander, io_card, stepper_config):
        super().__init__(commander, io_card, stepper_config)

        self._si_unit_per_motor_revolution = self.stepper_config['siUnitPerMotorRevolution']
        self._maximum_speed = self.stepper_config['maximumSpeed']

    def travel(self, value):
        # convert from physical travel in meters to (micro) steps of the motor
        self.step_travel = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._si_unit_per_motor_revolution)
    travel = property(None, travel)

    def direction(self, value):
        # set the turning direction of the motor according to the negative or positive input direction and the direction modifier
        if value == Direction.positive:
            self.step_direction = 1
        elif value == Direction.negative:
            self.step_direction = 0
        else:
            raise ValueError('Direction needs to be positive or negative.')
    direction = property(None, direction)

    def speed(self, value):
        if value > self._maximum_speed:
            # limit the speed according to config
            value = min(value, self._maximum_speed)
            print('WARNING: Limiting speed of ' + self.name + ' axis!')
        # convert from absolute (always positive) physical speed in meters per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._si_unit_per_motor_revolution)
    speed = property(None, speed)

    @property
    def position(self):
        return float(self.step_position) / self.micro_steps_per_step / self.steps_per_motor_revolution * self._si_unit_per_motor_revolution


class OrientedStepper(PhysicalStepper):

    def __init__(self, commander, io_card, stepper_config):
        super().__init__(commander, io_card, stepper_config)

        if self.stepper_config['orientation'] == 'inverse':
            self._inverse_direction = True
        else:
            self._inverse_direction = False

        self._approx_safe_travel = self.stepper_config['approximateSafeTravel']

        self._origin_is_set = False

    def shutdown(self):
        self._origin_is_set = False
        super().shutdown()

    def signed_travel(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = Direction.positive
                self.travel = value
            else:
                self.direction = Direction.negative
                self.travel = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = Direction.negative
                self.travel = value
            else:
                self.direction = Direction.positive
                self.travel = -value
    signed_travel = property(None, signed_travel)

    def signed_speed(self, value):
        if not self._inverse_direction:  # direction is default
            if value >= 0:
                self.direction = Direction.positive
                self.speed = value
            else:
                self.direction = Direction.negative
                self.speed = -value
        else:  # direction is inverse
            if value >= 0:
                self.direction = Direction.negative
                self.speed = value
            else:
                self.direction = Direction.positive
                self.speed = -value
    signed_speed = property(None, signed_speed)

    def get_signed_position(self):
        if not self._inverse_direction:  # direction is default
            return self.position
        else:  # direction is inverse
            return -self.position

    def set_signed_position(self, value):
        if not self._inverse_direction:  # direction is default
            self.travel = value
        else:  # direction is inverse
            self.travel = -value

    signed_position = property(get_signed_position, set_signed_position)

    def move(self, travel, speed=False):

        self.mode = "relative_positioning"
        if speed:
            self.signed_speed = speed
        self.signed_travel = travel

    # utility functions

    def find_linear_stepper_origin(self):

        if not self.is_referenced:

            print('Finding origin of ' + self.name + ' axis.')

            # store values to reapply later
            previous_mode = self.mode
            previous_micro_steps_per_step = self.micro_steps_per_step

            self.ramp_type = "jerkfree"
            self.jerk = 5

            self.mode = "external_reference_run"
            self.micro_steps_per_step = 1   # full step mode
            self.travel = 2 * self._approx_safe_travel  # m
            self.signed_speed = -0.02       # m/s

            self.run()
            while not self.is_referenced:
                time.sleep(0.5)
            self.stop()  # in case the motor was already referenced and it is still running

            # switch ro suitable settings to move within soft limits
            self.micro_steps_per_step = previous_micro_steps_per_step
            # move to within soft limits
            self.move(travel=self._approx_safe_travel / 3, speed=0.2)
            self.blocking_run()

            # restore previous settings
            self.mode = previous_mode

            self._origin_is_set = True

            print('Origin of ' + self.name + ' axis set. Now positioned at ' + str(self.signed_position))

        else:
            # must not happen
            raise ValueError('Find origin must not be called for an already referenced motor. Power cycle the motor before every new run of this software.')

    def find_rotational_stepper_origin(self, limit_switch='internal', direction=Direction.negative):

        if not self.is_referenced:

            print('Finding origin of ' + self.name + ' axis.')

            # store values to reapply later
            previous_mode = self.mode

            self.ramp_type = "jerkfree"
            self.jerk = 5

            if direction == Direction.positive:  # approaching origin from negative angles
                sign_modifier = 1

            elif direction == Direction.negative:  # approaching origin from positive angles
                sign_modifier = -1

            else:
                raise ValueError('Direction of rotation to find origin needs to be either negative or positive.')

            # assumption: rotational axis is started up in about neutral position

            # backup from neutral startup position by a quarter rotation
            self.mode = "relative_positioning"
            self.speed = 0.2
            self.signed_travel = sign_modifier * -math.pi / 4
            print('Backing up...')
            self.blocking_run()

            # activate the appropriate mode

            if limit_switch == 'internal':  # approaching origin from negative angles
                self.mode = 'internal_reference_run'

            elif limit_switch == 'external':  # approaching origin from positive angles
                self.mode = 'external_reference_run'

            else:
                raise ValueError('Limit switch parameter needs to be either internal or external.')

            # approach origin slowly
            self.speed = 0.1
            self.signed_travel = sign_modifier * 2 * math.pi
            self.run()
            while not self.is_referenced:
                time.sleep(0.5)
            self.stop()  # in case the motor was already referenced and it is still running

            # restore previous settings
            self.mode = previous_mode

            self._origin_is_set = True

            print('Origin of ' + self.name + ' axis set. Now positioned at ' + str(self.signed_position))

        else:
            # must not happen
            raise ValueError('Find origin must not be called for an already referenced motor. Power cycle the motor before every new run of this software.')

    def find_rotor_origin(self, limit_switch='internal', direction=Direction.negative):

        if not self.is_referenced:

            print('Finding origin of ' + self.name + ' axis.')

            # store values to reapply later
            previous_mode = self.mode

            self.ramp_type = "jerkfree"
            self.jerk = 3

            if direction == Direction.positive:  # approaching origin from negative angles
                sign_modifier = 1

            elif direction == Direction.negative:  # approaching origin from positive angles
                sign_modifier = -1

            else:
                raise ValueError('Direction of rotation to find origin needs to be either negative or positive.')

            # assumption: rotational axis is started up in about neutral position

            # backup from neutral startup position by a quarter rotation
            self.mode = "relative_positioning"
            self.speed = 0.05
            self.signed_travel = sign_modifier * -math.pi / 8
            print('Backing up...')
            self.blocking_run()

            # activate the appropriate mode

            if limit_switch == 'internal':  # approaching origin from negative angles
                self.mode = 'internal_reference_run'

            elif limit_switch == 'external':  # approaching origin from positive angles
                self.mode = 'external_reference_run'

            else:
                raise ValueError('Limit switch parameter needs to be either internal or external.')

            # adjust mictrosteps to increase speed of traveling down from limit switch
            self.micro_steps_per_step = 1

            # approach origin slowly
            self.speed = 0.02
            self.signed_travel = sign_modifier * 2 * math.pi
            self.run()
            while not self.is_referenced:
                time.sleep(0.5)
            self.stop()  # in case the motor was already referenced and it is still running

            # restore previous settings
            self.mode = previous_mode
            self.micro_steps_per_step = self.stepper_config['microStepsPerStep']

            self._origin_is_set = True

            print('Origin of ' + self.name + ' axis set. Now positioned at ' + str(self.signed_position))

        else:
            # must not happen
            raise ValueError('Find origin must not be called for an already referenced motor. Power cycle the motor before every new run of this software.')


class FiniteStepper(OrientedStepper):

    def __init__(self, commander, io_card, stepper_config):
        super().__init__(commander, io_card, stepper_config)

        self._near_soft_limit_location = False
        self._far_soft_limit_location = False

        self._is_off_near_soft_limit = False
        self._is_off_far_soft_limit = False

        self._safe_travel_range = False

        self._safety_margin = 0.01  # the margin between the soft limit and the safe zone

        self._is_limited = False

        near_soft_limit_port = self.stepper_config['nearSoftLimitPort']
        far_soft_limit_port = self.stepper_config['farSoftLimitPort']

        # register callbacks using the sequence i/o-card (connected to the soft limit switches)
        if near_soft_limit_port is not None:
            self._io_card.add_callback(near_soft_limit_port, 'RISING', self.near_soft_limit_enter)
            self._io_card.add_callback(near_soft_limit_port, 'FALLING', self.near_soft_limit_leave)

        if far_soft_limit_port is not None:
            self._io_card.add_callback(far_soft_limit_port, 'RISING', self.far_soft_limit_enter)
            self._io_card.add_callback(far_soft_limit_port, 'FALLING', self.far_soft_limit_leave)

    def shutdown(self):
        self._is_limited = False
        super().shutdown()

    def near_soft_limit_enter(self):
        print('Entering near soft limit of ' + self.name + ' axis.')
        self._is_off_near_soft_limit = True
        self.off_limits()

    def near_soft_limit_leave(self):
        print('Leaving near soft limit of ' + self.name + ' axis.')
        self._is_off_near_soft_limit = False

    def far_soft_limit_enter(self):
        print('Entering far soft limit of ' + self.name + ' axis.')
        self._is_off_far_soft_limit = True
        self.off_limits()

    def far_soft_limit_leave(self):
        print('Leaving far soft limit of ' + self.name + ' axis.')
        self._is_off_far_soft_limit = False

    def off_limits(self):
        if self._is_limited:
            self.immediate_stop()
            print('WARNING: ' + self.name + ' axis stopped due to soft limit violation.')
            raise SoftLimitViolationException

    def find_linear_stepper_limits(self):
        if not self.is_referenced:
            self.find_linear_stepper_origin()

        # move to off near limit
        self.move(travel=- 2 * self._approx_safe_travel, speed=0.1)
        self.run()
        while not self._is_off_near_soft_limit:
            time.sleep(0.1)
        self.stop()

        # slowly go back to within limit
        self.move(travel=2 * self._approx_safe_travel, speed=0.01)
        self.run()
        while self._is_off_near_soft_limit:
            time.sleep(0.01)
        self.stop()

        # set the near limit
        current_position = self.signed_position
        self._near_soft_limit_location = current_position
        print('Set near soft limit location of ' + self.name + ' axis to ' + str(current_position) + '.')

        # move to near far limit
        self.move(travel=self._approx_safe_travel, speed=0.2)
        self.run()
        while not self.is_ready:
            time.sleep(0.5)
        self.stop()

        # move beyond far limit
        self.move(travel=self._approx_safe_travel, speed=0.1)
        self.run()
        while not self._is_off_far_soft_limit:
            time.sleep(0.1)
        self.stop()

        # slowly go back to within limit
        self.move(travel=-self._approx_safe_travel, speed=0.01)
        self.run()
        while self._is_off_far_soft_limit:
            time.sleep(0.01)
        self.stop()

        # set the far limit
        current_position = self.signed_position
        self._far_soft_limit_location = current_position
        print('Set far soft limit location of ' + self.name + ' axis to ' + str(current_position) + '.')

        self._safe_travel_range = self._far_soft_limit_location - self._near_soft_limit_location - 2 * self._safety_margin

        # move to center
        self.move(travel=- self._safe_travel_range / 2, speed=0.2)
        self.blocking_run()

        self._is_limited = True

        print('Safe travel of ' + self.name + ' axis is ' + str(self._safe_travel_range) + '.')
        print('Soft limit guard of ' + self.name + ' axis activated.')

    def set_fake_rotational_stepper_limits(self, limit):

        self._near_soft_limit_location = - limit
        self._far_soft_limit_location = limit
        self._safety_margin = math.pi / 18  # 10°
        self._safe_travel_range = self._far_soft_limit_location - self._near_soft_limit_location - 2 * self._safety_margin

    def get_limited_position(self):
        return self.signed_position

    def set_limited_position(self, value):

        lower_limit = self._near_soft_limit_location + self._safety_margin
        upper_limit = lower_limit + self._safe_travel_range

        if value >= lower_limit and value <= upper_limit:  # all good
            self.signed_position = value

        elif value < lower_limit:
            print('WARNING: Position to go to is outside safe travel range. ' + self.name + ' axis movement limited!')
            self.signed_position = lower_limit

        elif value > upper_limit:
            print('WARNING: Position to go to is outside safe travel range. ' + self.name + ' axis movement limited!')
            self.signed_position = upper_limit

        else:
            raise ValueError('Okay, something is fucked up here.')

    limited_position = property(get_limited_position, set_limited_position)

    def get_near_limit(self):
        return self._near_soft_limit_location + self._safety_margin
    near_limit = property(get_near_limit, None)

    def get_far_limit(self):
        return self._far_soft_limit_location - self._safety_margin
    far_limit = property(get_far_limit, None)


class LocatedStepper(FiniteStepper):

    def __init__(self, commander, io_card, stepper_config):
        super().__init__(commander, io_card, stepper_config)

        self._position_offset = self.stepper_config['absolutePositionOffset']

    def get_absolute_position(self):
        return self.limited_position + self._position_offset

    def set_absolute_position(self, value):
        self.limited_position = value - self._position_offset

    absolute_position = property(get_absolute_position, set_absolute_position)

    def goto_absolute_position(self, position, speed):
        self.mode = "absolute_positioning"
        self.signed_speed = speed
        self.absolute_position = position

    def set_position_offset_to_current_position(self):
        self._position_offset = - self.limited_position
