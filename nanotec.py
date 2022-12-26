import time
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
            # print('Invoced ' + command.decode('UTF-8') + ' for motor ' + str(address) + ' received answer ' + answer.decode('UTF-8').rstrip('\r'))  # print
            return answer


class NanotecStepper():
    def __init__(self, commander, motor_address=1, steps_per_motor_revolution=200, micro_steps_per_step=8):
        self.commander = commander
        self._motor_address = motor_address
        self._steps_per_motor_revolution = steps_per_motor_revolution
        self._micro_steps_per_step = micro_steps_per_step

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
            "status": b'$'
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
            print('Initializing ' + key + ' of motor ' + str(self._motor_address) + ' to ' + str(self._ram_record[key]))

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
        # stop the motor with the current stop ramp
        self.commander.write_command(self._motor_address, b'S0')


class PhysicalLinearStepper(NanotecStepper):

    def __init__(self, commander, motor_address, steps_per_motor_revolution=200, micro_steps_per_step=8, distance_per_motor_revolution=0.12):
        super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step)

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

    def __init__(self, commander, motor_address, steps_per_motor_revolution=200, micro_steps_per_step=8, distance_per_motor_revolution=0.12, inverse_direction=False):
        super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step, distance_per_motor_revolution)

        self._inverse_direction = inverse_direction

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

    # utility functions
    def reference_run(self):
        if not self.is_referenced:

            print('Referencing ' + self._name + ' axis.')
            # store values to reapply later
            previous_mode = self.mode
            previous_micro_steps_per_step = self.micro_steps_per_step

            self.mode = "external_reference_run"
            self.micro_steps_per_step = 1   # full step mode
            self.distance = 1               # m
            self.signed_speed = -0.02       # m/s

            self.run()

            while not self.is_referenced:
                time.sleep(1)

            time.sleep(0.5)

            # restore previous values
            self.mode = previous_mode
            self.micro_steps_per_step = previous_micro_steps_per_step


class LocatedLinearStepper(OrientedLinearStepper):

    def __init__(self, commander, motor_address, steps_per_motor_revolution=200, micro_steps_per_step=8, distance_per_motor_revolution=0.12, inverse_direction=False, position_offset=0, name='DefaultName'):
        super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step, distance_per_motor_revolution, inverse_direction)

        self._position_offset = position_offset
        self._name = name

    def get_absolute_position(self):
        return self.signed_position + self._position_offset

    def set_absolute_position(self, value):
        self.signed_distance = value - self._position_offset

    absolute_position = property(get_absolute_position, set_absolute_position)


# class RotationMotor(NanotecStepper):

#     def __init__(self, commander, motor_address, angle_per_motor_revolution, steps_per_motor_revolution=200, micro_steps_per_step=8, inverse_direction=False):
#         super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step)

#         self._angle_per_motor_revolution = angle_per_motor_revolution

#     def angle(self, value):
#         # convert from physical angle in radians to (micro) steps of the motor
#         self.step_distance = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._angle_per_motor_revolution)
#     angle = property(None, angle)

#     def direction(self, value):
#         # set the turning direction of the motor according to the positive or negative input direction and the direction modifier
#         if value == RotationalDirection.cw:
#             if self._direction_modifier == "default":
#                 self.step_direction = 0
#             elif self._direction_modifier == "inverse":
#                 self.step_direction = 1
#             else:
#                 raise ValueError('Direction modifier needs to be default or inverse.')
#         elif value == RotationalDirection.ccw:
#             if self._direction_modifier == "default":
#                 self.step_direction = 1
#             elif self._direction_modifier == "inverse":
#                 self.step_direction = 0
#             else:
#                 raise ValueError('Direction modifier needs to be default or inverse.')
#         else:
#             raise ValueError('Direction of linear motor needs to be cw or ccw.')
#     direction = property(None, direction)

#     def speed(self, value):
#         # convert from physical speed in radians per second to (micro) steps per second of the motor
#         self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self._angle_per_motor_revolution)
#     speed = property(None, speed)
