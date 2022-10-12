import time


class Commander():
    def __int__(self, ser, port, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = ser

    def __del__(self):
        self.ser.close()  # close port

    def write_command(self, address, command):
        self.ser.write(b'#' + (str(address)).encode('UTF-8') + command + b'\r')
        answer = self.ser.read_until(b'\r')  # read until '\r' appears
        answer = answer[1:].rstrip(b'\r')
        # print('Invoced ' + command.decode('UTF-8') + ' for motor ' + str(address) + ' received answer ' + answer.decode('UTF-8').rstrip('\r'))  # print
        return answer


class NanotecPd6Motor():
    def __init__(self, commander, motor_address=1, steps_per_motor_revolution=200, micro_steps_per_step=2):
        self.commander = commander
        self.motor_address = motor_address
        self.steps_per_motor_revolution = steps_per_motor_revolution
        self.micro_steps_per_step = micro_steps_per_step

        self._command_letters = {
            "position_mode": b'p',
            "step_mode": b'g',
            "travel_distance": b's',
            "initial_step_frequency": b'u',
            "maximum_step_frequency": b'o',
            "second_maximum_step_frequency": b'n',
            "acceleration_ramp": b':accel',
            "break_ramp": b':decel',
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
            "position_mode": 1,
            "step_mode": self.micro_steps_per_step,   # number of microsteps per step
            "travel_distance": 1,
            "initial_step_frequency": 1,
            "maximum_step_frequency": 1,
            "second_maximum_step_frequency": 1,
            "acceleration_ramp": 10000,
            "break_ramp": 10000,
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
            self.commander.write_command(self.motor_address, command)
            print('Initializing ' + key + ' of motor ' + str(self.motor_address) + ' to ' + str(self._ram_record[key]))

    def step_speed(self, value):
        command = self._command_letters["maximum_step_frequency"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
    step_speed = property(None, step_speed)

    def step_distance(self, value):
        command = self._command_letters["travel_distance"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
    step_distance = property(None, step_distance)

    def direction(self, value):
        command = self._command_letters["direction_of_rotation"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
    direction = property(None, direction)

    def mode(self, value):
        if value == "relative_positioning":
            number = 1
        elif value == "absolute_positioning":
            number = 2
        elif value == "joystick_mode":
            number = 12
        elif value == "speed_mode":
            number = 5
        elif value == "external_reference_run":
            number = 4
        else:
            raise ValueError('Unsupported mode.')
        command = self._command_letters["position_mode"] + (str(number).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
    mode = property(None, mode)

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
        self.commander.write_command(self.motor_address, command)
    ramp_type = property(None, ramp_type)

    def jerk(self, value):
        command = self._command_letters["maximum_jerk_for_acceleration_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        command = self._command_letters["maximum_jerk_for_break_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
    jerk = property(None, jerk)

    def step_position(self):
        command = self._command_letters["step_position"]
        position_answer_byte = self.commander.write_command(self.motor_address, command)
        position_answer_str = position_answer_byte.decode('UTF-8')
        if "+" in position_answer_str:
            position_str = position_answer_str[2:]
        else:
            position_str = position_answer_str[1:]
        return int(position_str)
    step_position = property(step_position, None)

    def is_referenced(self):
        command = self._command_letters["is_referenced"]
        position_answer_byte = self.commander.write_command(self.motor_address, command)
        position_answer_str = position_answer_byte.decode('UTF-8')
        if "1" in position_answer_str[1:]:
            return True
        else:
            return False
    is_referenced = property(is_referenced, None)

    def is_ready(self):
        command = self._command_letters["status"]
        status_answer_byte = self.commander.write_command(self.motor_address, command)
        status_answer_str = status_answer_byte.decode('UTF-8').split("$")
        status_answer_int = int(status_answer_str[1])
        if status_answer_int & 1:
            return True
        else:
            return False
    is_ready = property(is_ready, None)

    def run(self):
        # start the motor with the current settings
        self.commander.write_command(self.motor_address, b'A')

    def stop(self):
        # stop the motor with the current ramp
        self.commander.write_command(self.motor_address, b'S1')

    def immediate_stop(self):
        # stop the motor with the current stop ramp
        self.commander.write_command(self.motor_address, b'S0')


class LinearMotor(NanotecPd6Motor):

    def __init__(self, commander, motor_address, distance_per_motor_revolution=0.12, steps_per_motor_revolution=200, micro_steps_per_step=2):
        super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step)

        self.distance_per_motor_revolution = distance_per_motor_revolution

    def distance(self, value):
        # convert from physical distance in meters to (micro) steps of the motor
        self.step_distance = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self.distance_per_motor_revolution)
    distance = property(None, distance)

    def speed(self, value):
        # convert from physical distance in meters per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self.distance_per_motor_revolution)
    speed = property(None, speed)

    @property
    def position(self):
        return float(self.step_position) / self.micro_steps_per_step / self.steps_per_motor_revolution * self.distance_per_motor_revolution


class RotationMotor(NanotecPd6Motor):

    def __init__(self, commander, motor_address, angle_per_motor_revolution, steps_per_motor_revolution=200, micro_steps_per_step=2):
        super().__init__(commander, motor_address, steps_per_motor_revolution, micro_steps_per_step)

        self.angle_per_motor_revolution = angle_per_motor_revolution

    def angle(self, value):
        # convert from physical angle in radians to (micro) steps of the motor
        self.step_distance = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self.angle_per_motor_revolution)
    angle = property(None, angle)

    def speed(self, value):
        # convert from physical speed in radians per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_motor_revolution * value / self.angle_per_motor_revolution)
    speed = property(None, speed)


class Slider(LinearMotor):

    def __init__(self, commander, motor_address, position_offset, distance_per_motor_revolution=0.12, steps_per_motor_revolution=200, micro_steps_per_step=2):
        super().__init__(commander, motor_address, distance_per_motor_revolution, steps_per_motor_revolution, micro_steps_per_step)

        self.position_offset = position_offset

    def get_absolute_position(self):
        return self.position + self.position_offset

    def set_absolute_position(self, value):
        self.distance = value - self.position_offset

    absolute_position = property(get_absolute_position, set_absolute_position)


class Horizontal_slider(Slider):

    def __init__(self, commander, motor_address, position_offset, distance_per_motor_revolution=0.12, steps_per_motor_revolution=200, micro_steps_per_step=2):
        super().__init__(commander, motor_address, position_offset, distance_per_motor_revolution, steps_per_motor_revolution, micro_steps_per_step)

    # def direction(self, value):
    #     if value == "Out":
    #         super().direction = 1
    #     elif value == "In":
    #         super().direction = 0
    #     else:
    #         raise ValueError('Direction of horizontal slider needs to be In or Out.')
    # direction = property(None, direction)

    def reference_run(self):
        if not self.is_referenced:

            self.mode = "external_reference_run"
            self.distance = 1       # m
            self.speed = 0.02       # m/s
            self.direction = 0

            self.run()

            while not self.is_referenced:
                time.sleep(1)

            time.sleep(1)
