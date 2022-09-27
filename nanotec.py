
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

        print('Invoced ' + command.decode('UTF-8') + ' for motor ' + str(address) + ' received answer ' + answer.decode('UTF-8').rstrip('\r'))  # print


class NanotecPd6Motor():
    def __init__(self, commander, motor_address=1, steps_per_revolution=200, micro_steps_per_step=2):
        self.commander = commander
        self.motor_address = motor_address
        self.steps_per_revolution = steps_per_revolution
        self.micro_steps_per_step = micro_steps_per_step
        self._step_speed = 10
        self._step_distance = 10
        self._step_acceleration = 10000
        self._direction = 0
        self._jerk = 1

        self._command_letters = {
            "position_mode": b'p',
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
        }
        self._ram_record = {
            "position_mode": 1,
            "travel_distance": self._step_distance,
            "initial_step_frequency": 1,
            "maximum_step_frequency": self._step_speed,
            "second_maximum_step_frequency": self._step_speed,
            "acceleration_ramp": self._step_acceleration,
            "break_ramp": self._step_acceleration,
            "direction_of_rotation": self._direction,
            "reversal_of_direction_of_rotation_for_repeat_records": 0,
            "repetitions": 1,
            "pause": 0,
            "record_number_of_continuation_record": 0,
            "maximum_jerk_for_acceleration_ramp": self.jerk,
            "maximum_jerk_for_break_ramp": self._jerk,
            "joystick_dead_range": 5,
            "joystick_filter": 16,
            "limit_switch_behavior": 9250,  # default + ext.lim = stop
            "input_1": 7,  # external reference switch
            "input_2": 7   # external reference switch

        }

        for key in self._ram_record:
            command = self._command_letters[key] + (str(self._ram_record[key]).encode('UTF-8'))
            self.commander.write_command(self.motor_address, command)
            print('Initializing ' + key + ' of motor ' + str(self.motor_address) + ' to ' + str(self._ram_record[key]))

    @property
    def step_speed(self):
        return self._step_speed

    @step_speed.setter
    def step_speed(self, value):
        command = self._command_letters["maximum_step_frequency"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        self._step_speed = value

    @property
    def step_distance(self):
        return self._step_distance

    @step_distance.setter
    def step_distance(self, value):
        command = self._command_letters["travel_distance"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        self._step_distance = value

    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, value):
        command = self._command_letters["direction_of_rotation"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        self._direction = value

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        if value == "relative_positioning":
            number = 1
        elif value == "absolute_positioning":
            number = 2
        elif value == "joystick_mode":
            number = 12
        elif value == "speed_mode":
            number = 5
        else:
            number = 1
        command = self._command_letters["position_mode"] + (str(number).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        self._mode = value

    @property
    def ramp_type(self):
        return self._ramp_type

    @ramp_type.setter
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
        self._ramp_type = value

    @property
    def jerk(self):
        return self._jerk

    @jerk.setter
    def jerk(self, value):
        command = self._command_letters["maximum_jerk_for_acceleration_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        command = self._command_letters["maximum_jerk_for_break_ramp"] + (str(value).encode('UTF-8'))
        self.commander.write_command(self.motor_address, command)
        self._jerk = value

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

    def __init__(self, commander, motor_address, distance_per_revolution, steps_per_revolution=200, micro_steps_per_step=2):
        super().__init__(commander, motor_address, steps_per_revolution, micro_steps_per_step)

        self._distance = 0.01  # meters
        self._speed = 0.01  # meters/second
        self.distance_per_revolution = distance_per_revolution

    @property
    def distance(self):
        return self._distance

    @distance.setter
    def distance(self, value):
        # convert from physical distance in meters to (micro) steps of the motor
        self.step_distance = int(self.micro_steps_per_step * self.steps_per_revolution * value / self.distance_per_revolution)
        self._distance = value  # update value (this is not nice, should be a write-only property)

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        # convert from physical distance in meters per second to (micro) steps per second of the motor
        self.step_speed = int(self.micro_steps_per_step * self.steps_per_revolution * value / self.distance_per_revolution)
        self._speed = value  # update value (this is not nice, should be a write-only property)
