# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

# Non-change to try the hook

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


class Commander():
    def __int__(self, ser, port='COM6', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = ser

    def __del__(self):
        self.ser.close()  # close port

    def write_command(self, address, command):
        self.ser.write(b'#' + (str(address)).encode('UTF-8') + command + b'\r')  # set positioning to relative
        cc = self.ser.read_until(b'\r')  # read until '\r' appears


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
        self.jerk = 10

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
            "maximum_jerk_for_break_ramp": b':B'
        }
        self._ram_record = {
            "position_mode": 1,
            "travel_distance": self._step_distance,
            "initial_step_frequency": 0,
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
            "maximum_jerk_for_break_ramp": self.jerk
        }

        for key in self._ram_record:
            command = self._command_letters[key] + (str(self._ram_record[key]).encode('UTF-8'))
            self.commander.write_command(self.motor_address, command)  # set positioning to relative
            print('Initializing ' + key + ' of motor ' + str(self.motor_address) + ' to ' + str(self._ram_record[key]))  # print

    @property
    def step_speed(self):
        return self._step_speed

    @step_speed.setter
    def step_speed(self, value):
        command = self._command_letters["maximum_step_frequency"] + (str(value).encode('UTF-8'))  # assemble the command
        self.commander.write_command(self.motor_address, command)  # write to the ram of the motor
        self._step_speed = value  # update private attribute

    @property
    def step_distance(self):
        return self._step_distance

    @step_distance.setter
    def step_distance(self, value):
        command = self._command_letters["travel_distance"] + (str(value).encode('UTF-8'))  # assemble the command
        self.commander.write_command(self.motor_address, command)  # write to the ram of the motor
        self._step_distance = value  # update private attribute

    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, value):
        command = self._command_letters["direction_of_rotation"] + (str(value).encode('UTF-8'))  # assemble the command
        self.commander.write_command(self.motor_address, command)  # write to the ram of the motor
        self._direction = value  # update private attribute

    def run(self):
        self.commander.write_command(self.motor_address, b'A')  # start the motor with the current settings


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


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    import serial

    serial_port = serial.Serial(port='COM6',
                                baudrate=115200,
                                bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                timeout=1)

    the_commander = Commander()  # create the singleton commander
    the_commander.ser = serial_port  # hand the serial port to the commander to use

    # the motor for the horizontal motion
    horizontal_slider = LinearMotor(commander=the_commander, motor_address=1, distance_per_revolution=0.12)

    # the motor for the vertical motion
    vertical_slider = LinearMotor(commander=the_commander, motor_address=2, distance_per_revolution=0.12)

    horizontal_slider.speed = 0.5     # m/s
    horizontal_slider.distance = 0.2    # m
    horizontal_slider.direction = 1     # 1 is out, 0 is in

    horizontal_slider.run()