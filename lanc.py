
import time
import pigpio

LANC_START_STOP_CMD = (0x18, 0x33)  # might be start / stop recording
LANC_REC_CMD = (0x18, 0x3A)
LANC_STOP_CMD = (0x18, 0x30)

BIT_DURATION = 0.000104  # 104 microseconds
HALF_BIT_DURATION = 0.000052  # 52 microseconds


class LanC:

    def __init__(self, receive_pin, send_pin):

        self.gpio = pigpio.pi()
        self.receive_pin = receive_pin
        self.send_pin = send_pin

        self.gpio.set_mode(self.receive_pin, pigpio.INPUT)
        self.gpio.set_mode(self.send_pin, pigpio.OUTPUT)
        self.gpio.write(self.send_pin, 0)

        self.high_timer = 0.0  # counting microseconds of high signal time
        self.high_start_tick = 0.0  # remembering when the signal has become high

    def rising_edge_callback(self, channel, level, tick):
        self.high_start_tick = tick

    def falling_edge_callback(self, channel, level, tick):
        self.high_timer = tick - self.high_start_tick

    def wait_for_start(self):
        # the start bit follows a period of > 5 milliseconds high signal
        self.high_timer = 0.0
        # activate callbacks
        rising_edge_cb = self.gpio.callback(self.receive_pin, pigpio.RISING_EDGE, self.rising_edge_callback)
        falling_edge_cb = self.gpio.callback(self.receive_pin, pigpio.FALLING_EDGE, self.falling_edge_callback)

        while self.high_timer < 5000:
            pass  # let the interrupts and callbacks do their work

        # signal has now been high for > 5 milliseconds
        rising_edge_cb.cancel()
        falling_edge_cb.cancel()

        # now, this is why this whole thing doesn't work at all:
        # we need to be able to detect the falling edge after the > 5 ms high period,
        # and return to the calling function within less than 100 microseconds.
        # I couldn't find a way to do that in python. Both pigpio an RPi.GPIO require around
        # 1000 microseconds or more to detect an edge change.
        # Better use an arduino for LAN-C

    def send_command(self, command_tuple):
        bytes_to_send = [[],[]]
        # create a formatting specification to convert e.g. 0x33 to '00110011'
        spec = '{fill}{align}{width}{type}'.format(fill='0', align='>', width=8, type='b')

        for index in range(2):
            # reformat the command bytes to a binary string representation
            command_binary_sting = format(command_tuple[index], spec)
            # reverse the string since it has to be sent LSB first
            bytes_to_send[index] = command_binary_sting[::-1]

        # prepare the wave forms to send
        pulses = [[], []]
        waves = []
        self.gpio.wave_clear()
        bit_delay = 52  # the bit width in microseconds (52 instead of 104 for unknown reason)
        for byte_number in range(2):  # only the first two bytes of a message are used for sending
            for bit in range(8):
                if bytes_to_send[byte_number][bit]:  # if there is a logic high to send:
                    pulses[byte_number].append(pigpio.pulse(1 << self.send_pin, 0, bit_delay))  # set the pin high, pulling the signal to GND
                else:  # send logic zero
                    pulses[byte_number].append(pigpio.pulse(0, 1 << self.send_pin, bit_delay))  # to send logic zero, release the pin, let signal rise
            pulses[byte_number].append(pigpio.pulse(0, 1 << self.send_pin, 0))  # always end with low
            # do the wave creation from the pulses
            self.gpio.wave_add_generic(pulses[byte_number])
            waves.append(self.gpio.wave_create())

        for repetition in range(4):  # LANC commands must be repeated 4 times in order to be accepted by the camera
            self.wait_for_start()  # wait for the next start-bit of a new lanc message
            # loop over the 8 bytes making up one message
            for byte_number in range(8):  # only the first two bytes of a message are used for sending
                while self.gpio.read(self.receive_pin) == 0:  # wait for the start bit to be over (signal rise)
                    pass
                if byte_number < 2:  # only the first two bytes of a message are used for sending
                    # send the wave
                    self.gpio.wave_send_once(waves[byte_number])

                time.sleep(HALF_BIT_DURATION)  # make sure to be in the stop bit before waiting for next start
                while self.gpio.read(self.receive_pin) == 1:  # wait for next start
                    pass
