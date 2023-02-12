import time
import pigpio
import smbus
from bitstring import BitArray

DEVICE_ADDRESS = 0x20  # 7 bit address (will be left shifted to add the read write bit)

INPORT_REG_ADD = 0x00
OUTPORT_REG_ADD = 0x01
POLINV_REG_ADD = 0x02
CFG_REG_ADD = 0x03

relayMaskRemap = [0x01, 0x04, 0x40, 0x10, 0x20, 0x80, 0x08, 0x02]
relayChRemap = [0, 2, 6, 4, 5, 7, 3, 1]

optoMaskRemap = [0x08, 0x04, 0x02, 0x01, 0x10, 0x20, 0x40, 0x80]
optoChRemap = [3, 2, 1, 0, 4, 5, 6, 7]

RELAY_STACK_LEVEL = 0x00
OPTO_STACK_LEVEL = 0x02


class SequentPorts():

    def __init__(self, interrupt_gpio_channel):

        self._sm_bus = smbus.SMBus(1)
        time.sleep(1)             # sm_bus needs to settle

        gpio = pigpio.pi()

        self._rising_edge_callbacks = []
        self._falling_edge_callbacks = []

        for i in range(8):
            self._rising_edge_callbacks.append(False)
            self._falling_edge_callbacks.append(False)

        # verify config of relay card, shall be 0
        hardware_address = DEVICE_ADDRESS + (0x07 ^ RELAY_STACK_LEVEL)

        cfg = self._sm_bus.read_byte_data(hardware_address, CFG_REG_ADD)
        if cfg != 0:
            self._sm_bus.write_byte_data(hardware_address, CFG_REG_ADD, 0x00)

        # turn all outputs off
        self._sm_bus.write_byte_data(hardware_address, OUTPORT_REG_ADD, 0x00)

        # verify config of opto card, shall be 0
        hardware_address = DEVICE_ADDRESS + (0x07 ^ OPTO_STACK_LEVEL)

        cfg = self._sm_bus.read_byte_data(hardware_address, CFG_REG_ADD)
        if cfg != 0xff:
            self._sm_bus.write_byte_data(hardware_address, CFG_REG_ADD, 0xff)

        # read from the inputs card for a firs time to clear the interrupt
        self._current_input_values = self.read_inputs()

        # listen to the interrupt line of the inputs card
        # .setup(interrupt_gpio_channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # GPIO.add_event_detect(interrupt_gpio_channel, GPIO.FALLING, callback=self.interrupt)

        gpio.set_mode(interrupt_gpio_channel, pigpio.INPUT)
        gpio.set_pull_up_down(interrupt_gpio_channel, pigpio.PUD_UP)
        gpio.callback(interrupt_gpio_channel, pigpio.FALLING_EDGE, self.interrupt)

    def interrupt(self, channel, level, tick):
        # read and clear the interrupt
        new_input_values = self.read_inputs()

        new_input_bit_array = BitArray(uint=new_input_values, length=8)
        current_input_bit_array = BitArray(uint=self._current_input_values, length=8)

        new_input_bit_string = new_input_bit_array.bin
        current_input_bit_string = current_input_bit_array.bin

        # edge detection
        input_number = 0

        for input_tuple in zip(current_input_bit_string, new_input_bit_string):

            # detect rising edge
            if input_tuple[0] == '0' and input_tuple[1] == '1':
                print('Input ' + str(input_number) + ' rising.')

                # check for rising edge callbacks
                if self._rising_edge_callbacks[input_number]:
                    self._rising_edge_callbacks[input_number]()

            # detect falling edge
            if input_tuple[0] == '1' and input_tuple[1] == '0':
                print('Input ' + str(input_number) + ' falling.')

                # check for falling edge callbacks
                if self._falling_edge_callbacks[input_number]:
                    self._falling_edge_callbacks[input_number]()

            input_number += 1

        self._current_input_values = new_input_values

    def read_inputs(self):

        hardware_address = DEVICE_ADDRESS + (0x07 ^ OPTO_STACK_LEVEL)

        io_value = self._sm_bus.read_byte_data(hardware_address, INPORT_REG_ADD)

        opto_value = 0
        for i in range(0, 8):
            if (io_value & optoMaskRemap[i]) == 0:
                opto_value = opto_value + (1 << i)

        return opto_value

    def set_output(self, relay, switch_state):

        hardware_address = DEVICE_ADDRESS + (0x07 ^ RELAY_STACK_LEVEL)

        current_io_value = self._sm_bus.read_byte_data(hardware_address, INPORT_REG_ADD)

        # remap from io to relay nr
        current_relay_value = 0
        for i in range(0, 8):
            if (current_io_value & relayMaskRemap[i]) != 0:
                current_relay_value = current_relay_value + (1 << i)

        # calculate the new state
        if switch_state == 0:
            new_relay_value = current_relay_value & (~(1 << (relay - 1)))
        else:
            new_relay_value = current_relay_value | (1 << (relay - 1))

        # map back to io
        new_io_value = 0
        for i in range(0, 8):
            if (new_relay_value & (1 << i)) != 0:
                new_io_value = new_io_value + relayMaskRemap[i]

        self._sm_bus.write_byte_data(hardware_address, OUTPORT_REG_ADD, new_io_value)

    def add_callback(self, input, edge, function):

        if edge == 'RISING':
            self._rising_edge_callbacks[input] = function
        elif edge == 'FALLING':
            self._falling_edge_callbacks[input] = function
        else:
            raise ValueError('Edge needs to be either RISING or FALLING.')
