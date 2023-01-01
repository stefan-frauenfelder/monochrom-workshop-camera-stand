import smbus
import time

# bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

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

    def __init__(self):

        self._sm_bus = smbus.SMBus(1)     # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        time.sleep(1)             # sm_bus needs to settle

        # verify config of relay card, shall be 0
        stack_level = RELAY_STACK_LEVEL

        hardware_address = DEVICE_ADDRESS + (0x07 ^ stack_level)

        cfg = self._sm_bus.read_byte_data(hardware_address, CFG_REG_ADD)
        if cfg != 0:
            self._sm_bus.write_byte_data(hardware_address, CFG_REG_ADD, 0)

        # verify config of opto card, shall be 0
        stack_level = OPTO_STACK_LEVEL

        hardware_address = DEVICE_ADDRESS + (0x07 ^ stack_level)

        cfg = self._sm_bus.read_byte_data(hardware_address, CFG_REG_ADD)
        if cfg != 0xff:
            self._sm_bus.write_byte_data(hardware_address, CFG_REG_ADD, 0xff)

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
