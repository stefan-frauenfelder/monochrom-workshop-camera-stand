import smbus
import time


# I2C address of the device
ADS1115_IIC_ADDRESS0 = 0x48
ADS1115_IIC_ADDRESS1 = 0x49

# ADS1115 Register Map
# Conversion register
ADS1115_REG_POINTER_CONVERT = 0x00
# Configuration register
ADS1115_REG_POINTER_CONFIG = 0x01
# Lo_thresh register
ADS1115_REG_POINTER_LOWTHRESH = 0x02
# Hi_thresh register
ADS1115_REG_POINTER_HITHRESH = 0x03

# ADS1115 Configuration Register
# No effect
ADS1115_REG_CONFIG_OS_NOEFFECT = 0x00
# Begin a single conversion
ADS1115_REG_CONFIG_OS_SINGLE = 0x80
# Differential P = AIN0, N = AIN1 (default)
ADS1115_REG_CONFIG_MUX_DIFF_0_1 = 0x00
# Differential P = AIN0, N = AIN3
ADS1115_REG_CONFIG_MUX_DIFF_0_3 = 0x10
# Differential P = AIN1, N = AIN3
ADS1115_REG_CONFIG_MUX_DIFF_1_3 = 0x20
# Differential P = AIN2, N = AIN3
ADS1115_REG_CONFIG_MUX_DIFF_2_3 = 0x30
# Single-ended P = AIN0, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_0 = 0x40
# Single-ended P = AIN1, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_1 = 0x50
# Single-ended P = AIN2, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_2 = 0x60
# Single-ended P = AIN3, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_3 = 0x70
# +/-6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_6_144V = 0x00
# +/-4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_4_096V = 0x02
# +/-2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_2_048V = 0x04
# +/-1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_1_024V = 0x06
# +/-0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_512V = 0x08
# +/-0.256V range = Gain 16
ADS1115_REG_CONFIG_PGA_0_256V = 0x0A
# Continuous conversion mode
ADS1115_REG_CONFIG_MODE_CONTIN = 0x00
# Power-down single-shot mode (default)
ADS1115_REG_CONFIG_MODE_SINGLE = 0x01
# 8 samples per second
ADS1115_REG_CONFIG_DR_8SPS = 0x00
# 16 samples per second
ADS1115_REG_CONFIG_DR_16SPS = 0x20
# 32 samples per second
ADS1115_REG_CONFIG_DR_32SPS = 0x40
# 64 samples per second
ADS1115_REG_CONFIG_DR_64SPS = 0x60
# 128 samples per second (default)
ADS1115_REG_CONFIG_DR_128SPS = 0x80
# 250 samples per second
ADS1115_REG_CONFIG_DR_250SPS = 0xA0
# 475 samples per second
ADS1115_REG_CONFIG_DR_475SPS = 0xC0
# 860 samples per second
ADS1115_REG_CONFIG_DR_860SPS = 0xE0
# Traditional comparator with hysteresis (default)
ADS1115_REG_CONFIG_CMODE_TRAD = 0x00
# Window comparator
ADS1115_REG_CONFIG_CMODE_WINDOW = 0x10
# ALERT/RDY pin is low when active (default)
ADS1115_REG_CONFIG_CPOL_ACTVLOW = 0x00
# ALERT/RDY pin is high when active
ADS1115_REG_CONFIG_CPOL_ACTVHI = 0x08
# Non-latching comparator (default)
ADS1115_REG_CONFIG_CLAT_NONLAT = 0x00
# Latching comparator
ADS1115_REG_CONFIG_CLAT_LATCH = 0x04
# Assert ALERT/RDY after one conversion
ADS1115_REG_CONFIG_CQUE_1CONV = 0x00
# Assert ALERT/RDY after two conversions
ADS1115_REG_CONFIG_CQUE_2CONV = 0x01
# Assert ALERT/RDY after four conversions
ADS1115_REG_CONFIG_CQUE_4CONV = 0x02
# Disable the comparator and put ALERT/RDY in high state (default)
ADS1115_REG_CONFIG_CQUE_NONE = 0x03


class ADS1115:

    def __init__(self, range=None):

        self.address = ADS1115_IIC_ADDRESS0
        self.coefficient = None
        if range:
            self.gain = range
        else:  # default
            self.gain = ADS1115_REG_CONFIG_PGA_6_144V
        self.coefficient_from_gain(self.gain)
        self.channel = 0

        # Get I2C bus
        self._bus = smbus.SMBus(3)

    def coefficient_from_gain(self, gain):
        if gain == ADS1115_REG_CONFIG_PGA_6_144V:
            self.coefficient = 0.1875
        elif gain == ADS1115_REG_CONFIG_PGA_4_096V:
            self.coefficient = 0.125
        elif gain == ADS1115_REG_CONFIG_PGA_2_048V:
            self.coefficient = 0.0625
        elif gain == ADS1115_REG_CONFIG_PGA_1_024V:
            self.coefficient = 0.03125
        elif gain == ADS1115_REG_CONFIG_PGA_0_512V:
            self.coefficient = 0.015625
        elif gain == ADS1115_REG_CONFIG_PGA_0_256V:
            self.coefficient = 0.0078125
        else:
            self.coefficient = 0.125

    def set_single(self):

        if self.channel == 0:
            config_register = [
                ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_0 | self.gain | ADS1115_REG_CONFIG_MODE_CONTIN,
                ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CQUE_NONE]
        elif self.channel == 1:
            config_register = [
                ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_1 | self.gain | ADS1115_REG_CONFIG_MODE_CONTIN,
                ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CQUE_NONE]
        elif self.channel == 2:
            config_register = [
                ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_2 | self.gain | ADS1115_REG_CONFIG_MODE_CONTIN,
                ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CQUE_NONE]
        elif self.channel == 3:
            config_register = [
                ADS1115_REG_CONFIG_OS_SINGLE | ADS1115_REG_CONFIG_MUX_SINGLE_3 | self.gain | ADS1115_REG_CONFIG_MODE_CONTIN,
                ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CQUE_NONE]

        self._bus.write_i2c_block_data(self.address, ADS1115_REG_POINTER_CONFIG, config_register)

    def read_value(self):
        # read from bus
        data = self._bus.read_i2c_block_data(self.address, ADS1115_REG_POINTER_CONVERT, 2)
        # Convert the data
        raw_adc = data[0] * 256 + data[1]
        if raw_adc > 32767:
            raw_adc -= 65535
        # convert to milli volts
        value = int(float(raw_adc) * self.coefficient)
        return value

    def read_voltage(self, channel):
        self.channel = channel
        self.set_single()
        time.sleep(0.02)
        return float(self.read_value()) / 1000
