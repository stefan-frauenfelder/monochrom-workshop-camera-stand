import sys
import time

import smbus

# rgbbutton default i2c addr
RGBBUTTON_DEFAULT_I2C_ADDR = 0x23
# RGBButton chip ID
RGBBUTTON_PART_ID = 0x43DF

# RGBButton register address
# RGBButton I2C ADDR REG
RGBBUTTON_I2C_ADDR_REG = 0x00

# RGBButton RED REG
RGBBUTTON_RED_REG = 0x01
# RGBButton GREEN REG
RGBBUTTON_GREEN_REG = 0x02
# RGBButton BLUE REG
RGBBUTTON_BLUE_REG = 0x03

# RGBBUTTON BUTTON SIGNAL REG
RGBBUTTON_BUTTON_SIGNAL_REG = 0x04

# RGBButton PID MSB REG
RGBBUTTON_PID_MSB_REG = 0x09
# RGBButton PID LSB REG
RGBBUTTON_PID_LSB_REG = 0x0A


class RgbButton(object):
    e_red = 0xFF0000
    e_orange = 0xFF7F00
    e_yellow = 0xFFFF00
    e_green = 0x00FF00
    e_cyan = 0x00FFFF
    e_blue = 0x0000FF
    e_purple = 0x8B00FF
    e_white = 0xFFFFFF
    e_black = 0x000000

    def __init__(self, i2c_addr=RGBBUTTON_DEFAULT_I2C_ADDR):
        self._addr = i2c_addr
        self._i2c = smbus.SMBus(3)

    def begin(self):
        ret = True
        chip_id = self._read_reg(RGBBUTTON_PID_MSB_REG, 2)
        if RGBBUTTON_PART_ID != ((chip_id[0] << 8) | chip_id[1]):
            ret = False
        return ret

    def set_rgb_color(self, *args):
        rgb_buf = [0] * 3
        if 1 == len(args):
            rgb_buf[0] = (args[0] >> 16) & 0xFF
            rgb_buf[1] = (args[0] >> 8) & 0xFF
            rgb_buf[2] = args[0] & 0xFF
        elif 3 == len(args):
            rgb_buf[0] = args[0]
            rgb_buf[1] = args[1]
            rgb_buf[2] = args[2]
        self._write_reg(RGBBUTTON_RED_REG, rgb_buf)

    def get_button_status(self):
        button_status = False
        if 1 == self._read_reg(RGBBUTTON_BUTTON_SIGNAL_REG, 1)[0]:
            button_status = True
        return button_status

    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = [data]
        try:
            self._i2c.write_i2c_block_data(self._addr, reg, data)
        except IOError:
            print("[Errno 121] Remote I/O error")

    def _read_reg(self, reg, length):
        try:
            return self._i2c.read_i2c_block_data(self._addr, reg, length)
        except IOError:
            print("[Errno 121] Remote I/O error")
            return [0] * length
