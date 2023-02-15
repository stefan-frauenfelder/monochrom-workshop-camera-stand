"""!
  @file DFRobot_CH423.py
  @brief This is a I2C to IO expansion board based on CH432, an universal remote I/O expansion chip with a two-wire serial interface. It features:
  @n 1. 8 Bi-directional input/output pin: GPIO0 ~ GPIO7;
  @n 2. 16 general output pins: GPO0~GPO15;
  @n 3. Support input level change interrupt: if GPIO pin level is not the same as the initial level, GPO15 will output a low level signal;
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author [Arya](xue.peng@dfrobot.com)
  @version  V1.0
  @date  2022-03-14
  @https://github.com/DFRobot/DFRobot_CH423
"""

import sys
import time
import smbus


class Ch423:
    ## Set system parameter command
    CH423_CMD_SET_SYSTEM_ARGS = (0x48 >> 1)
    ## Set low 8bit general-purpose output command
    CH423_CMD_SET_GPO_L = (0x44 >> 1)
    ## Set high 8bit general-purpose output command
    CH423_CMD_SET_GPO_H = (0x46 >> 1)
    ## Set bi-directional input/output pin command
    CH423_CMD_SET_GPIO = 0x30
    ## Set bi-directional input/output pin command
    CH423_CMD_READ_GPIO = (0x4D >> 1)

    ## General-purpose output pin, GPO0
    eGPO0 = 0
    ## General-purpose output pin, GPO1
    eGPO1 = 1
    ## General-purpose output pin, GPO2
    eGPO2 = 2
    ## General-purpose output pin, GPO3
    eGPO3 = 3
    ## General-purpose output pin, GPO4
    eGPO4 = 4
    ## General-purpose output pin, GPO5
    eGPO5 = 5
    ## General-purpose output pin, GPO6
    eGPO6 = 6
    ## General-purpose output pin, GPO7
    eGPO7 = 7
    ## General-purpose output pin, GPO8
    eGPO8 = 8
    ## General-purpose output pin, GPO9
    eGPO9 = 9
    ## General-purpose output pin, GPO10
    eGPO10 = 10
    ## General-purpose output pin, GPO11
    eGPO11 = 11
    ## General-purpose output pin, GPO12
    eGPO12 = 12
    ## General-purpose output pin, GPO13
    eGPO13 = 13
    ## General-purpose output pin, GPO14
    eGPO14 = 14
    ## General-purpose output pin, GPO15
    eGPO15 = 15
    ## Total number of general-purpose pins
    eGPO_TOTAL = 16
    ## Bi-directional input/output pin, GPIO0
    eGPIO0 = 0
    ## Bi-directional input/output pin, GPIO1
    eGPIO1 = 1
    ## Bi-directional input/output pin, GPIO2
    eGPIO2 = 2
    ## Bi-directional input/output pin, GPIO3
    eGPIO3 = 3
    ## Bi-directional input/output pin, GPIO4
    eGPIO4 = 4
    ## Bi-directional input/output pin, GPIO5
    eGPIO5 = 5
    ## Bi-directional input/output pin, GPIO6
    eGPIO6 = 6
    ## Bi-directional input/output pin, GPIO7
    eGPIO7 = 7
    ## Total number of Bi-directional input/output pin
    eGPIO_TOTAL = 8

    ## Bi-directional input/output pin, GPIO0~GPIO7
    eGPIO = 0
    ## General output pin, GPO0~GPO15
    eGPO = 1
    ## General output pin, GPO0~GPO7
    eGPO0_7 = 2
    ## General output pin, GPO8~GPO15
    eGPO8_15 = 3

    ## GPIO pin input mode, at high level when floating
    eINPUT = 0
    ## GPIO pin output mode, can output high/low level
    eOUTPUT = 1
    ## GPO pin open-drain output mode, can only be used for eGPO0_7 and eGPO8_15 digital ports. And GPO can only output low level or do not output in this mode
    eOPEN_DRAIN = 2
    ## GPO pin push-pull output mode, can only be used for eGPO0_7 and eGPO8_15 digital ports. And GPO can output high or low level.
    ePUSH_PULL = 3

    ## configure pin interrupt, low level interrupt
    eLOW = 0
    ## configure pin interrupt, high level interrupt
    eHIGH = 1
    ## configure pin interrupt, rising edge interrupt
    eRISING = 2
    ## configure pin interrupt, falling edge interrupt
    eFALLING = 3
    ## configure pin interrupt, double edge interrupt
    eCHANGE = 4

    ARGS_BIT_IO_EN = 0
    ARGS_BIT_DEC_L = 1
    ARGS_BIT_DEC_H = 2
    ARGS_BIT_INT_EN = 3
    ARGS_BIT_OD_EN = 4
    ARGS_BIT_SLEEP = 6

    def __init__(self):
        # Get I2C bus
        self._bus = smbus.SMBus(3)
        self._args = 0
        self.enable_interrupt()

    def gpio_write(self, level):
        self._bus.write_byte(self.CH423_CMD_SET_GPIO, level)

    def gpio_read(self):
        return self._read_gpio()

    def enable_interrupt(self):
        # set output to current input since interrupts are triggered by comparison
        value = self.gpio_read()
        self.gpio_write(value)
        self._args = 0
        self._args |= (1 << self.ARGS_BIT_INT_EN)
        self._set_system_args()

    def _set_system_args(self):
        self._bus.write_byte(self.CH423_CMD_SET_SYSTEM_ARGS, self._args)

    def _read_gpio(self):
        return self._bus.read_byte(self.CH423_CMD_READ_GPIO)
