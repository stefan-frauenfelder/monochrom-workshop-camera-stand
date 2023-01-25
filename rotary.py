# Rotary encoder class based on pigpio library
# version: 0.2.5

try:
    import pigpio
except ModuleNotFoundError:
    import sys
    from unittest.mock import MagicMock
    sys.modules['pigpio'] = MagicMock()


import time

# States
a_gpio1 = 'D'  # a_gpio is high
a_gpio0 = 'd'  # a_gpio is low
b_gpio1 = 'C'  # b_gpio is high
b_gpio0 = 'c'  # b_gpio is low

# State sequences
SEQUENCE_UP = a_gpio1 + b_gpio1 + a_gpio0 + b_gpio0
SEQUENCE_DOWN = b_gpio1 + a_gpio1 + b_gpio0 + a_gpio0


class RotaryEncoder:

    sequence = ''

    # Default values for the rotary encoder
    _min = 0
    _max = 100
    scale = 1
    debounce = 300
    _counter = 0
    last_counter = 0
    rotary_callback = None

    wait_time = time.time()
    long = False

    def __init__(self, gpios, b_gpio=None, a_gpio=None, debug=False):
        if not (b_gpio and a_gpio):
            raise BaseException("b_gpio and a_gpio pin must be specified!")

        self.DEBUG = debug
        self.pi = gpios
        self.b_gpio = b_gpio
        self.a_gpio = a_gpio
        self.pi.set_glitch_filter(self.b_gpio, self.debounce)
        self.pi.set_glitch_filter(self.a_gpio, self.debounce)
        self.setup_pigpio_callbacks()

    def setup_pigpio_callbacks(self):
        self.pi.callback(self.b_gpio, pigpio.FALLING_EDGE, self.b_gpio_fall)
        self.pi.callback(self.b_gpio, pigpio.RISING_EDGE, self.b_gpio_rise)
        self.pi.callback(self.a_gpio, pigpio.FALLING_EDGE, self.a_gpio_fall)
        self.pi.callback(self.a_gpio, pigpio.RISING_EDGE, self.a_gpio_rise)

    @property
    def counter(self):
        return self._counter

    @counter.setter
    def counter(self, value):
        if self._counter != value:
            self._counter = value
            if self.rotary_callback:
                self.rotary_callback(self._counter)

    @property
    def min(self):
        return self._min

    @min.setter
    def min(self, value):
        self._min = value
        if self._counter < value:
            self.counter = value

    @property
    def max(self):
        return self._max

    @max.setter
    def max(self, value):
        self._max = value
        if self._counter > value:
            self.counter = value

    def b_gpio_fall(self, _gpio, _level, _tick):
        if self.DEBUG:
            print(self.sequence + ':{}'.format(b_gpio1))
        if len(self.sequence) > 2:
            self.sequence = ''
        self.sequence += b_gpio1

    def b_gpio_rise(self, _gpio, _level, _tick):
        if self.DEBUG:
            print(self.sequence + ':{}'.format(b_gpio0))
        self.sequence += b_gpio0
        if self.sequence == SEQUENCE_UP:
            if self.counter < self._max:
                self.counter += self.scale
            self.sequence = ''

    def a_gpio_fall(self, _gpio, _level, _tick):
        if self.DEBUG:
            print(self.sequence + ':{}'.format(a_gpio1))
        if len(self.sequence) > 2:
            self.sequence = ''
        self.sequence += a_gpio1

    def a_gpio_rise(self, _gpio, _level, _tick):
        if self.DEBUG:
            print(self.sequence + ':{}'.format(a_gpio0))
        self.sequence += a_gpio0
        if self.sequence == SEQUENCE_DOWN:
            if self.counter > self._min:
                self.counter -= self.scale
            self.sequence = ''

    def setup_rotary(
        self,
        rotary_callback=None,
        min=None,
        max=None,
        scale=None,
        debounce=None,
    ):
        if not rotary_callback:
            print('At least one callback should be given')
        # rotary callback has to be set first since the self.counter property depends on it
        self.rotary_callback = rotary_callback
        if min is not None:
            self._min = min
            self.counter = self._min
            self.last_counter = self._min
        if max is not None:
            self._max = max
        if scale is not None:
            self.scale = scale
        if debounce is not None:
            self.debounce = debounce
            self.pi.set_glitch_filter(self.b_gpio, self.debounce)
            self.pi.set_glitch_filter(self.a_gpio, self.debounce)
