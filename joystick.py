import time
import json
import os

from ads1115 import ADS1115

# create an adc to be used by multiple joysticks, not sure this works
adc_from_outside = ADS1115()

class Joystick:

    def __init__(self, adc_ports=(0, 1)):

        self.adc = adc_from_outside
        self.adc_ports = adc_ports
        self.x = adc_ports[0]
        self.y = adc_ports[1]
        self.dead_zone = 0.01  # what portion of the whole range is still considered neutral

        try:
            self.calibration = json.loads(open("joystick_calibration.json").read())
            print('Applied existing joystick calibration.')
        except os.error:
            self.calibration = None

    def get_voltages(self):
        x_voltage = self.adc.read_voltage(self.x)
        y_voltage = self.adc.read_voltage(self.y)
        # return as a tuple
        return x_voltage, y_voltage

    def get_position(self):

        # initialize output as a list
        position = [0, 0]

        if self.calibration:

            # read the current voltage values
            voltages = self.get_voltages()

            for axis in range(2):

                # access the single axis
                value = voltages[axis]
                cal = self.calibration[axis]

                # local variables for readability
                minimum = cal['minimum_voltage']
                neutral = cal['neutral_voltage']
                maximum = cal['maximum_voltage']

                full_span = maximum - minimum
                half_dead_zone = full_span * self.dead_zone / 2

                # check for exactly neutral
                if value == neutral:
                    output = 0.0
                # check for lower zone
                elif value < neutral:
                    # check for lower dead zone
                    if value > neutral - half_dead_zone:
                        output = 0.0
                    # needs to be lower active zone
                    else:
                        lower_active_span = (neutral - half_dead_zone) - minimum
                        output = -1.0 + (value - minimum) / lower_active_span
                # needs to be upper zone
                else:
                    # check for upper dead zone
                    if value < neutral + half_dead_zone:
                        output = 0.0
                    # needs to be upper active zone
                    else:
                        upper_active_span = maximum - (neutral + half_dead_zone)
                        output = (value - (neutral + half_dead_zone)) / upper_active_span

                # limit to avoid corner cases exceeding the valid range
                output = max(output, -1.0)
                output = min(output, 1.0)
                position[axis] = output
        else:
            print('This joystick is uncalibrated and can not be used.')

        return tuple(position)

    def calibrate(self):

        print('Release the Joystick to its neutral position...')

        print('Sampling in ...')
        for counter in range(5):
            print(5-counter)
            time.sleep(1)
        initial_neutral = self.get_voltages()
        print('Sampled.')
        time.sleep(1)


        print('Now move the Joystick to all its limits in all directions...')
        for counter in range(4):
            print(4-counter)
            time.sleep(1)
        print('Go!')
        time.sleep(0.5)

        # initialize limit values
        x_min = initial_neutral[0]
        x_max = x_min
        y_min = initial_neutral[1]
        y_max = y_min

        start_time = time.time()
        duration = 8             # seconds
        t = 0

        while t < duration:
            # update time
            t = time.time() - start_time
            # read new values
            (x, y) = self.get_voltages()
            # get extremes
            x_min = min(x_min, x)
            x_max = max(x_max, x)
            y_min = min(y_min, y)
            y_max = max(y_max, y)
            # sample with approx. 20Hz
            time.sleep(0.05)
            print(str(x) + ', ' + str(y))

        print('Now Release the Joystick to its neutral position again...')

        print('Sampling in ...')
        for counter in range(5):
            print(5 - counter)
            time.sleep(1)
        end_neutral = self.get_voltages()
        print('Sampled.')
        time.sleep(1)

        x_neutral = (initial_neutral[0] + end_neutral[0]) / 2
        y_neutral = (initial_neutral[1] + end_neutral[1]) / 2

        self.calibration = [{
            'minimum_voltage': x_min,
            'neutral_voltage': x_neutral,
            'maximum_voltage': x_max,
        }, {
            'minimum_voltage': y_min,
            'neutral_voltage': y_neutral,
            'maximum_voltage': y_max
        }]

        with open('joystick_calibration.json', 'w') as file:
            json.dump(self.calibration, file, indent=4)

        print('The joystick is now calibrated')



