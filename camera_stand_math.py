
import math


def arm_extension_speed(alpha, distance, radius):

    nominator = distance * radius * math.sin(alpha)

    denominator = math.sqrt(pow(distance, 2) + 2 * distance * radius * math.cos(alpha) + pow(radius, 2))

    return nominator / denominator


def arm_extension(alpha, distance, radius):

    return math.sqrt(pow(distance, 2) + pow(radius, 2) - (2 * distance * radius * math.cos(alpha)))


def beta(alpha, distance, radius, arm_extension):

    return math.asin(math.sin(alpha) * radius / arm_extension)


def angular_speed(alpha, distance, radius):

    # the following line is the actual formula from wolfram alpha, but it does not work
    # nominator = math.copysign(1, math.sin(alpha)) * radius * (distance * math.cos(alpha) - radius)

    # the following line is working for the circle center on the beta = 0 axis
    nominator = radius * (distance * math.cos(alpha) - radius)

    denominator = -2 * distance * radius * math.cos(alpha) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator
