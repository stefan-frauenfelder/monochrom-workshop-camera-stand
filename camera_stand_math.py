
import math


def slider_speed(alpha, distance, radius):

    nominator = distance * radius * math.sin(alpha)

    denominator = math.sqrt(pow(distance, 2) + 2 * distance * radius * math.cos(alpha) + pow(radius, 2))

    return nominator / denominator


def arm_extension(alpha, distance, radius):

    return math.sqrt(pow(distance, 2) + pow(radius, 2) - (2 * distance * radius * math.cos(alpha)))


def beta(alpha, distance, radius, arm_extension):

    return math.asin(math.sin(alpha) * radius / arm_extension)


def angular_speed(alpha, distance, radius, arm_extension):

    nominator = radius * math.cos(alpha)

    denominator = math.sqrt(pow(arm_extension, 2) - pow(radius, 2) * pow(math.sin(alpha), 2))

    return nominator / denominator


def angular_speed_simple(alpha, distance, radius):

    nominator = math.copysign(1, math.sin(alpha)) * radius * (distance * math.cos(alpha) - radius)
    nominator = radius * (distance * math.cos(alpha) - radius)
    denominator = -2 * distance * radius * math.cos(alpha) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator


def angular_speed_real(alpha, distance, radius):

    nominator = math.sin(alpha) * abs(radius) * (distance * math.cos(alpha) - radius)

    denominator = abs(math.sin(alpha)) * (-2 * distance * radius * math.cos(alpha) + pow(distance, 2) + pow(radius, 2))

    return nominator / denominator
