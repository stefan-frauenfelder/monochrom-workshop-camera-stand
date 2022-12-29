
import math

# for circular motion, the following assumptions apply
# - alpha is the angle of the circular motion with a given radius around a center with a given distance from the origin
# - alpha is given as a funktion of time as alpha = k * t


def circular_motion_arm_position(alpha, distance, radius):

    return math.sqrt(pow(distance, 2) + pow(radius, 2) - (2 * distance * radius * math.cos(alpha)))


def circular_motion_rotor_angle(alpha, distance, radius):

    return math.atan(math.sin(alpha) * radius / (radius * math.cos(alpha) + distance))


def circular_motion_arm_speed(t, k, distance, radius):

    nominator = - distance * radius * k * math.sin(k * t)

    denominator = math.sqrt(pow(distance, 2) + 2 * distance * radius * math.cos(k * t) + pow(radius, 2))

    return nominator / denominator


def circular_motion_rotor_speed(t, k, distance, radius):

    nominator = k * radius * (distance * math.cos(k * t) + radius)

    denominator = 2 * distance * radius * math.cos(k * t) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator
