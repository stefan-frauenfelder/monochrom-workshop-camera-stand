
import math

# for circular motion, the following assumptions apply
# - alpha is the angle of the circular motion with a given radius around a center with a given distance from the origin
# - alpha is given as a funktion of time as alpha = k * t


def circular_motion_arm_position(alpha, distance, radius):

    return math.sqrt(pow(distance, 2) + pow(radius, 2) - (2 * distance * radius * math.cos(math.pi - alpha)))


def circular_motion_rotor_angle(alpha, distance, radius):

    return math.atan(math.sin(alpha) * radius / (radius * math.cos(alpha) + distance))


def circular_motion_arm_speed(t, k, a_0, distance, radius):

    nominator = - distance * radius * k * math.sin(k * t + a_0)

    denominator = math.sqrt(pow(distance, 2) + 2 * distance * radius * math.cos(k * t + a_0) + pow(radius, 2))

    return nominator / denominator


def circular_motion_rotor_speed(t, k, a_0, distance, radius):

    nominator = k * radius * (distance * math.cos(k * t + a_0) + radius)

    denominator = 2 * distance * radius * math.cos(k * t + a_0) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator
