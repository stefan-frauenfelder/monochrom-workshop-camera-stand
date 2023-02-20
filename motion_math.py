
import math

# for circular motion, the following assumptions apply
# - alpha is the angle of the circular motion with a given radius around a center with a given distance from the origin
# - alpha is given as a funktion of time as alpha = k * t


def circular_motion_arm_position(alpha, distance, radius):

    return math.sqrt(pow(distance, 2) + pow(radius, 2) - (2 * distance * radius * math.cos(math.pi - alpha)))


def circular_motion_rotor_angle(alpha, distance, radius):

    return math.atan(math.sin(alpha) * radius / (radius * math.cos(alpha) + distance))


def circular_motion_pan_angle(alpha, distance, radius):

    rotor_angle = circular_motion_rotor_angle(alpha, distance, radius)

    return alpha - rotor_angle - math.pi


def circular_motion_arm_speed(t, k, a_0, distance, radius):

    nominator = - distance * radius * k * math.sin(k * t + a_0)

    denominator = math.sqrt(pow(distance, 2) + 2 * distance * radius * math.cos(k * t + a_0) + pow(radius, 2))

    return nominator / denominator


def circular_motion_rotor_speed(t, k, a_0, distance, radius):

    nominator = k * radius * (distance * math.cos(k * t + a_0) + radius)

    denominator = 2 * distance * radius * math.cos(k * t + a_0) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator


def circular_motion_pan_speed(t, k, a_0, distance, radius):

    nominator = distance * k * (distance + radius * math.cos(k * t + a_0))

    denominator = 2 * distance * radius * math.cos(k * t + a_0) + pow(distance, 2) + pow(radius, 2)

    return nominator / denominator


def front_linear_motion_arm_position(s, distance):

    return math.sqrt(pow(distance, 2) + pow(s, 2))


def front_linear_motion_arm_speed(t, k, s_0, distance):

    nominator = k * (k * t + s_0)

    denominator = math.sqrt(pow(distance, 2) + pow((k * t + s_0), 2))

    return nominator / denominator


def front_linear_motion_rotor_pan_angle(s, distance):

    return math.atan(s / distance)


def front_linear_motion_rotor_pan_speed(t, k, s_0, distance):

    nominator = distance * k

    denominator = pow(distance, 2) + pow((k * t + s_0), 2)

    return nominator / denominator


def rectilinear_camera_coordinates(rotor_angle, arm_position):

    x = math.cos(rotor_angle) * arm_position
    y = math.sin(rotor_angle) * arm_position

    return x, y
