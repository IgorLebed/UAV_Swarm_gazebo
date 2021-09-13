from math import cos, sin, atan2, sqrt

global x_3
global y_3

def lengths(x_0, y_0, x_1, y_1):
    return sqrt(x_0 ** 2 + y_0 ** 2), sqrt((x_1 - x_0) ** 2 + (y_1 - y_0) ** 2)


def angles(x_0, y_0, x_1, y_1):
    return atan2(y_0, x_0), atan2(y_1 - y_0, x_1 - x_0) - atan2(y_0, x_0)


def calculate_x(len_s, ang_s):
    """
    :param len_s: list or tuple of lengths
    :param ang_s: list or tuple of angles
    :return: X coordinate of circle center
    """
    return len_s[1] * (cos(ang_s[0] + ang_s[1]) - sin(ang_s[0] + ang_s[1])) + cos(ang_s[0]) * len_s[0]


def calculate_y(len_s, ang_s):
    """
    :param len_s: list or tuple of lengths
    :param ang_s: list or tuple of angles
    :return: Y coordinate of circle center
    """
    return len_s[1] * (cos(ang_s[0] + ang_s[1]) + sin(ang_s[0] + ang_s[1])) + sin(ang_s[0]) * len_s[0]


def main(x_0, y_0, x_1, y_1):
    """
    :param x_0: scout X position
    :param y_0: scout Y position
    :param x_1: mid point X position
    :param y_1: mid point Y position
    :return: coordinates of circle center
    """
    lns = lengths(x_0, y_0, x_1, y_1)
    thetas = angles(x_0, y_0, x_1, y_1)
    x_3 = calculate_x(lns, thetas)
    y_3 = calculate_y(lns, thetas)
    return x_3, y_3

