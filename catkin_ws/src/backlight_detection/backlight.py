import pysolar
import datetime
import pytz
import warnings
from math import atan, radians, sin, cos, degrees
from sympy.geometry import Point, Plane, Segment
from scipy.spatial.transform import Rotation as R

# Timezone
LOCAL_TIMEZONE = pytz.timezone('Europe/Moscow')  # Timezone

# Sensor parameters
SENSOR_HEIGHT = 24
SENSOR_WIDTH = 32
SENSOR_FOCAL = 35

# Sample data
SPCRAS_LOCATION = (59.939771653721714, 30.269367826467633)  # Sample location
SAMPLE_DATETIME = datetime.datetime(2021, 8, 12, 15, 0, 0)  # Sample datetime (12.08.2021 15:00) (NOT LOCALIZED!)


def get_sun_position(latitude, longitude, date_and_time):
    """Given the gps coordinates and a datetime object,
    calculate angular position of the Sun for this location and time
    :param latitude: absolute gps latitude
    :param longitude: absolute gps longitude
    :param datetime: datetime
    :return Tuple(azimuth, altitude):
        azimuth is a horisontal angle between north direction and the sun
        altitude is a vertical angle between horisontal plane and the sun
    """
    localized_datetime = LOCAL_TIMEZONE.localize(date_and_time)  # Add timezone information
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", category=UserWarning)
        sun_position = pysolar.solar.get_position(latitude, longitude, localized_datetime)  # Calculate using pysolar
    return sun_position


def calculate_camera_angles(h, w, f):
    """
    Given the geometric properties of a camera sensor, calculate it's view angles
    :param h: height of sensor in millimeters
    :param w: width of sensor in millimeters
    :param f: focal length of lens
    :return: Tuple(vertical_angle, horisontal_angle):
        vertical_angle - angle of vertical view in degrees
        horisontal_angle - angle of horisontal view in degrees
    """
    vertical_angle = 2 * atan(h / (2 * f))
    horisontal_angle = 2 * atan(w / (2 * f))
    return vertical_angle, horisontal_angle


def spherical_to_unit_cartesian(phi, theta):
    """
    Transform two angles in spherical coordinate system to three bases of cartesian coordinate system
    :param theta: a vertical angle between the point and positive "Z" (e.g. zenith angle of the Sun)
    :param phi: a horisontal angle between the point and positive "X" (e.g. azimuth of the Sun)
    """
    x = sin(phi) * cos(theta)
    y = sin(phi) * sin(theta)
    z = cos(phi)
    return x, y, z


def calculate_decision_angle(latitude, longitude, date_and_time, euler_x, euler_y, euler_z):
    """
    Given global object coordinates, date and time, and euler angles,
    calculate an angle between the sun and the camera sight ray.
    If this angle is less than minimal camera angle, the sun is in camera sight and the frame is backlit.
    The problem is solved on 3d unit spherical and cartesian plane.
    :param latitude: global latitude
    :param longitude: global longitude
    :param date_and_time: datetime without tzdata
    :param euler_x: euler angle X
    :param euler_y: euler angle Y
    :param euler_z: euler angle Z
    :return: decision_angle: an angle between the sun and the camera sight line
    """
    # Define a unit cartesian plane
    zero_point = Point(0, 0, 0)
    horisontal_plane = Plane(zero_point, normal_vector=(0, 0, 1))  # TODO

    # Calculate sun position, spherical and cartesian coordinates
    sun_azimuth, sun_altitude = get_sun_position(latitude, longitude, date_and_time)
    sun_zenith = 90 - sun_altitude
    sun_azimuth_rad, sun_altitude_rad, sun_zenith_rad = map(radians, [sun_azimuth, sun_altitude, sun_zenith])
    sun_x, sun_y, sun_z = spherical_to_unit_cartesian(sun_zenith_rad, sun_azimuth_rad)

    # Define a point and a unit vector representing the sun
    sun_position = Point(sun_x, sun_y, sun_z)
    sun_segment = Segment(zero_point, sun_position)

    # Define camera position
    camera_starting_position = [0, 0, -1]  # Looks down by default
    rotation = R.from_euler('xyz', [euler_x, euler_y, euler_z], degrees=True)
    camera_final_position = rotation.apply(camera_starting_position).tolist()  # Apply positioning
    camera_final_point = Point(camera_final_position)  # Define a point
    camera_final_segment = Segment(zero_point, camera_final_point)  # Define a vector

    decision_angle = degrees(camera_final_segment.angle_between(sun_segment))  # Calculate angle
    return decision_angle


def is_backlight(camera_angles, decision_angle):
    return decision_angle < (min(camera_angles) / 2)


if __name__ == "__main__":
    # Sun position calculation. Expected:
    # (217.85716540935564, 40.16159931193142)
    position = get_sun_position(*SPCRAS_LOCATION, SAMPLE_DATETIME)
    print(position, sep=';  ')

    # Camera angles calculation. Expected:
    # Vertical: 37.84928883210247; Horisontal: 49.13434264120263
    vertical_angle, horisontal_angle = map(degrees, calculate_camera_angles(SENSOR_HEIGHT,
                                                                            SENSOR_WIDTH,
                                                                            SENSOR_FOCAL))
    print("Vertical: {}; Horisontal: {}".format(vertical_angle, horisontal_angle))

    # Two simple models preparation.
    sample_latitude, sample_longitude = SPCRAS_LOCATION
    # Model 1: looking down. No rotation. Expecting decision angle to be about 130 degrees, is_backlight = False
    decision_angle = calculate_decision_angle(sample_latitude, sample_longitude, SAMPLE_DATETIME, 0, 0, 0)
    print(decision_angle)
    print("Backlight: {}".format(is_backlight((vertical_angle, horisontal_angle), decision_angle)))
    # Model 2: looking almost directly at the sun. Rotate 225 degrees by Y, then 225 degrees by Z.
    # Expecting decision angle to be about 7 degrees, is_backlight = True.
    decision_angle = calculate_decision_angle(sample_latitude, sample_longitude, SAMPLE_DATETIME, 0, 225, 225)
    print(decision_angle)
    print("Backlight: {}".format(is_backlight((vertical_angle, horisontal_angle), decision_angle)))


