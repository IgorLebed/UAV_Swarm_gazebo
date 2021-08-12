# A dictionary containing data for my module to work and data my module will answer with
import datetime

sample_expected_data = {
    'drone_id': 123,  # Drone identification
    'altitude': 0.0,  # Float. Absolute altitude (gps).
    'longitude': 0.0,  # Float. Absolute longitude (gps).
    'date_and_time': datetime.datetime(2021, 8, 12, 17, 0, 0, 0),  # Datetime. Current local time to consider.
    'euler_angles': (0, 0, 0),  # Euler angles in absolute coordinates in degrees. Order: X angle, Y angle, Z angle (!!)
}

sample_answer_data = {
    'drone_id': 123,  # Drone identification
    'backlight': True  # Boolean value. True means that there is backlight, False - no backlight.
}
