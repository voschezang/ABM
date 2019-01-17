import numpy as np


def kmh_to_ms(speed):
    return speed / 3.6


def ms_to_kmh(speed):
    return speed * 3.6


def distance_in_seconds(distance_abs, vel_self, vel_other):
    """
    distance_abs -- absolute distance
    vel - tuple of velocity in x,y direction in seconds
    """
    if np.any(vel_self == 0):
        return 1e15
    # TODO choose to vel in km/h or m/s?
    speed = np.linalg.norm(vel)
    return distance_abs / (vel_self - vel_other)
