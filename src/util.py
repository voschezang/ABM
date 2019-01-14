import numpy as np


def distance_in_seconds(distance_abs, vel):
    """
    distance_abs -- absolute distance
    vel - tuple of velocity in x,y direction in seconds
    """
    # TODO choose to vel in km/h or m/s?
    speed = np.linalg.norm(vel)
    return distance_abs / speed
