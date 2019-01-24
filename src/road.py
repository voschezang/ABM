import numpy as np
from mesa.space import ContinuousSpace

from enum import IntEnum
from collections import namedtuple


class Direction(IntEnum):
    L = -1
    C = 0
    R = +1


class Neighbours:
    def __init__(self, left, current, right):
        """Store the neighbours (and distances) per lane (Left, Current, Right)"""
        self._cars = [left[0], current[0], right[0]]
        self._distances = [left[1], current[1], right[1]]

    def in_direction(self, direction):
        return self._cars[direction + 1]

    def distances(self, direction):
        return self._distances[direction + 1]

    @property
    def f(self):
        """Neighbour in front."""
        return self._cars[1][0]

    @property
    def f_d(self):
        """Distance to the neighbour in front."""
        return self._distances[1][0]


def distance_in_seconds(distance_abs, vel_self, vel_other=None):
    """ Returns the distance to another object in seconds, assuming the object is moving with a constant velocity
    Velocity in the y-dimension is ignored

    distance_abs -- absolute distance
    vel_self, vel_other - tuple of velocity in x,y direction in seconds
    """
    vel = vel_self[0]
    if vel_other is not None:
        vel -= vel_other[0]
    if vel == 0:
        return 1e15  # arbitrarily large
    return distance_abs / vel


class Road(ContinuousSpace):
    LANE_WIDTH = 3.5  # in meters.

    def __init__(self, model, length, n_lanes, torus):
        super().__init__(length, n_lanes * self.lane_width, torus)

        self.length = length
        self.model = model
        self.n_lanes = n_lanes

    # override
    def place_agent(self, agent, pos):
        super().place_agent(agent, pos)
        agent.lane = self.lane_at(pos)

    # override
    def move_agent(self, agent, pos):
        super().move_agent(agent, pos)
        agent.lane = self.lane_at(pos)

    @property
    def lane_width(self):
        return self.LANE_WIDTH

    def lane_at(self, pos):
        """Returns the lane number of a position"""
        return int(pos[1] // self.lane_width)

    def center_of_lane(self, lane):
        """Return the position of the center of a lane."""
        return (lane + 0.5) * self.lane_width

    def lane_exists(self, lane):
        return lane >= 0 and lane < self.n_lanes

    def distance_from_center_of_lane(self, pos):
        return pos[1] % self.lane_width - self.lane_width / 2

    def is_right_of_center_of_lane(self, pos):
        return self.distance_from_center_of_lane(pos) > 0

    def distance_toroidal(self, a, b, forward=True):
        """Returns forward/backward distance (on a toroidal road) in meter from a to b"""
        d = self.distance_between_coordinates(a.pos, b.pos)
        if not forward:
            d *= -1
        if d < 0:
            d += self.length
        return max(0, d - a.length)  # TODO not b.length?

    def distance_between_coordinates(self, a, b):
        return b[0] - a[0]

    def distance_between_objects(self, a, b):
        assert (not self.torus)  # not implemented
        d = self.distance_between_coordinates(a.pos, b.pos)
        # assume pos indicates the front of the object
        # (this does not matter for the visualization, since all cars have the same size)
        if d >= 0:
            return max(0, d - b.length)
        else:
            return min(0, d + a.length)

    def cars_in_lane(self, lane, exclude=[]):
        """Returns all cars in a certain lane (optionally excluding some cars)"""
        if not self.lane_exists(lane):
            return []
        if not isinstance(exclude, (list, tuple)):
            exclude = [exclude]
        return [
            car for car in self._index_to_agent.values()
            if car.lane == lane and car not in exclude
        ]

    def first_car_in_lane(self, lane):
        cars = self.cars_in_lane(lane)
        if not cars:
            return None
        return min(cars, key=lambda car: car.pos[0])

    def neighbours(self, car, lane=None):
        """Get the first car in a lane in front and back, and the absolute distances (in m) to them if they exist (-1 as default).
        Note that the car_length is subtracted for cars in front of the agent

        Parameters
        ----------
        car -- to get the neighbours of.
        lane -- in which to look for neighbours (if `None` uses the current lane of `car`).
        Returns
        -------
        tuple ([car_front, car_back], [distance_front, distance_back]).
        """
        cars = [None, None]
        distances = [-1, -1]
        for other_car in self.cars_in_lane(
                lane if not lane is None else car.lane, exclude=car):
            for i, forward in enumerate([True, False]):
                #d = self.distance_between_objects(car, other_car)
                d = self.distance_toroidal(car, other_car, forward=forward)
                # # only include cars in front/back when looking forward/backward
                # if (forward and d < 0) or (not forward and d > 0):
                #     continue
                # # get the absolute distance when looking backward
                # if not forward:
                #     d *= -1
                if d >= 0 and (cars[i] is None or d < distances[i]):
                    cars[i] = other_car
                    distances[i] = d

        return cars, distances

    def all_neighbours(self, car):
        """Return all neighbours for a car"""
        return Neighbours(*[
            self.neighbours(car, car.lane + i)
            for i in [Direction.L, Direction.C, Direction.R]
        ])
