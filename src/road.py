import numpy as np
from mesa.space import ContinuousSpace

from enum import IntEnum


class Direction(IntEnum):
    L = -1
    R = +1


class Road(ContinuousSpace):
    def __init__(self, model, length, n_lanes, lane_width, torus):
        super().__init__(length, n_lanes * lane_width, torus)

        self.length = length
        self.model = model
        self.n_lanes = n_lanes
        self.lane_width = lane_width

    def lane(self, car):
        return int(car.pos[1] // self.lane_width)

    def lane_at(self, pos):
        return int(pos[1] // self.lane_width)

    def distance_from_center_of_lane(self, pos):
        return pos[1] % self.lane_width - self.lane_width / 2

    def is_left_of_center_of_lane(self, pos):
        return self.distance_from_center_of_lane(pos) < 0

    def cars_in_range(self, car, forward=True):
        """Return a list with tuples (first car, distance) in the left, current and right lane."""

        # get cars ahead within vision range of: max_speed + car_size + min_spacing
        vision = self.model.max_speed + self.model.car_length + self.model.min_spacing * self.model.max_speed

        # get the distances to each car
        dists = self._agent_points - car.pos if forward else car.pos - self._agent_points
        if self.torus:
            for dist in dists:
                if dist[0] < 0:
                    dist[0] += self.length

        # filter out cars outside of the vision range or more than one lane away
        idxs = np.where(
            np.all(
                [
                    dists[:, 0] > 0, dists[:, 0] < vision,
                    np.abs(dists[:, 1]) < self.lane_width * 1.5
                ],
                axis=0))[0]

        # list for storing tuples of (closest car, distance) in the left, current and right lane
        cars = [None, None, None]

        # store the closest car for each lane (left, current, right)
        lane = self.lane(car)
        for i in idxs:
            x = self._index_to_agent[i]
            for j in range(3):
                if x.lane == lane + j - 1:
                    if cars[j] == None or dists[i, 0] < cars[j][1]:
                        cars[j] = (x, dists[i, 0])

        return cars

    def change_lane(self, car, lane):
        car.pos[1] = (lane + 0.5) * self.lane_width

    def can_change_lane(self, car, target_lane, cars_front, cars_back):
        """Returns whether there is room fo a car to change lane in direction (L/R)"""

        # if target lane exists
        if target_lane < 0 or target_lane >= self.n_lanes:
            return False

        # index for getting cars in target_lane from cars_front/cars_back
        i = (target_lane - car.lane) + 1

        # check if there is space in front in the target lane
        if not cars_front[i] or car.vel[0] * self.model.time_step < cars_front[i][1] - self.model.car_length - self.model.min_spacing * car.vel[0]:  # TODO check rule for looking forward
            # check if there is space backwards in target lane
            # TODO: incorporate car.minimal_overtake_distance
            if not cars_back[i] or cars_back[i][0].vel[0] * self.model.time_step < cars_back[i][1] - self.model.car_length - self.model.min_spacing * cars_back[i][0].vel[0]:  # TODO change rule for checking backwards
                return True

        return False
