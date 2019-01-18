import numpy as np
from mesa.space import ContinuousSpace

from enum import IntEnum
from collections import namedtuple


class Direction(IntEnum):
    L = -1
    C = 0
    R = +1


# tuple for storing neighbours.
# f -- front
# b -- back
# l -- left
# r -- right
# d -- distance
Neighbours = namedtuple("Neighbours", [
    "f_l", "b_l", "f_l_d", "b_l_d", "f", "b", "f_d", "b_d", "f_r", "b_r",
    "f_r_d", "b_r_d"
])


class Road(ContinuousSpace):
    def __init__(self, model, length, n_lanes, lane_width, torus):
        super().__init__(length, n_lanes * lane_width, torus)

        self.length = length
        self.model = model
        self.n_lanes = n_lanes
        self.lane_width = lane_width

    # override
    def place_agent(self, agent, pos):
        super().place_agent(agent, pos)
        agent.lane = self.lane_at(pos)

    # override
    def move_agent(self, agent, pos):
        super().move_agent(agent, pos)
        agent.lane = self.lane_at(pos)

    def lane_at(self, pos):
        """Returns the lane number of a position"""
        return int(pos[1] // self.lane_width)

    def lane_exists(self, lane):
        return lane >= 0 and lane < self.n_lanes

    def distance_from_center_of_lane(self, pos):
        return pos[1] % self.lane_width - self.lane_width / 2

    def is_right_of_center_of_lane(self, pos):
        return self.distance_from_center_of_lane(pos) > 0

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

    def distance(self, a, b, forward=True):
        """Returns forward/backward distance from a to b

        Note
        ----
        returned distance does not include car size, so distance is center-to-center
        """
        d = b.pos[0] - a.pos[0]
        if not forward:
            d *= -1

        if self.torus:
            if d < 0:
                d += self.length
        return d

    def neighbours(self, car, lane=None):
        """Get the first car in a lane in front and back, and the distance to them (if they exist).

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
        for x in self.cars_in_lane(
                lane if lane != None else car.lane, exclude=car):
            for i, forward in enumerate([True, False]):
                d = self.distance(car, x, forward)
                if cars[i] == None or (d > 0 and d < distances[i]):
                    cars[i] = x
                    distances[i] = d

        return cars, distances

    def all_neighbours(self, car):
        """Return all neighbours for a car"""
        lanes = car.lane + np.array([-1, 0, 1])

        (cars_l, dists_l) = self.neighbours(car, car.lane - 1)
        (cars, dists) = self.neighbours(car, car.lane)
        (cars_r, dists_r) = self.neighbours(car, car.lane + 1)

        return Neighbours(*cars_l, *dists_l, *cars, *dists, *cars_r, *dists_r)

    def steer_to_lane(self, car, vel, neighbours, directions=[Direction.R]):
        # TODO
        # compute the velocity required to steer a car to another lane
        # returns a tuple (success: bool, vel: (int,int) )

        for direction in directions:
            if not self.lane_exists(car.lane + direction):
                continue

            if self.can_go_to_lane(vel, neighbours, direction):
                vel = self.steer(vel, direction)
                return (True, vel)
        return (False, vel)

    def can_go_to_lane(self, vel, neighbours, direction):
        """Returns whether there is room fo a car (at pos, with vel) to change lane in direction (L/R)"""
        f = neighbours.f_l if direction == Direction.L else neighbours.f_r
        b = neighbours.b_l if direction == Direction.L else neighbours.b_r
        f_d = (neighbours.f_l_d if direction == Direction.L else
               neighbours.f_r_d) - self.model.car_length
        b_d = (neighbours.b_l_d if direction == Direction.L else
               neighbours.b_r_d) - self.model.car_length

        if not f or f_d > vel[0] * (
                self.model.time_step + self.model.min_spacing):
            if not b or b_d > b.vel[0] * (
                    self.model.time_step + self.model.min_spacing):
                return True
        return False

    def steer(self, vel, direction):
        vel[1] = direction * self.lane_width / self.model.lane_change_time
        return vel

    def center_on_current_lane(self, pos, vel):
        d = self.distance_from_center_of_lane(pos)
        direction = Direction.L if self.is_right_of_center_of_lane(
            pos) else Direction.R
        vel = self.steer(vel, direction)

        if direction * (vel[1] * self.model.time_step + d) > 0:
            vel[1] = vel[1] * self.model.time_step + d
        return vel

    def relative_distance_from_to(self, a, b, dimension=0):
        # relative distance from Agent a to Agent b (in 1 dimension)
        distance_abs = a.pos[dimension] - b.pos[dimension]
        return util.distance_in_seconds(distance_abs, a.vel[dimension],
                                        b.vel[dimension])
