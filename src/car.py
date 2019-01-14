import numpy as np
from enum import Enum

from mesa import Agent

import src.util as util

overtake_direction = Enum('overtake_direction', 'R L')


class Car(Agent):
    def __init__(self, unique_id, model, pos, vel):
        """Create a `Car` agent

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- the model the agent is part of.
        pos -- initial position of the car.
        vel -- initial velocity of the car.
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)

    def current_speed(self):
        return np.linalg.norm(self.vel)

    def forward_distance_to_car(self, car):
        """Return forward distance (in x-direction) to a car."""

        distance = car.pos[0] - self.pos[0]
        if distance < 0:
            distance = self.model.length + distance
        return distance - self.model.car_length

    def cars_in_front(self):
        """Return a list of tuples of the first car in front and the distance to that car."""

        # get cars ahead within vision range of: max_speed + car_size + min_spacing
        vision = self.model.max_speed + self.model.car_length + self.model.min_spacing
        cars = self.model.space.get_neighbors(self.pos + np.array(
            (vision / 2, 0)), vision / 2)
        # create list of tuple (car, distance) for cars on the same lane
        cars = [(x, self.forward_distance_to_car(x)) for x in cars
                if x != self and x.pos[1] == self.pos[1]]
        # sort on (forward) distance
        return sorted(cars, key=lambda x: x[1])

    def slow_down(self, distance):
        if self.vel[0] > distance - self.model.min_spacing:
            self.vel[0] = distance - self.model.min_spacing

    def desire_to_overtake(self, distance_to_next_car):
        if distance_to_next_car > self.overtake_threshold():
            return False

        radius = 1  # TODO limit search to the left/right lane (instead of radius)
        neighbors = self.model.space.get_neighbors(
            self.pos, radius, include_center=False)
        if neighbors:
            return False

        return True

    def overtake(self, direction):
        d_vel = np.zeros(2)
        angle = 0.1 * self.current_speed()
        if direction == overtake_direction.R:
            self.vel[1] = angle
        else:
            self.vel[1] = -angle

        self.vel /= np.linalg.norm(self.vel)

    def overtake_threshold(self, a=3):
        """ Return the relative distance
        """
        # TODO find appropriate value for param a
        return util.distance_in_seconds(self.model.car_length * a, self.vel)

    def step(self):
        """Apply Nagel-Schreckenberg rules
        """

        # 1. accelerate if not maximum speed
        if self.vel[0] < self.model.max_speed:
            self.vel[0] = min(self.vel[0] + self.model.car_acc,
                              self.model.max_speed)

        # 2. slow down if car in front
        # possibly take over other cars
        cars = self.cars_in_front()
        if cars:
            car, distance = cars[0]
            if self.desire_to_overtake(distance):
                self.overtake(overtake_direction.L)
                # TODO make sure the agent does not linger between lanes
            else:
                self.slow_down(distance)

        # 3. random slow down
        if self.random.random() < self.model.p_slowdown:
            self.vel[0] -= self.model.car_acc

        # clip negative velocities to zero
        self.vel[0] = max(self.vel[0], 0)

        # 4. move car to new position
        pos = self.pos + self.vel
        self.model.space.move_agent(self, pos)
