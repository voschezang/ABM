import numpy as np
import collections
from enum import Enum

from mesa import Agent

import src.util as util

direction = Enum('direction', 'R L')


class Car(Agent):
    def __init__(self, unique_id, model, pos, vel, max_speed, bias_right_lane,
                 minimal_overtake_distance):
        """Create an agent that represents a car

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- reference to the model the agent is part of.
        pos -- tuple of current position of the car.
        vel -- tuple of current x,y velocities of the car.
        max_speed -- in km/h
        bias_right_lane -- bias to move to the right-hand lane in range [0,1]
        minimal_overtake_distance -- in seconds
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.max_speed = max_speed
        self.bias_right_lane = bias_right_lane
        self.minimal_overtake_distance = minimal_overtake_distance

    def step(self):
        # 1. accelerate if not maximum speed
        if self.vel[0] < self.max_speed:
            self.vel[0] = min(self.vel[0] + self.model.car_acc, self.max_speed)

        # 2. prevent collision with other cars
        #    possibly move to another lane
        (cars_right, cars_current,
         cars_left) = self.model.get_neighbors_per_lane(self)
        car, distance = self.car_in_front(cars_current)
        if car:
            # bias to move to the closest lane
            cars_dict = collections.ordereddict()
            if self.is_left_of_center():
                cars_dict[direction.L] = cars_left
                cars_dict[direction.R] = cars_right
                self.overtake(cars_dict)
            else:
                cars_dict[direction.R] = cars_right
                cars_dict[direction.L] = cars_left

            self.overtake(cars_dict)

        else:
            # bias to move to the right
            if self.lane_is_free(cars_right):
                if self.random.random() < self.bias_right_lane:
                    self.move_to_right_lane()
            else:
                self.center_on_current_lane()

        # 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            self.vel[0] -= self.model.car_acc

            # clip negative velocities to zero
            self.vel[0] = max(self.vel[0], 0)

        # 4. move car to new position
        pos = self.pos + self.vel
        self.model.space.move_agent(self, pos)

    def overtake(self, cars_dict):
        """
        cars_dict -- (ordered)dict of type {direction: [Car]}
        """
        for lane, cars in cars_dict:
            if self.lane_is_free(cars):
                self.move_to_lane(direction)
                return

        self.slow_down()
        self.center_on_current_lane()

    def slow_down(self, distance):
        if self.vel[0] > distance - self.model.min_spacing:
            self.vel[0] = distance - self.model.min_spacing

    def move_to_right_lane(self):
        pass

    def center_on_current_lane(self):
        pass

    # predicates

    def car_in_front(self, cars):
        chosen_car = None
        distance = self.model.min_spacing
        for car in cars:
            car_distance = self.relative_distance_to(car)
            if car_distance < self.model.min_spacing:
                if car_distance < distance:
                    chosen_car = car
        return (chosen_car, distance)

    def is_left_of_center(self):
        return self.pos[0] % self.model.lane_width > 0.5

    def lane_is_free(self, direction, cars):
        if direction == direction.L:
            target_lane = self.model.lane(self.pos) + 1
        else:
            target_lane = self.model.lane(self.pos) - 1

        for car in cars:
            if self.model.lane(car.pos) == target_lane:
                distance = self.relative_distance_to(car, dimension=0)
                if distance < self.model.min_spacing:
                    if distance < self.minimal_overtake_distance:
                        return False
        return True

    def relative_distance_to(self, car, dimension=0):
        # in seconds
        distance_abs = self.pos[0] - car.pos[0]
        return util.distance_in_seconds(distance_abs, self.vel[0], car.vel[0])

    def distance_in_seconds(self, car):
        distance_abs = self.pos[0] - car.pos[0]
        return util.distance_in_seconds(distance_abs, self.vel[0], car.vel[0])

    #
    #
    ### unused functions
    #
    #

    def current_speed(self):
        return np.linalg.norm(self.vel)

    def forward_distance_to_car(self, car):
        """Return forward distance (in x-direction) to a car."""

        distance = car.pos[0] - self.pos[0]
        if distance < 0:
            distance = self.model.length + distance
            if self.vel[0] > distance - self.model.min_spacing:
                self.vel[0] = distance - self.model.min_spacing

        return distance - self.model.car_length

    def cars_in_front(self):
        """Return a list of tuples of the first car in front and the distance to that car."""

        # get cars ahead within vision range of: max_speed + car_size + min_spacing
        vision = self.max_speed + self.model.car_length + self.model.min_spacing
        cars = self.model.space.get_neighbors(self.pos + np.array(
            (vision / 2, 0)), vision / 2)
        # create list of tuple (car, distance) for cars on the same lane
        cars = [(x, self.forward_distance_to_car(x)) for x in cars
                if x != self and x.pos[1] == self.pos[1]]
        # sort on (forward) distance
        return sorted(cars, key=lambda x: x[1])
