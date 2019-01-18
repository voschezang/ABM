import numpy as np
import collections

from mesa import Agent

import src.util as util

from .road import Direction


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
        max_speed -- in m/s.
        bias_right_lane -- bias to move to the right-hand lane in range [0,1].
        minimal_overtake_distance -- in time units.
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.max_speed = max_speed
        self.bias_right_lane = bias_right_lane
        self.minimal_overtake_distance = minimal_overtake_distance
        self.target_lane = None

    def step(self):
        self.update_vel_next()
        self.move()

    def move(self):
        self.vel = self.vel_next
        self.pos += self.vel * self.model.time_step

        self.model.space.move_agent(self, self.pos)

    def update_vel_next(self):
        """update the property `vel_next`: the intended velocity of agent based of the current state."""

        ### 1. accelerate if not maximum speed
        vel_next = self.accelerate_vel(
            self.vel.copy())

        ### 2. prevent collision with other cars
        # TODO combine searching (search forward for cars in current lane and backward for cars in adjacent lanes)
        # (optional) stop searching adjacent lanes when a blocking car is found
        #   i.e. a reason that makes overtaking impossible
        # We may have to simplify the interface, i.e. store cars per lane and simply iterate all cars in that lane, instead of searching in a radius
        neighbours = self.model.space.all_neighbours(self)

        # if car in front
        if neighbours.f and self.needs_to_brake(vel_next, neighbours.f_d - self.model.car_length):
            if not self.model.space.is_right_of_center_of_lane(self.pos):  # i.e. in the middle or left
                if self.unique_id == 1:
                    print(self.unique_id, "try left")
                success, vel_next = self.model.space.steer_to_lane(
                    self, vel_next, neighbours, [Direction.L, Direction.R])
            else:
                if self.unique_id == 1:
                    print(self.unique_id, "try right")
                success, vel_next = self.model.space.steer_to_lane(
                    self, vel_next, neighbours, [Direction.R, Direction.L])

            if not success:
                if self.unique_id == 1:
                    print(self.unique_id, "brake")
                vel_next = self.model.space.center_on_current_lane(self.pos, vel_next)
                vel_next = self.brake(vel_next, neighbours.f_d - self.model.car_length)

        # no car in front
        else:
            if self.random.random() < self.bias_right_lane:
                if self.unique_id == 1:
                    print(self.unique_id, "try right bias")
                succes, vel_next = self.model.space.steer_to_lane(
                    self, vel_next, neighbours, [Direction.R])
                if not succes:
                    vel_next = self.model.space.center_on_current_lane(self.pos, vel_next)
                    if self.unique_id == 1:
                        print(self.unique_id, "center 1")
            else:
                if self.unique_id == 1:
                    print(self.unique_id, "center 2")
                vel_next = self.model.space.center_on_current_lane(self.pos, vel_next)

        ### 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            vel_next[0] -= self.model.car_acc * self.model.time_step

        # clip negative velocities to zero
        vel_next[0] = max(vel_next[0], 0)

        self.vel_next = vel_next
        

    def accelerate_vel(self, vel):
        # returns accelerated vel, upper limited by the maximum speed
        vel[0] = min(
            vel[0] + self.model.car_acc * self.model.time_step,
            self.max_speed)
        return vel

    def needs_to_brake(self, vel, distance):
        """Returns if needs to brake in order to keep minimum spacing"""
        return distance < vel[0] * (self.model.time_step + self.model.min_spacing)

    def brake(self, vel, distance):
        if self.needs_to_brake(vel, distance):
            vel[0] = distance / (
                self.model.time_step + self.model.min_spacing)
        return vel

    

    # predicates

    def is_left_of_center(self):
        return self.pos[0] % self.model.lane_width > 0.5

    def car_in_front(self, cars):
        chosen_car = None
        distance = self.model.min_spacing
        for car in cars:
            car_distance = self.relative_distance_to(car)
            if car_distance < self.model.min_spacing:
                if car_distance < distance:
                    chosen_car = car
        return (chosen_car, distance)
