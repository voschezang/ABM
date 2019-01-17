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
        self.lane = self.model.space.lane(self)
        self.target_lane = None

    def step(self):
        self.update_vel_next()
        self.move()

    def move(self):
        self.vel = self.vel_new
        self.pos += self.vel * self.model.time_step

        self.model.space.move_agent(self, self.pos)

    def update_vel_next(self):
        # update the property `vel_next`: the intended velocity of agent based of the current state
        # TODO force np array type
        ### 1. accelerate if not maximum speed
        self.vel_next = accelerate_vel(
            self.vel.copy())  # accelerate if not at max speed

        ### 2. prevent collision with other cars
        # TODO combine searching (search forward for cars in current lane and backward for cars in adjacent lanes)
        # (optional) stop searching adjacent lanes when a blocking car is found
        #   i.e. a reason that makes overtaking impossible
        # We may have to simplify the interface, i.e. store cars per lane and simply iterate all cars in that lane, instead of searching in a radius
        cars_front = self.model.space.cars_in_range(self)

        # if car in front
        if self.model.space.car_in_front(cars_front):
            car, d_head_to_head = cars_front[1]  # index 1?
            d = d_head_to_head - self.model.car_length

            if not self.right_of_center_of_lane():  # i.e. in the middle or left
                success, self.vel_next = self.model.space.steer_to_lane(
                    self, [Direction.L, Direction.R])
            else:
                success, self.vel_next = self.model.space.steer_to_lane(
                    self, [Direction.R, Direction.L])

            if not success:
                # TODO (optional) mv functions outside of class to avoid confusion of behaviour
                # e.g. next_vel = brake(self, next_vel)
                self.brake(distance)

        else:  # no car in front
            if self.bias_right_lane > self.random.random():
                _, vel_next = self.model.space.can_steer_to_lane(
                    self, [Direction.R])
            else:
                vel_next = center_on_current_lane(vel_next)

        ### 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            vel_new[0] -= self.model.car_acc * self.model.time_step

        # clip negative velocities to zero
        vel_new[0] = max(vel_new[0], 0)

        # self.vel_next = vel_next
        return vel_next

    def accelerate_vel(self, vel):
        # returns accelerated vel, upper limited by the maximum speed
        if vel[0] < self.max_speed:
            vel[0] = min(
                vel_new[0] + self.model.car_acc * self.model.time_step,
                self.max_speed)
        return vel

    def brake(self, distance):
        # if needs to brake in order to keep minimum spacing
        if distance < self.vel_next[0] * (
                self.model.time_step + self.model.min_spacing):
            self.vel_next[0] = distance / (
                self.model.time_step + self.model.min_spacing)

    #
    #
    #
    #
    #
    #
    #
    #

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

    def relative_distance_to(self, car, dimension=0):
        # in seconds
        distance_abs = self.pos[0] - car.pos[0]
        return util.distance_in_seconds(distance_abs, self.vel[0], car.vel[0])

    def distance_in_seconds(self, car):
        distance_abs = self.pos[0] - car.pos[0]
        return util.distance_in_seconds(distance_abs, self.vel[0], car.vel[0])
