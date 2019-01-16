import numpy as np
import collections

from mesa import Agent

import src.util as util

from .road import Direction


class Car(Agent):
    def __init__(self, unique_id, model, pos, vel, max_speed, bias_right_lane, minimal_overtake_distance):
        """Create an agent that represents a car

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- reference to the model the agent is part of.
        pos -- tuple of current position of the car.
        vel -- tuple of current x,y velocities of the car.
        max_speed -- .
        bias_right_lane -- bias to move to the right-hand lane in range [0,1].
        minimal_overtake_distance -- in time units.
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
            self.vel[0] = min(self.vel[0] + self.model.car_acc * self.model.time_step, self.max_speed)

        #print("Car {}: {} m/s".format(self.unique_id, self.vel[0]))

        # 2. prevent collision with other cars
        lane = self.model.space.lane(self)
        cars = self.model.space.cars_in_range(self)
        cars_back = self.model.space.cars_in_range(self, forward=False)

        # if car in front
        if cars[1] != None:
            car, d = cars[1]
            d -= self.model.car_length
            spacing = self.model.min_spacing * self.vel[0]

            # if needs to brake in order to keep minimum spacing
            if self.vel[0] * self.model.time_step > d - spacing:
                if self.model.space.can_change_lane(self, Direction.L, cars, cars_back):
                    self.model.space.change_lane(self, lane + Direction.L)
                else:
                    # brake
                    self.vel[0] = (d - spacing) / self.model.time_step

        # check if can move to right lane
        if self.model.space.can_change_lane(self, Direction.R, cars, cars_back) and self.random.random() < self.bias_right_lane:
            self.model.space.change_lane(self, lane + Direction.R)


        # 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            self.vel[0] -= self.model.car_acc * self.model.time_step

        # clip negative velocities to zero
        self.vel[0] = max(self.vel[0], 0)

        # 4. move car to new position
        pos = self.pos + self.vel * self.model.time_step
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


 

