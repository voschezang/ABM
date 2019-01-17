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
        self.update_velocity()
        self.move()

        return
        ### 1. accelerate if not maximum speed
        if self.vel[0] < self.max_speed:
            self.vel[0] = min(self.vel[0] + self.model.car_acc * self.model.time_step, self.max_speed)


        ### 2. prevent collision with other cars
        cars_front = self.model.space.cars_in_range(self)
        cars_back = self.model.space.cars_in_range(self, forward=False)

        if self.unique_id == 1:
            print("Car {}: {} m/s".format(self.unique_id, self.vel[0]))
            print(cars_front)
            print(cars_back)

        # if car in front
        if cars_front[1] != None:
            car, d = cars_front[1]
            d -= self.model.car_length
            spacing = self.model.min_spacing * car.vel[0]

            # if needs to brake in order to keep minimum spacing
            if self.vel[0] * self.model.time_step > d - spacing:
                if self.model.space.can_change_lane(self, self.lane + Direction.L, cars_front, cars_back):
                    self.target_lane = self.lane + Direction.L
                    self.vel[1] = Direction.L * self.model.space.lane_width / self.model.lane_change_time
                else:
                    # brake
                    self.vel[0] = (d - spacing) / self.model.time_step

        # check if can move to right lane
        if self.target_lane == None and self.model.space.can_change_lane(self, self.lane + Direction.R, cars_front, cars_back) and self.random.random() < self.bias_right_lane:
            self.target_lane = self.lane + Direction.R
            self.vel[1] = Direction.R * self.model.space.lane_width / self.model.lane_change_time


        ### 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            self.vel[0] -= self.model.car_acc * self.model.time_step

        # clip negative velocities to zero
        self.vel[0] = max(self.vel[0], 0)


        ### 4. move car to new position
        pos = self.pos + self.vel * self.model.time_step
        # if changing lane check if lane change finished
        if self.target_lane != None:
            self.lane = self.model.space.lane_at(pos)
            #print(self.lane, self.model.space.distance_from_center_of_lane(pos), self.model.space.lane_width / self.model.lane_change_time * self.model.time_step)
            if self.lane == self.target_lane and np.abs(self.model.space.distance_from_center_of_lane(pos)) < self.model.space.lane_width / self.model.lane_change_time * self.model.time_step:
                self.target_lane = None
                self.vel[1] = 0
                pos[1] = (self.lane + 0.5) * self.model.space.lane_width
                #print("{} lane_change finish".format(self.unique_id))
        self.model.space.move_agent(self, pos)


    def update_velocity(self):
        vel_new = self.vel

        ### 1. accelerate if not maximum speed
        if vel_new[0] < self.max_speed:
            vel_new[0] = min(vel_new[0] + self.model.car_acc * self.model.time_step, self.max_speed)

        ### 2. prevent collision with other cars
        cars_front = self.model.space.cars_in_range(self)
        cars_back = self.model.space.cars_in_range(self, forward=False)

         # if car in front
        if cars_front[1] != None:
            car, d = cars_front[1]
            d -= self.model.car_length # absolute distance (bumper-to-bumper)

            # if needs to brake in order to keep minimum spacing
            if d < vel_new[0] * (self.model.time_step + self.model.min_spacing):
                vel_new[0] = d / (self.model.time_step + self.model.min_spacing)

        ### 3. randomly slow down
        if self.random.random() < self.model.p_slowdown:
            vel_new[0] -= self.model.car_acc * self.model.time_step

        # clip negative velocities to zero
        vel_new[0] = max(vel_new[0], 0)

        self.vel_new = vel_new

    def move(self):
        self.vel = self.vel_new
        self.pos += self.vel * self.model.time_step

        self.model.space.move_agent(self, self.pos)

        







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


 

