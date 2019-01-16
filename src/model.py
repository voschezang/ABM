import numpy as np

from mesa import Model
from mesa.space import ContinuousSpace
from mesa.time import RandomActivation

import src.util as util
from .car import Car
from .road import Road


class MyModel(Model):
    def __init__(self,
                 length,
                 lane_width,
                 n_lanes,
                 n_cars,
                 max_speed,
                 car_length,
                 min_spacing,
                 car_acc,
                 p_slowdown,
                 time_step):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lane_width -- width of a lane.
        n_lanes -- number of lanes.
        n_cars -- number of cars.
        max_speed -- maximum speed cars can (and want to) travel at.
        car_length -- length of each car.
        min_spacing -- the minimum distance cars keep from each other (bumper to bumper) in seconds.
        car_acc -- acceleration of the cars.
        p_slowdown -- probability of a car slowing down at random.
        time_step -- in seconds.
        """

        super().__init__()
        np.random.seed()

        self.n_cars = n_cars
        self.max_speed = max_speed
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.car_acc = car_acc
        self.p_slowdown = p_slowdown
        self.time_step = time_step

        self.space = Road(self, length, n_lanes, lane_width, torus=True)
        self.schedule = RandomActivation(self)

        self.make_agents()

    def step(self):
        self.schedule.step()

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        for i in range(self.n_cars):
            x = self.random.random() * self.space.length
            y = (self.random.randint(0, self.space.n_lanes - 1) + 0.5) * self.space.lane_width
            pos = (x, y)
            vel = (self.max_speed, 0)
            max_speed = np.random.normal(self.max_speed, 0)
            bias_right_lane = 1  # TODO stochastic?
            minimal_overtake_distance = 2  # TODO stochastic?

            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)





    

    def get_neighbors_per_lane(self, car):
        current_lane = self.lane(car.pos)
        cars_all = self.get_neighbours(car)
        cars_right = []
        cars_current = []
        cars_left = []
        for other_car in cars_all:
            other_lane = lane(other_car.pos)
            if other_lane == current_lane - 1:
                cars_right.append(other_car)
            elif other_lane == current_lane:
                cars_current.append(other_car)
            elif other_lane == current_lane + 1:
                cars_left.append(other_car)
            else:  # a non-neighboring lane
                pass
        return (cars_right, cars_current, cars_left)

    def get_neighbors(self, car):
        # max distance worst case = a static (non-moving) object (with 0 velocity)
        max_forward_distance_seconds = 2
        speed = util.kmh_to_ms(car.vel[0])  # neglect y axis
        max_forward_distance_meters = max_radius_seconds / car.vel[0]
        max_radius_meters = np.linalg.norm((max_forward_distance_meters,
                                            1.5 * self.lane_width))
        return self.space.get_neighbors(
            car.pos, max_radius_meters, include_center=False)

    