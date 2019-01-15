import numpy as np

from mesa import Model
from mesa.space import ContinuousSpace
from mesa.time import RandomActivation

import src.util as util
from .car import Car


class MyModel(Model):
    def __init__(self,
                 length=100,
                 n_lanes=1,
                 n_cars=10,
                 max_speed=10,
                 car_length=5,
                 min_spacing=2,
                 car_acc=1,
                 p_slowdown=0.2):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lanes -- number of lanes.
        n_cars -- number of cars.
        max_speed -- maximum speed cars can (and want to) travel at.
        car_length -- length of each car.
        min_spacing -- the minimum distance cars keep from each other (bumper to bumper) in seconds
        car_acc -- acceleration of the cars.
        p_slowdown -- probability of a car slowing down at random.
        """

        super().__init__()

        self.length = length
        self.n_lanes = n_lanes
        self.n_cars = n_cars
        self.max_speed = max_speed
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.car_acc = car_acc
        self.p_slowdown = p_slowdown

        self.space = ContinuousSpace(length, n_lanes + 1, torus=True)
        self.schedule = RandomActivation(self)

        self.make_agents()

    def step(self):
        self.schedule.step()

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        for i in range(self.n_cars):
            x = self.random.random() * self.space.x_max
            y = self.random.randint(1, self.n_lanes)
            pos = (x, y)
            vel = (self.max_speed, 0)
            # max_speed = util.km_to_ms(self.max_speed)  # TODO stochastic?
            max_speed = self.max_speed  # TODO stochastic?
            bias_right_lane = 1  # TODO stochastic?
            minimal_overtake_distance = 2  # TODO stochastic?
            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, pos)
            self.schedule.add(car)

    def lane(self, pos):
        return round(pos[1] / self.lane_width)

    def is_left_of_center_of_lane(self, pos):
        return pos[0] % self.lane_width < 0.5

    def get_neighbors_per_lane(self, car):
        current_lane = lane(car.pos)
        cars_all = self.get_neighbours(self)
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
