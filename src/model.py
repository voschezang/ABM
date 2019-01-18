import numpy as np

from mesa import Model
from mesa.space import ContinuousSpace
from mesa.time import StagedActivation, RandomActivation
from mesa.datacollection import DataCollector

import src.util as util
from .car import Car
from .road import Road

### datacollection functions


def vel0(model):
    """Velocity of car 0"""
    return model.schedule.agents[0].vel[0]


###


class MyModel(Model):
    max_lanes = 10

    def __init__(self, length, lane_width, n_lanes, n_cars, max_speed,
                 car_length, min_spacing, car_acc, p_slowdown, time_step):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lane_width -- width of a lane.
        n_lanes -- number of lanes.
        n_cars -- number of cars.
        max_speed -- maximum speed cars can (and want to) travel at in km/h (is converted to m/s).
        car_length -- length of each car.
        min_spacing -- the minimum distance cars keep from each other (bumper to bumper) in seconds.
        car_acc -- acceleration of the cars.
        p_slowdown -- probability of a car slowing down at random.
        time_step -- in seconds.
        """

        super().__init__()
        np.random.seed()

        self.n_cars = n_cars
        self.max_speed = max_speed / 3.6
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.car_acc = car_acc
        self.p_slowdown = p_slowdown
        self.time_step = time_step
        self.lane_change_time = 2

        self.space = Road(self, length, n_lanes, lane_width, torus=True)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, ["update_vel_next", "move"],
            shuffle=False,
            shuffle_between_stages=False)

        self.make_agents()

        # create data collectors
        self.data_collector = DataCollector(model_reporters={"Velocity": vel0})

    def step(self):
        self.data_collector.collect(self)
        self.schedule.step()

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        for i in range(self.n_cars):
            x = self.random.random() * self.space.length
            y = (self.random.randint(0, self.space.n_lanes - 1) + 0.5
                 ) * self.space.lane_width
            pos = (x, y)
            vel = (self.max_speed if i == 0 else self.max_speed / 2, 0)
            max_speed = np.random.normal(self.max_speed, 0)
            bias_right_lane = 1.0  # TODO stochastic?
            minimal_overtake_distance = 2.0  # TODO stochastic?

            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)
