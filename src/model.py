import numpy as np

from mesa import Model
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
                 car_length, min_spacing, car_acc_pos, car_acc_neg, p_slowdown,
                 time_step):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lane_width -- width of a lane.
        n_lanes -- number of lanes.
        n_cars -- number of cars.
        max_speed -- maximum speed cars can (and want to) travel at in km/h (will be converted to m/s).
        car_length -- length of each car.
        min_spacing -- the minimum distance cars keep from each other (bumper to bumper) in seconds.
        car_acc_pos -- positive acceleration of the cars.
        car_acc_neg -- negative acceleration of the cars.
        p_slowdown -- probability of a car slowing down per hour.
        time_step -- in seconds.
        """

        super().__init__()
        np.random.seed()

        self.time_step = time_step
        self.n_cars = n_cars
        self.max_speed = max_speed / 3.6
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.car_acc_pos = car_acc_pos
        self.car_acc_neg = car_acc_neg
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.lane_change_time = 2  # TODO use rotation matrix

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
            max_speed_sigma = 5
            max_speed = self.stochastic_params(
                self.max_speed, max_speed_sigma, seconds=None)
            vel = np.array([self.max_speed, 0])
            # bias to go to the right lane (probability based, per minute)
            bias_right_lane = self.stochastic_params(0.5, sigma=3, seconds=60)
            minimal_overtake_distance = 2.0  # TODO stochastic?

            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)

    def delay_time_to_probability(self, T=0):
        """Return the probability required to simulate a delay in communication
        used to simulate reaction time
        T: period (interval time)
        f = 1/T
        """
        T_in_time_steps = T / self.time_step
        frequency = 1 / T_in_time_steps
        probability = frequency
        return probability

    def stochastic_params(self, mean, sigma=1, pos=True, seconds=1):
        # returns a stochastic parameter
        p = np.random.normal(mean, sigma)
        if pos:
            p = np.clip(p, 0, None)
        if seconds:
            p = self.probability_per(p, seconds)
        return p

    def probability_per(self, p, seconds=60):
        # note that this limits the amount of timesteps
        assert (seconds > self.time_step)
        return p * self.time_step / seconds
