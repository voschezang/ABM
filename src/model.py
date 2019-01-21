import numpy as np

import mesa
from mesa.time import StagedActivation, RandomActivation
from mesa.datacollection import DataCollector

import src.util as util
from .car import Car
from .road import Road

### datacollection functions


def density(model):
    """Density: number of cars per unit length of road."""
    return len(model.schedule.agents) / model.space.length


def flow(model):
    """Flow: number of cars passing a reference point per unit of time."""
    reference_pont = model.space.length / 2

    for car in model.schedule.agents:
        if car.pos[0] > reference_pont:
            model.flow_cars.add(car.unique_id)

    if model.schedule.time * model.time_step % 10 < model.time_step:
        model.flow = len(model.flow_cars.difference(model.flow_cars_previous))
        model.flow_cars_previous = model.flow_cars
        model.flow_cars = set()

    return model.flow


###


class Model(mesa.Model):
    max_lanes = 10

    def __init__(self, length, lane_width, n_lanes, n_cars, max_speed,
                 car_length, min_spacing, car_acc, car_dec, p_slowdown,
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
        car_acc -- acceleration of the cars (in m\s2).
        car_dec -- deceleration of the cars (in m\s2).
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
        self.car_acc = car_acc
        self.car_dec = car_dec
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.lane_change_time = 2  # TODO use rotation matrix
        self.bias_right_lane_seconds = 60

        self.space = Road(self, length, n_lanes, lane_width, torus=False)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, ["update_vel_next", "move"],
            shuffle=False,
            shuffle_between_stages=False)

        #self.make_agents()

        # create data collectors
        self.flow = 0
        self.flow_cars = set()
        self.flow_cars_previous = set()
        self.data_collector = DataCollector(model_reporters={
            "Density": density,
            "Flow": flow
        })

    def step(self):
        self.generate_cars()
        self.schedule.step()
        self.data_collector.collect(self)

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        for i in range(self.n_cars):
            x = self.random.random() * self.space.length
            y = self.space.center_of_lane(
                self.random.randint(0, self.space.n_lanes - 1))
            pos = (x, y)
            vel = (self.max_speed, 0)
            max_speed = np.random.normal(self.max_speed
                                         if i == 0 else self.max_speed / 3, 5)
            bias_right_lane = 1.0  # TODO stochastic?
            minimal_overtake_distance = 2.0  # TODO stochastic?

            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)

    def generate_cars(self):
        if self.random.random() < 0.1:
            x = 0  # self.random.random() * self.space.length
            y = self.space.center_of_lane(
                self.random.randint(0, self.space.n_lanes - 1))
            pos = (x, y)
            max_speed_sigma = 5
            max_speed = self.stochastic_params(
                self.max_speed, max_speed_sigma, seconds=None)
            vel = np.array([self.max_speed, 0])
            # bias to go to the right lane (probability based, per minute)
            bias_right_lane = self.stochastic_params(
                0.5, sigma=3, seconds=self.bias_right_lane_seconds)
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
