import numpy as np

import mesa
from mesa.time import StagedActivation, RandomActivation
from mesa.datacollection import DataCollector

import src.util as util
import src.road as road
from .car import Car
import src.data as data


class Model(mesa.Model):
    MAX_LANES = 10
    BIAS_RIGHT_LANE_SECONDS = 60

    def __init__(self,
                 length=1000,
                 lane_width=1,
                 n_lanes=1,
                 flow=1,
                 max_speed=10,
                 car_length=2,
                 min_spacing=1,
                 min_distance_mu=1,
                 min_distance_sigma=0,
                 car_acc=3,
                 car_dec=6,
                 p_slowdown=0.1,
                 time_step=0.1,
                 seed=None,
                 verbose=3):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road in meters.
        lane_width -- width of a lane in meters.
        n_lanes -- number of lanes.
        flow -- number of cars generated per lane per second (stochastic)
        max_speed -- maximum speed cars will try to travel at in km/h (will be converted to m/s).
        car_length -- length of each car.
        min_spacing -- the minimum distance in seconds a car keeps from other cars (front to back). Incorporating the cars' velocity but ignoring the other cars' velocity
        min_distance_mu -- the mean of the mean min-distance for each car. Min-distance is the min. preferred amount of seconds that a car would want to keep from other cars (incl the other car's velocity). A relative distance of x seconds means that an car will reach another car in x seconds.
        min_distance_sigma -- the standard deviation of the min-distance for each car
        car_acc -- acceleration of the cars (in m\s2).
        car_dec -- deceleration of the cars (in m\s2).
        p_slowdown -- probability of a car slowing down per hour.
        time_step -- in seconds.
        seed -- random seed to use (default `None`, results in time-based seed).
        verbose -- verbosity level (`0` is silent).
        """

        super().__init__()
        np.random.seed(seed)
        self.reset_randomizer(seed)
        self.verbose = verbose

        self.time_step = time_step
        self.flow = self.probability_per(flow * n_lanes, seconds=1)
        self.max_speed = max_speed / 3.6
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.min_distance_mu = min_distance_mu
        self.min_distance_sigma = min_distance_sigma
        self.car_acc = car_acc
        self.car_dec = car_dec
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.lane_change_time = 2  # TODO use rotation matrix

        self.space = road.Road(self, length, n_lanes, lane_width, torus=False)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, ["update_vel_next", "move"],
            shuffle=False,
            shuffle_between_stages=False)

        self.data = data.Data()
        self.data_collector = DataCollector(
            model_reporters={
                "Density": data.density,
                "Flow":
                data.flow  # TODO measuring the flow is maybe not necessary,
                # since the flow rate is a parameter for the simulation,
                # unless we want to measure the flow rate at another reference point
            })

    def step(self):
        self.generate_cars()
        self.schedule.step()
        self.data_collector.collect(self)

    def generate_cars(self):
        if self.random.random() < self.flow:
            self.generate_car()

    def generate_car(self,
                     max_speed_sigma=3,
                     bias_right_lane=0.5,
                     bias_right_lane_sigma=3,
                     x=0):
        vel = np.array([self.max_speed, 0])
        max_speed = self.stochastic_params(
            self.max_speed, max_speed_sigma, seconds=None)
        max_speed = np.clip(max_speed, self.max_speed / 2, None)

        # TODO forward, backward min distance
        min_distance = self.stochastic_params(self.min_distance_mu,
                                              self.min_distance_sigma)
        # bias to go to the right lane (probability based, per minute)
        bias_right_lane = self.stochastic_params(
            bias_right_lane,
            bias_right_lane_sigma,
            seconds=Model.BIAS_RIGHT_LANE_SECONDS)
        try:
            pos_placeholder = np.zeros(2)
            car = Car(self.next_id(), self, pos_placeholder, vel, max_speed,
                      bias_right_lane, min_distance)
            self.set_car_position(car, x)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)
        except UserWarning as e:
            if self.verbose: print(e)

    def set_car_position(self, car, x=0):
        # randomly iterate all lanes until an empty slot is found
        for lane_index in np.random.permutation(self.space.n_lanes):
            car.pos[0] = 0
            car.pos[1] = self.space.center_of_lane(lane_index)
            (other_car, _), (distance_abs, _) = self.space.neighbours(
                car, lane=lane_index)
            if not other_car:
                return car
            distance_s = road.distance_in_seconds(distance_abs, car.vel)
            if distance_s >= self.min_spacing:
                return car
        raise UserWarning('Cannot generate new car')

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
        if p == 0:
            return 0
        # the probability per second should exceed the time step length
        assert (seconds >= self.time_step)
        return p * self.time_step / seconds

    def probability_to_reset_bias_right_lane(self):
        # frequency = 1 / number of seconds
        # probability = frequency
        return 1 / Model.BIAS_RIGHT_LANE_SECONDS * self.time_step
