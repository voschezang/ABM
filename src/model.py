import numpy as np

import mesa
from mesa.time import StagedActivation, RandomActivation


import src.util as util
import src.road as road
from .car import Car
import src.data as data


class Model(mesa.Model):
    """Traffic flow simulation with multiple lanes and lane-chaning."""

    MAX_LANES = 10
    BIAS_RIGHT_LANE_SECONDS = 1

    def __init__(self,
                 length=1000,
                 lane_width=3.5,
                 n_lanes=1,
                 n_cars=10,
                 fraction_autonomous=0,
                 max_speed_mu=10,
                 max_speed_sigma=3,
                 car_length=5,
                 min_spacing=1,
                 min_distance_mu=2,
                 min_distance_sigma=0,
                 car_acc=3,
                 car_dec=6,
                 p_slowdown=0.1,
                 bias_right_lane_mu=1,
                 bias_right_lane_sigma=0,
                 time_step=0.1,
                 seed=None,
                 verbose=3):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road in meters.
        lane_width -- width of a lane in meters.
        n_lanes -- number of lanes.
        n_cars -- number of cars on the road.
        fraction_autonomous -- fraction of `n_cars` that are autonomous vehicles.
        max_speed_mu -- maximum speed cars will try to travel at in km/h (will be converted to m/s).
        max_speed_sigma -- standard deviation of max_speed.
        car_length -- length of each car.
        min_spacing -- the minimum distance in seconds a car keeps from other cars (front to back). Incorporating the cars' velocity but ignoring the other cars' velocity
        min_distance_mu -- the mean of the mean min-distance for each car. Min-distance is the min. preferred amount of seconds that a car would want to keep from other cars (incl the other car's velocity). A relative distance of x seconds means that an car will reach another car in x seconds.
        min_distance_sigma -- the standard deviation of the min-distance for each car
        car_acc -- acceleration of the cars (in m\s2).
        car_dec -- deceleration of the cars (in m\s2).
        p_slowdown -- probability of a car slowing down per hour.
        bias_right_lane_mu -- per second. I.e. frequency of checking if going to the right lane is possible.
        bias_right_lane_sigma -- standard deviation
        time_step -- in seconds.
        seed -- random seed to use (default `None`, results in time-based seed).
        verbose -- verbosity level (`0` is silent).
        """

        super().__init__()
        np.random.seed(seed)
        self.reset_randomizer(seed)
        self.verbose = verbose

        self.time_step = time_step
        self.n_cars = n_cars
        self.fraction_autonomous = fraction_autonomous
        self.max_speed_mu = max_speed_mu / 3.6
        self.max_speed_sigma = max_speed_sigma
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.min_distance_mu = min_distance_mu
        self.min_distance_sigma = min_distance_sigma
        self.car_acc = car_acc
        self.car_dec = car_dec
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.bias_right_lane_mu = bias_right_lane_mu
        self.bias_right_lane_sigma = bias_right_lane_sigma
        self.lane_change_time = 2  # TODO use rotation matrix

        self.space = road.Road(self, length, n_lanes, lane_width, torus=True)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, ["update_vel_next", "move"],
            shuffle=False,
            shuffle_between_stages=False)

        # create the data collector object
        self.data = data.Data(flow_reference_point=length / 2)

        self.make_agents()

    def step(self):
        #self.generate_cars()
        self.schedule.step()
        self.data.collect(self)

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        # x coordinates for the agents
        xs = (np.random.permutation(self.n_cars) + np.random.random(self.n_cars)) / self.n_cars * self.space.length

        for i in range(self.n_cars):
            y = self.space.center_of_lane(self.random.randint(0, self.space.n_lanes-1))
            pos = (xs[i], y)

            vel = np.array([self.max_speed_mu, 0])
            max_speed = self.stochastic_params(
                self.max_speed_mu, self.max_speed_sigma, seconds=None)
            max_speed = np.clip(max_speed, self.max_speed_mu / 2, None)

            min_distance = np.random.normal(self.min_distance_mu, self.min_distance_sigma)
            # bias to go to the right lane (probability based, per minute)
            bias_right_lane = self.stochastic_params(
                self.bias_right_lane_mu,
                self.bias_right_lane_sigma,
                seconds=Model.BIAS_RIGHT_LANE_SECONDS)

            p_slowdown = np.random.normal(self.p_slowdown, 0)

            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, min_distance, p_slowdown)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)



    def remove_car(self, car):
        self.space.remove_agent(car)
        self.schedule.remove(car)

    def generate_cars(self):
        if self.random.random() < self.flow:
            self.generate_car()

    def generate_car(self, x=0):
        vel = np.array([self.max_speed_mu, 0])
        max_speed = self.stochastic_params(
            self.max_speed_mu, self.max_speed_sigma, seconds=None)
        max_speed = np.clip(max_speed, self.max_speed_mu / 2, None)

        min_distance = np.random.normal(self.min_distance_mu,
                                        self.min_distance_sigma)
        # bias to go to the right lane (probability based, per minute)
        bias_right_lane = self.stochastic_params(
            self.bias_right_lane_mu,
            self.bias_right_lane_sigma,
            seconds=Model.BIAS_RIGHT_LANE_SECONDS)
        try:
            pos_placeholder = np.zeros(2)
            car = Car(self.next_id(), self, pos_placeholder, vel, max_speed,
                      bias_right_lane, min_distance)
            self.set_car_position(car, x)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)
        except UserWarning as e:
            if self.verbose > 1: print(e)

    def set_car_position(self, car, x=0):
        # randomly iterate all lanes until an empty slot is found
        for lane_index in np.random.permutation(self.space.n_lanes):
            car.pos[0] = 0
            car.pos[1] = self.space.center_of_lane(lane_index)
            first_car = self.space.first_car_in_lane(lane_index)
            if not first_car or road.distance_in_seconds(
                    first_car.pos[0], car.vel) >= self.min_spacing:
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
        """Returns the probability of something happening in a timestep, based on a chance per `seconds`."""
        # note that this limits the amount of timesteps
        if p == 0:
            return 0
        # the probability per second should exceed the time step length
        assert (seconds >= self.time_step)
        return p * self.time_step / seconds
