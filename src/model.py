import numpy as np

import mesa
from mesa.time import StagedActivation, RandomActivation
from mesa.datacollection import DataCollector

import src.util as util
from .car import Car
from .road import Road
import src.data as data


class Model(mesa.Model):
    MAX_LANES = 10
    BIAS_RIGHT_LANE_SECONDS = 60

    def __init__(self, length, lane_width, n_lanes, flow, max_speed,
                 car_length, min_spacing, car_acc, car_dec, p_slowdown,
                 time_step):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lane_width -- width of a lane.
        n_lanes -- number of lanes.
        flow -- number of cars generated per lane per second (stochastic)
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
        self.flow = self.probability_per(flow * n_lanes, seconds=1)
        self.max_speed = max_speed / 3.6
        self.car_length = car_length
        self.min_spacing = min_spacing
        self.car_acc = car_acc
        self.car_dec = car_dec
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.lane_change_time = 2  # TODO use rotation matrix

        self.space = Road(self, length, n_lanes, lane_width, torus=False)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, ["update_vel_next", "move"],
            shuffle=False,
            shuffle_between_stages=False)

        self.data = data.Data()
        self.data_collector = DataCollector(model_reporters={
            "Density": data.density,
            "Flow": data.flow
        })
        self.v = 1  # verbose lvl

    def step(self):
        self.generate_cars()
        self.generate_cars()
        self.generate_cars()
        self.schedule.step()
        self.data_collector.collect(self)

    def generate_cars(self):
        if self.random.random() < self.flow:
            self.generate_car()

    def generate_car(self):
        max_speed_sigma = 5
        max_speed = self.stochastic_params(
            self.max_speed, max_speed_sigma, seconds=None)
        vel = np.array([self.max_speed, 0])
        # bias to go to the right lane (probability based, per minute)
        bias_right_lane = self.stochastic_params(
            0.5, sigma=3, seconds=Model.BIAS_RIGHT_LANE_SECONDS)
        minimal_overtake_distance = 2.0  # TODO stochastic?
        try:
            pos = self.generate_car_position(vel)
            print('pos', pos)
            car = Car(self.next_id(), self, pos, vel, max_speed,
                      bias_right_lane, minimal_overtake_distance)

            self.space.place_agent(car, car.pos)
            self.schedule.add(car)
        except UserWarning as e:
            if self.v: print(e)

    def generate_car_position(self, vel, x=0):
        # randomly iterate all lanes until an empty slot is found
        for lane_index in np.random.permutation(self.space.n_lanes):
            x = 0
            y = (lane_index + 0.5) * self.space.lane_width
            (other_car, _), (distance, _) = self.space.neighbours(
                None, lane=lane_index)
            # util.distance_in_seconds(distance, vel[0], other_car.vel[0])
            if distance < self.min_spacing:
                # TODO distance in s
                return (x, y)
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
        assert (seconds > self.time_step)
        return p * self.time_step / seconds

    def probability_to_reset_bias_right_lane(self):
        # frequency = 1 / number of seconds
        # probability = frequency
        return 1 / Model.BIAS_RIGHT_LANE_SECONDS * self.time_step
