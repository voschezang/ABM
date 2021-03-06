import numpy as np

import mesa
from mesa.time import StagedActivation, RandomActivation

import src.road as road
from .car import Car
import src.data as data


class Model(mesa.Model):
    # This documentation can be viewed in the browser simulation by clicking on "About"
    """Traffic flow simulation with multiple lanes and lane-chaning. 
    Blue cars are driver by humans and red cars are autonomous.
    The graph shows the flow, measured as the number of cars (passing the midpoint of the road) per lane, each timestep.
    """

    BIAS_RIGHT_LANE_SECONDS = 1
    MAX_LANES = 10

    def __init__(self,
                 length: int = 1000,
                 n_lanes: int = 1,
                 density: float = 30,
                 fraction_autonomous=0,
                 max_speed_mu=120,
                 min_spacing=2,
                 min_distance_mu=2,
                 min_distance_min=1,
                 min_distance_max=3,
                 car_acc=12,
                 car_dec=9.2,
                 p_slowdown=0.1,
                 bias_right_lane=1,
                 time_step=0.1,
                 seed: int = None,
                 verbose=3):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road in meters.
        n_lanes -- number of lanes.
        density -- number of cars per 1000 meter road.
        n_cars -- number of cars on the road.
        fraction_autonomous -- fraction of `n_cars` that are autonomous vehicles.
        max_speed_mu -- maximum speed cars will try to travel at in km/h (will be converted to m/s).
        min_spacing -- the minimum distance in seconds a car keeps from other cars (front to back). Incorporating the cars' velocity but ignoring the other cars' velocity
        min_distance_mu -- the mean of the mean min-distance for each car. Min-distance is the min. preferred amount of seconds that a car would want to keep from other cars (incl the other car's velocity). A relative distance of x seconds means that an car will reach another car in x seconds.
        car_acc -- acceleration of the cars (in m\s2).
        car_dec -- deceleration of the cars (in m\s2).
        p_slowdown -- probability of a car slowing down per hour.
        bias_right_lane -- per second. I.e. frequency of checking if going to the right lane is possible.
        time_step -- in seconds.
        seed -- random seed to use (default `None`, results in time-based seed).
        verbose -- verbosity level (`0` is silent).
        """

        super().__init__()

        np.random.seed(seed)
        self.reset_randomizer(seed)
        self.verbose = verbose

        self.time_step = time_step
        self.n_cars = int(round(density * n_lanes * length / 1000))
        self.density = self.n_cars / (n_lanes * length / 1000)
        self.fraction_autonomous = fraction_autonomous
        self.max_speed_mu = max_speed_mu / 3.6
        self.max_speed_min = (max_speed_mu - 15) / 3.6
        self.max_speed_max = (max_speed_mu + 10) / 3.6
        self.min_spacing = min_spacing
        self.min_distance_mu = min_distance_mu
        self.min_distance_min = min_distance_min
        self.min_distance_max = min_distance_max
        self.car_acc = car_acc
        self.car_dec = car_dec
        self.p_slowdown = self.probability_per(p_slowdown, seconds=3600)
        self.bias_right_lane = bias_right_lane
        self.lane_change_time = 2
        self.max_abs_rel_est_error = 0.1

        self.space = road.Road(self, length, n_lanes, torus=True)

        # uncomment one of the two lines below to select the timing schedule (random, or staged)
        # self.schedule = RandomActivation(self)
        self.schedule = StagedActivation(
            self, Car.STAGE_LIST, shuffle=False, shuffle_between_stages=False)

        # create the data collector object
        self.data = data.Data(flow_reference_point=length / 2)

        self.make_agents()

    def step(self):
        self.schedule.step()
        self.data.collect(self)

    def make_agents(self) -> None:
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        # x coordinates for the agents
        np.random.permutation(self.n_cars)
        xs = (
            np.random.permutation(self.n_cars) + np.random.random(self.n_cars)
        ) / self.n_cars * self.space.length

        normal_cars = int(round(self.n_cars * (1 - self.fraction_autonomous)))
        autonomous_cars = self.n_cars - normal_cars

        for i in range(self.n_cars):
            y = self.space.center_of_lane(
                self.random.randint(0, self.space.n_lanes - 1))
            pos = (xs[i], y)
            vel = np.array([self.max_speed_mu, 0])

            autonomous = False
            # if normal car
            if i < normal_cars:
                preferred_speed = self.stochastic_params(
                    self.max_speed_mu,
                    limit=(self.max_speed_min, self.max_speed_max))

                min_distance = self.stochastic_params(
                    self.min_distance_mu,
                    limit=(self.min_distance_min, self.min_distance_max))

                skill = (np.random.random())
                distance_error_sigma = self.max_abs_rel_est_error * (1 - skill)
                p_slowdown = (0.5 + (1 - skill)) * self.p_slowdown

                # bias right lane is equal for all normal cars
                bias_right_lane = self.probability_per(
                    self.bias_right_lane, self.BIAS_RIGHT_LANE_SECONDS)

            # if autonomous car
            else:
                autonomous = True
                preferred_speed = self.max_speed_mu
                min_distance = self.min_distance_mu
                distance_error_sigma = 0
                p_slowdown = 0
                bias_right_lane = 1

            # create the car agent
            car = Car(self.next_id(), self, pos, vel, preferred_speed,
                      bias_right_lane, min_distance, distance_error_sigma,
                      p_slowdown, autonomous)
            self.add_car(car)

    def add_car(self, car):
        # add the created car to the space and scheduler
        self.space.place_agent(car, car.pos)
        self.schedule.add(car)

    def stochastic_params(self,
                          mean,
                          sigma=1,
                          limit=[],
                          pos=True,
                          seconds=None):
        # returns a stochastic parameter, within a limit or with a standard deviation (sigma)
        if limit:
            return self.triangular(mean, *limit)
        elif not sigma:
            return mean
        p = np.random.normal(mean, sigma)
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

    def triangular(self, mode, minn=0, maxx=1):
        assert (minn <= mode), "minn < mode: %f < %f" % (minn, mode)
        assert (mode <= maxx)
        return self.random.triangular(minn, mode, maxx)
