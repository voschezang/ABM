import numpy as np

from mesa import Model
from mesa.space import ContinuousSpace
from mesa.time import RandomActivation

from .agent import Car


class MyModel(Model):
    def __init__(self, length=100, lanes=1, n_cars=10,
                 max_speed=10, car_size=5, min_spacing=1, car_acc=1, p_slowdown=0.2):
        """Initialise the traffic model.

        Parameters
        ----------
        length -- length of the road.
        lanes -- number of lanes.
        n_cars -- number of cars.
        max_speed -- maximum speed cars can (and want to) travel at.
        car_size -- length of each car.
        min_spacing -- the minimum distance cars keep from each other (bumper to bumper).
        car_acc -- acceleration of the cars.
        p_slowdown -- probability of a car slowing down at random.
        """

        super().__init__()

        self.length = length
        self.lanes = lanes
        self.n_cars = n_cars
        self.max_speed = max_speed
        self.car_size = car_size
        self.min_spacing = min_spacing
        self.car_acc = car_acc
        self.p_slowdown = p_slowdown

        self.space = ContinuousSpace(length, lanes+1, torus=True)
        self.schedule = RandomActivation(self)
        
        self.make_agents()

    def make_agents(self):
        """Create self.n_cars number of agents and add them to the model (space, schedule)"""

        for i in range(self.n_cars):
            x = self.random.random() * self.space.x_max
            y = self.random.randint(1, self.lanes)
            pos = (x, y)
            vel = (self.max_speed, 0)

            car = Car(self.next_id(), self, pos, vel)
            self.space.place_agent(car, pos)
            self.schedule.add(car)

    def step(self):
        self.schedule.step()
