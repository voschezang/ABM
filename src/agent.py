import numpy as np

from mesa import Agent


class Car(Agent):

    def __init__(self, unique_id, model, pos, vel):
        """Create a `Car` agent

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- the model the agent is part of.
        pos -- initial position of the car.
        vel -- initial velocity of the car.
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)

    def forward_distance_to_car(self, car):
        """Return forward distance (in x-direction) to a car."""

        distance = car.pos[0] - self.pos[0]
        if distance < 0:
            distance = self.model.length + distance
        return distance - self.model.car_size

    def car_in_front(self):
        """Return a tuple of the first car in front and the distance to."""

        # get cars ahead within vision range of: max_speed + car_size + min_spacing
        vision = self.model.max_speed + self.model.car_size + self.model.min_spacing
        cars = self.model.space.get_neighbors(self.pos + np.array((vision/2, 0)), vision/2)
        # create list of tuple (car, distance) for cars on the same lane
        cars = [(x, self.forward_distance_to_car(x)) for x in cars if x != self and x.pos[1] == self.pos[1]]
        # sort on (forward) distance
        cars.sort(key=lambda x: x[1])
        # return the closest car in front if there is any
        return cars[0] if cars else None

    def step(self):
        """Apply Nagel-Schreckenberg rules"""

        # 1. accelerate if not maximum speed
        if self.vel[0] < self.model.max_speed:
            self.vel[0] = min(self.vel[0] + self.model.car_acc, self.model.max_speed)

        # 2. slow down if car in front
        car = self.car_in_front()
        if car:
            distance = car[1]
            if self.vel[0] > distance - self.model.min_spacing:
                self.vel[0] = distance - self.model.min_spacing

        # 3. random slow down
        if self.random.random() < self.model.p_slowdown:
            self.vel[0] -= self.model.car_acc

        # clip negative velocities to zero
        self.vel[0] = max(self.vel[0], 0)

        # 4. move car to new position
        pos = self.pos + self.vel
        self.model.space.move_agent(self, pos)
