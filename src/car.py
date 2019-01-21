import numpy as np
import collections
import enum
from mesa import Agent

import src.util as util
from .road import Direction


class Action(enum.Enum):
    """ Agent actions
    center -- reset to center of lane
    right -- go to the right-most lane
    overtake -- overtake either the left or right car
    """
    center = enum.auto()
    right = enum.auto()
    overtake = enum.auto()


# Action = Enum('Action', 'center right overtake')


class Car(Agent):
    def __init__(self, unique_id, model, pos, vel, max_speed, bias_right_lane,
                 minimal_overtake_distance):
        """Create an agent that represents a car

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- reference to the model the agent is part of.
        pos -- tuple of current position of the car.
        vel -- tuple of current x,y velocities of the car.
        max_speed -- in m/s.
        bias_right_lane -- bias to move to the right-hand lane in range [0,1].
        minimal_overtake_distance -- in time units.
        action -- the current action
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.max_speed = max_speed
        self.bias_right_lane = bias_right_lane
        self.minimal_overtake_distance = minimal_overtake_distance
        self.lane = None
        self.action = Action.center

    def step(self):
        self.update_vel_next()
        self.move()

    def move(self):
        self.vel = self.vel_next
        self.pos += self.vel * self.model.time_step

        if not self.model.space.torus and self.model.space.out_of_bounds(
                self.pos):
            self.model.space.remove_agent(self)
            self.model.schedule.remove(self)
            return

        self.model.space.move_agent(self, self.pos)

    def update_vel_next(self):
        """update the property `vel_next`: the intended velocity of agent based of the current state."""

        ### 1. accelerate if not maximum speed
        vel_next = self.accelerate_vel(self.vel.copy())

        ### 2. prevent collision with other cars
        # TODO (optional) stop searching adjacent lanes when a blocking car is found
        #   i.e. a reason that makes overtaking impossible
        # We may have to simplify the interface, i.e. store cars per lane and simply iterate all cars in that lane, instead of searching in a radius
        neighbours = self.model.space.all_neighbours(self)

        # if car in front
        if neighbours.f and (self.needs_to_brake(
                vel_next, neighbours.f_d - self.model.car_length)):
            success, vel_next = self.can_overtake(vel_next, neighbours)
            if success:
                self.action = Action.overtake
            else:
                self.action = Action.center
                if self.unique_id == 1:
                    print(self.unique_id, "brake")
                vel_next = self.model.space.center_on_current_lane(
                    self.pos, vel_next)
                vel_next = self.brake(vel_next,
                                      neighbours.f_d - self.model.car_length)

        # no car in front
        else:
            self.possibly_reset_action()
            if self.action == Action.right or \
               self.random.random() < self.bias_right_lane:
                # TODO scale probability with dt
                if self.unique_id == 1:
                    print(self.unique_id, "try right bias")
                success, vel_next = self.model.space.steer_to_lane(
                    self, vel_next, neighbours, [Direction.R])
                if success:
                    self.action = Action.right
                else:
                    self.action = Action.center
                    vel_next = self.model.space.center_on_current_lane(
                        self.pos, vel_next)
                    if self.unique_id == 1:
                        print(self.unique_id, "center 1")
            else:
                self.action = Action.center
                if self.unique_id == 1:
                    print(self.unique_id, "center 2")
                vel_next = self.model.space.center_on_current_lane(
                    self.pos, vel_next)

        ### 3. randomly slow down
        if self.will_randomly_slow_down():
            self.random_slow_down(vel_next)

        # clip negative velocities to zero
        vel_next[0] = max(0, vel_next[0])

        self.vel_next = vel_next

    def accelerate_vel(self, vel):
        # returns accelerated vel, upper limited by the maximum speed
        vel[0] = min(vel[0] + self.model.car_acc * self.model.time_step,
                     self.max_speed)
        return vel

    def needs_to_brake(self, vel, distance):
        """Returns if needs to brake in order to keep minimum spacing"""
        # TODO use reaction time
        return distance < vel[0] * (
            self.model.time_step + self.model.min_spacing)

    def brake(self, vel, distance):
        # TODO distance in seconds
        if self.needs_to_brake(vel, distance):
            vel[0] = distance / (self.model.time_step + self.model.min_spacing)
        return vel

    def will_randomly_slow_down(self):
        return self.random.random() < self.model.p_slowdown

    def random_slow_down(self, vel):
        vel[0] -= self.model.car_dec * self.model.time_step
        return vel

    def steer(self, vel, degrees):
        """Rotate the velocity vector with a number of degrees."""
        # example:
        # change lanes in 2 seconds, determine angle
        # angle = ArcTan(3.5 / 2 / 33) = 3
        angle = np.radians(degrees)
        cos = np.cos(angle)
        sin = np.sin(angle)
        rotation_matrix = np.array([[cos, -sin], [sin, cos]])
        vel = rotation_matrix @ vel
        return vel

    def can_overtake(self, vel_next, neighbours):
        # Returns a tuple (success: bool, required_vel: [int,int] )
        if not self.model.space.is_right_of_center_of_lane(
                self.pos):  # i.e. in the middle or left
            if self.unique_id == 1:
                print(self.unique_id, "try left")
            success, vel_next = self.model.space.steer_to_lane(
                self, vel_next, neighbours, [Direction.L, Direction.R])
        else:
            if self.unique_id == 1:
                print(self.unique_id, "try right")
            success, vel_next = self.model.space.steer_to_lane(
                self, vel_next, neighbours, [Direction.R, Direction.L])
        return (success, vel_next)

    def possibly_reset_action(self):
        """ Reset the field `action` by chance iff current action = 'right'
        """
        if self.action == Action.right:
            if self.random.random(
            ) < 1 / self.bias_right_lane_seconds * self.model.time_step:
                self.reset_action()

    def reset_action(self):
        self.action = Action.center
