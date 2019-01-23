import numpy as np
import collections
import enum
from mesa import Agent

import src.util as util
import src.road as road
from .road import Direction

# TODO vision in seconds? or min_distance-backward in seconds?


class Action(enum.Enum):
    """ Agent actions
    center -- reset to center of lane
    right -- go to the right-most lane
    overtake -- overtake either the left or right car
    """
    center = enum.auto()
    right = enum.auto()
    overtake = enum.auto()


class CarInFront(enum.Enum):
    """
    min_spacing has priority over min_relative_distance because it assumes the worst case
    """
    min_spacing = enum.auto()
    min_relative_distance = enum.auto()
    no = enum.auto()


class Car(Agent):
    def __init__(self, unique_id, model, pos, vel, max_speed, bias_right_lane,
                 min_distance):
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
        self.min_distance = min_distance
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
        neighbours = self.model.space.all_neighbours(self)

        ### 2. car in front
        needs_to_brake = self.needs_to_brake(vel_next, neighbours.f_d,
                                             neighbours.f)
        if needs_to_brake == CarInFront.no:
            self.possibly_reset_action()
            if self.action == Action.right or \
               self.random.random() < self.bias_right_lane:
                # TODO scale probability with dt
                if self.model.verbose > 2 and self.unique_id == 1:
                    print(self.unique_id, "try right bias")
                success, vel_next = self.model.space.steer_to_lane(
                    self, vel_next, neighbours, [Direction.R])
                if success:
                    self.action = Action.right
                else:
                    self.action = Action.center
                    vel_next = self.model.space.center_on_current_lane(
                        self.pos, vel_next)
                    if self.model.verbose > 2 and self.unique_id == 1:
                        print(self.unique_id, "center 1")
            else:
                self.action = Action.center
                if self.model.verbose > 2 and self.unique_id == 1:
                    print(self.unique_id, "center 2")
                vel_next = self.model.space.center_on_current_lane(
                    self.pos, vel_next)

        else:
            success, vel_next = self.can_overtake(vel_next, neighbours)
            if success:
                self.action = Action.overtake
            else:
                if self.model.verbose > 2 and self.unique_id == 1:
                    print(self.unique_id, "brake")
                vel_next = self.brake(needs_to_brake, vel_next, neighbours.f_d,
                                      neighbours.f)
                vel_next = self.model.space.center_on_current_lane(
                    self.pos, vel_next)
                self.action = Action.center

        ### 3. randomly slow down
        if self.will_randomly_slow_down():
            self.random_slow_down(vel_next)

        # clip negative velocities to zero
        vel_next[0] = max(0, vel_next[0])

        # TODO limit acc to car specs (2d?)
        # vel_next =  np.clip(( vel_next - self.vel )/self.model.time_step, self.model.dec, self.model.acc)
        self.vel_next = vel_next
        # print(self.unique_id, 'vel', vel_next)

    def accelerate_vel(self, vel):
        # returns accelerated vel, upper limited by the maximum speed
        vel[0] = min(vel[0] + self.model.car_acc * self.model.time_step,
                     self.max_speed)
        return vel

    def needs_to_brake(self, vel, distance_abs, other_car=None):
        """Returns a value CarInFront
        """
        # TODO use reaction time
        if other_car is None:
            return CarInFront.no
        if not self.is_inside_vision_range(distance_abs, vel):
            return CarInFront.no

        distance_s = self.distance_s(distance_abs, vel)
        if distance_s < self.model.min_spacing:
            print('min spacing >')
            return CarInFront.min_spacing

        distance_rel_s = self.distance_rel_s(distance_abs, vel, other_car)
        if distance_rel_s < self.min_distance:
            return CarInFront.min_relative_distance

        return CarInFront.no
        # return (distance_s < self.model.min_spacing,
        #         distance_rel_s < self.min_distance)

    def is_inside_vision_range(self, distance_abs, vel):
        # TODO use limited vision (in s) param
        vision = 200
        distance_s = road.distance_in_seconds(distance_abs, vel)
        # distance_s = road.distance_in_seconds(distance_abs, -vel)
        return distance_s < vision

    def brake(self, reason, vel, distance_abs, other_car):
        print(self.unique_id, '\t breaks for\t', other_car.unique_id)
        if vel[0] <= 0:
            vel[0] = 0
            return vel

        if reason == CarInFront.no:
            pass
        elif reason == CarInFront.min_spacing:
            distance_s = self.distance_s(distance_abs, vel)
            vel[0] *= .5  # TODO

        elif reason == CarInFront.min_relative_distance:
            # vel[0] = distance / (self.model.time_step + self.model.min_spacing)
            distance_rel_s = road.distance_in_seconds(distance_abs, vel,
                                                      other_car.vel)
            vel[0] *= .5
            # TODO is geometric decelerate function allowed?
            # d_vel_rel = (vel[0] - other_car.vel[0]) / vel[0]
            # vel[0] *= d_vel_rel * self.model.time_step
            # if d_vel_rel <= 0:
            # vel[0] = 0

        # n_time_steps = distance_rel_s * self.model.time_step
        # assert (d_vel_rel >= 0)
        # assert (n_time_steps > 0)
        # vel[0] *= 1 - d_vel_rel / n_time_steps
        return vel

    def will_randomly_slow_down(self):
        return self.random.random() < self.model.p_slowdown

    def random_slow_down(self, vel):
        vel[0] -= self.model.car_dec * self.model.time_step
        return vel

    # def steer(self, vel, degrees):
    #     """Rotate the velocity vector with a number of degrees."""
    #     # example:
    #     # change lanes in 2 seconds, determine angle
    #     # angle = ArcTan(3.5 / 2 / 33) = 3
    #     angle = np.radians(degrees)
    #     cos = np.cos(angle)
    #     sin = np.sin(angle)
    #     rotation_matrix = np.array([[cos, -sin], [sin, cos]])
    #     vel = rotation_matrix @ vel
    #     return vel

    def can_overtake(self, vel_next, neighbours):
        # Returns a tuple (success: bool, required_vel: [int,int] )
        if not self.model.space.is_right_of_center_of_lane(
                self.pos):  # i.e. in the middle or left
            if self.model.verbose > 2 and self.unique_id == 1:
                print(self.unique_id, "try left")
            success, vel_next = self.model.space.steer_to_lane(
                self, vel_next, neighbours, [Direction.L, Direction.R])
        else:
            if self.model.verbose > 2 and self.unique_id == 1:
                print(self.unique_id, "try right")
            success, vel_next = self.model.space.steer_to_lane(
                self, vel_next, neighbours, [Direction.R, Direction.L])
        return (success, vel_next)

    def possibly_reset_action(self):
        """ Reset the field `action` by chance iff current action = 'right'
        """
        if self.action == Action.right:
            if self.random.random(
            ) < self.model.probability_to_reset_bias_right_lane():
                self.reset_action()

    def reset_action(self):
        self.action = Action.center

    def length(self):
        return self.model.car_length

    def distance_s(self, distance_abs, vel):
        # TODO use this function only once per step
        return road.distance_in_seconds(distance_abs, vel)

    def distance_rel_s(self, distance_abs, vel, other_car):
        return road.distance_in_seconds(distance_abs, vel, other_car.vel)
