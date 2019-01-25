import numpy as np
import collections
import enum
from mesa import Agent

import src.road as road
from .road import Direction


class CarInFront(enum.Enum):
    """
    min_spacing has priority over min_relative_distance because it assumes the worst case
    """
    min_spacing = enum.auto()
    min_relative_distance = enum.auto()
    no = enum.auto()


class Car(Agent):
    LENGTH = 4.4  # in meters
    WIDTH = 1.8  # in meters

    def __init__(self,
                 unique_id,
                 model,
                 pos,
                 vel,
                 preferred_speed,
                 bias_right_lane,
                 min_distance,
                 distance_error_sigma,
                 p_slowdown,
                 autonomous=False):
        """Create an agent that represents a car

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- reference to the model the agent is part of.
        pos -- tuple of current position of the car.
        vel -- tuple of current x,y velocities of the car.
        preferred_speed -- in m/s.
        bias_right_lane -- bias to move to the right-hand lane in range [0,1].
        minimal_distance -- in time units.
        p_slowdown -- probability of random slowing down in a timestep.
        autonomous -- whether the car is autonomous or not.
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.preferred_speed = preferred_speed
        self.bias_right_lane = bias_right_lane
        self.min_distance = min_distance
        self.distance_error_sigma = distance_error_sigma
        self.p_slowdown = p_slowdown
        self.autonomous = autonomous
        self.lane = None
        self.target_lane = None
        self.distance_rel_error = 0
        self.distance_max_abs_rel_error = 0.1  # in %/100

    @property
    def length(self):
        return self.LENGTH

    @property
    def width(self):
        return self.WIDTH

    def step(self):
        self.update_distance_rel_error()
        self.update_vel_next()
        self.move()

    def move(self):
        self.vel = self.vel_next
        pos_next = self.pos + self.vel * self.model.time_step

        # check if car passed the flow measurement point
        if self.pos[0] < self.model.data.flow_reference_point and \
            pos_next[0] > self.model.data.flow_reference_point:
            self.model.data.flow += 1

        self.pos = pos_next
        self.model.space.move_agent(self, self.pos)

    def update_distance_rel_error(self):
        e = self.distance_rel_error
        f = self.distance_error_sigma * np.random.randn()
        dt = self.model.time_step
        m = self.distance_max_abs_rel_error
        self.distance_rel_error = np.clip(e + f * dt, -m, m)

    def update_vel_next(self):
        """update the property `vel_next`: the intended velocity of agent based of the current state."""

        ### accelerate if not maximum speed
        vel_next = self.accelerate_vel(self.vel.copy())

        ### get the neighbours surrounding the current car
        neighbours = self.model.space.all_neighbours(self)

        ### check if current car would need to brake, because there is a car in front
        needs_to_brake, distances = self.needs_to_brake(
            vel_next, neighbours.f_d, neighbours.f)

        ### lane-chaning and braking logic

        # if not changing lane
        if self.target_lane == None:
            # if no need to brake
            if needs_to_brake == CarInFront.no:
                # go to right lane (if possible) with a certain probability (bias)
                if self.random.random() < self.bias_right_lane:
                    _, vel_next = self.try_steer_to_lane(
                        vel_next, neighbours, self.lane + Direction.R)

            else:  # if needs to brake
                # try overtaking
                success, vel_next = self.try_steer_to_lane(
                    vel_next, neighbours, self.lane + Direction.L)
                if not success:
                    vel_next = self.brake(needs_to_brake, vel_next, distances)

        # if changing lane and on target_lane
        elif self.lane == self.target_lane:
            vel_next = self.center_on_current_lane(vel_next)

            # if needs to brake
            if needs_to_brake != CarInFront.no:
                vel_next = self.brake(needs_to_brake, vel_next, distances)

        # if changing lane and not on target_lane
        else:
            success, vel_next = self.try_steer_to_lane(vel_next, neighbours,
                                                       self.target_lane)
            if not success:
                self.target_lane = self.lane
                vel_next = self.center_on_current_lane(vel_next)

            # if needs to brake
            if needs_to_brake != CarInFront.no:
                vel_next = self.brake(needs_to_brake, vel_next, distances)

        ### randomly slow down
        if self.will_randomly_slow_down():
            self.random_slow_down(vel_next)

        # prevent negative velocities
        vel_next[0] = max(0, vel_next[0])

        self.vel_next = vel_next

    def accelerate_vel(self, vel):
        # returns accelerated vel, upper limited by the maximum speed
        vel[0] = min(vel[0] + self.model.car_acc * self.model.time_step,
                     self.preferred_speed)
        return vel

    def needs_to_brake(self, vel, distance_abs, other_car=None):
        """Returns a a tuple (CarInFront, info_dict) with
        info_dict : default_dict(None){
            CarInFront.min_spacing: distance_abs,
            CarInFront.min_relative_distance: distance_del_s
          }
        """
        d = collections.defaultdict(None)
        no = (CarInFront.no, d)
        if other_car is None:
            return no

        distance_s = self.distance_s(distance_abs, vel)
        if distance_s < self.model.min_spacing + self.model.time_step:
            d['distance_abs'] = distance_abs
            return (CarInFront.min_spacing, d)

        distance_rel_s = self.distance_rel_s(distance_abs, vel, other_car)
        if distance_rel_s >= 0 and distance_rel_s < self.min_distance:
            d['distance_rel_s'] = distance_rel_s
            return (CarInFront.min_relative_distance, d)

        return (CarInFront.no, d)

    def brake(self, reason, vel, distances):
        if reason == CarInFront.no:
            pass
        elif reason == CarInFront.min_spacing:
            distance_abs = distances['distance_abs']
            # compute a value for vel[0] s.t.:
            #  distance - vel[0] * time_step = vel[0] * min_spacing
            vel[0] = distance_abs / (
                self.model.time_step + self.model.min_spacing)

        elif reason == CarInFront.min_relative_distance:
            distance_rel_s = distances['distance_rel_s']
            d_vel_rel = distance_rel_s / self.min_distance
            d_vel = min(self.model.car_dec, d_vel_rel) * self.model.time_step
            vel[0] = self.vel[0] * (1 - d_vel)

        return vel

    def will_randomly_slow_down(self):
        return self.random.random() < self.p_slowdown

    def random_slow_down(self, vel):
        vel[0] -= self.model.car_dec * self.model.time_step
        return vel

    def distance_s(self, distance_abs, vel):
        return road.distance_in_seconds(distance_abs, vel)

    def distance_rel_s(self, distance_abs, vel, other_car):
        d = road.distance_in_seconds(distance_abs, vel, other_car.vel)
        return d * (1 + self.distance_rel_error)

    def try_steer_to_lane(self, vel, neighbours, lane):
        """Returns whether steering to lane is possible and the new velocity."""
        if self.pos[1] > self.model.space.center_of_lane(lane):
            direction = Direction.L
        else:
            direction = Direction.R

        if not self.model.space.lane_exists(lane):
            return (False, vel)
        elif not self.can_steer_to_lane(vel, neighbours, direction):
            return (False, vel)

        vel = self.steer(vel, direction)
        self.target_lane = lane
        return (True, vel)

    def can_steer_to_lane(self, vel, neighbours, direction):
        # get the neighbours (f: front, b: back) in the target lane
        f, b = neighbours.in_direction(direction)
        f_d, b_d = neighbours.distances(direction)
        if f and f_d < vel[0] * (self.model.time_step + self.min_distance):
            return False
        elif b and b_d < b.vel[0] * (self.model.time_step + self.min_distance):
            return False
        return True

    def steer(self, vel, direction):
        """Steer in the specified direction, such that a complete lane-change can be performed in `lane_change_time`"""
        # TODO decrease vel in x dimension (rotation matrix)
        vel[1] = direction * self.model.space.lane_width / self.model.lane_change_time
        return vel

    # def steer(self, vel, degrees): TODO implement
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

    def center_on_current_lane(self, vel):
        d = self.model.space.distance_from_center_of_lane(self.pos)
        direction = Direction.L if self.model.space.is_right_of_center_of_lane(
            self.pos) else Direction.R
        vel = self.steer(vel, direction)

        # check if moving in direction of the center, would result in overshooting
        if direction * (vel[1] * self.model.time_step + d) >= 0:
            # center the car in the lane
            self.pos[1] = self.model.space.center_of_lane(
                self.lane
            )  # TODO setting the position might seem a bit dangerous,
            # but this is okay for now since the y-pos of this car will not be used by other cars.
            vel[1] = 0
            self.target_lane = None
            return vel
        return vel
