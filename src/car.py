import numpy as np
import collections
import enum
from mesa import Agent

import src.util as util
import src.road as road
from .road import Direction

# TODO vision in seconds? or min_distance-backward in seconds?


class CarInFront(enum.Enum):
    """
    min_spacing has priority over min_relative_distance because it assumes the worst case
    """
    min_spacing = enum.auto()
    min_relative_distance = enum.auto()
    no = enum.auto()


class Car(Agent):
    def __init__(self,
                 unique_id,
                 model,
                 pos,
                 vel,
                 max_speed,
                 bias_right_lane,
                 min_distance,
                 p_slowdown,
                 autonomous=False):
        """Create an agent that represents a car

        Parameters
        ----------
        unique_id -- unique id of the agent.
        model -- reference to the model the agent is part of.
        pos -- tuple of current position of the car.
        vel -- tuple of current x,y velocities of the car.
        max_speed -- in m/s.
        bias_right_lane -- bias to move to the right-hand lane in range [0,1].
        minimal_distance -- in time units.
        p_slowdown -- probability of random slowing down in a timestep.
        autonomous -- whether the car is autonomous or not.
        """

        super().__init__(unique_id, model)
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.max_speed = max_speed
        self.bias_right_lane = bias_right_lane
        self.min_distance = min_distance
        self.p_slowdown = p_slowdown
        self.autonomous = autonomous
        self.lane = None
        self.target_lane = None

    def step(self):
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

    def update_vel_next(self):
        """update the property `vel_next`: the intended velocity of agent based of the current state."""

        ### accelerate if not maximum speed
        vel_next = self.accelerate_vel(self.vel.copy())

        ### get the neighbours surrounding the current car
        neighbours = self.model.space.all_neighbours(self)

        ### check if current car would need to brake, because there is a car in front
        needs_to_brake = self.needs_to_brake(vel_next, neighbours.f_d,
                                             neighbours.f)

        ### lane-chaning and braking logic

        # if not chaning lane
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
                    vel_next = self.brake(needs_to_brake, vel_next,
                                          neighbours.f_d, neighbours.f)

        # if changing lane and on target_lane
        elif self.lane == self.target_lane:
            vel_next = self.center_on_current_lane(vel_next)

            # if needs to brake
            if needs_to_brake != CarInFront.no:
                vel_next = self.brake(needs_to_brake, vel_next, neighbours.f_d,
                                      neighbours.f)

        # if changing lane and not on target_lane
        else:
            success, vel_next = self.try_steer_to_lane(vel_next, neighbours,
                                                       self.target_lane)
            if not success:
                self.target_lane = self.lane
                vel_next = self.center_on_current_lane(vel_next)

            # if needs to brake
            if needs_to_brake != CarInFront.no:
                vel_next = self.brake(needs_to_brake, vel_next, neighbours.f_d,
                                      neighbours.f)

        ### randomly slow down
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

    def needs_to_brake(self, vel, distance_abs, other_car=None):
        """Returns a value CarInFront
        """
        # TODO use reaction time
        if other_car is None:
            return CarInFront.no
        if not self.is_inside_vision_range(distance_abs, vel):
            return CarInFront.no

        distance_s = self.distance_s(distance_abs, vel)
        if distance_s < self.model.min_spacing + self.model.time_step:
            return CarInFront.min_spacing

        distance_rel_s = self.distance_rel_s(distance_abs, vel, other_car)
        if distance_rel_s >= 0 and distance_rel_s < self.min_distance:
            return CarInFront.min_relative_distance

        return CarInFront.no

    def is_inside_vision_range(self, distance_abs, vel):
        # TODO use limited vision (in s) param
        vision = 200
        distance_s = road.distance_in_seconds(distance_abs, vel)
        # distance_s = road.distance_in_seconds(distance_abs, -vel)
        return distance_s < vision

    def brake(self, reason, vel, distance_abs, other_car):
        if vel[0] <= 0:
            vel[0] = 0
            return vel

        if reason == CarInFront.no:
            pass
        elif reason == CarInFront.min_spacing:
            # print(self.unique_id, '\t breaks for\t', other_car.unique_id,
            #       '\t (min spacing)')
            # keep `min_spacing` seconds distance
            vel[0] = distance_abs / (
                self.model.time_step + self.model.min_spacing)

        elif reason == CarInFront.min_relative_distance:
            # print(self.unique_id, '\t breaks for\t', other_car.unique_id,
            #       '\t (min distance)')
            # print('\t d vel (int): %i - %i' % (self.vel[0], other_car.vel[0]))
            distance_rel_s = self.distance_rel_s(distance_abs, vel, other_car)
            assert (distance_rel_s > 0)
            d_vel_rel = distance_rel_s / self.min_distance
            d_vel = max(self.model.car_dec, d_vel_rel) * self.model.time_step
            vel[0] = self.vel[0] * (1 - d_vel)
            # print('vel: %f, rel vel: %f' % (vel[0],
            #                                 d_vel_rel * self.model.time_step))

        return vel

    def will_randomly_slow_down(self):
        return self.random.random() < self.p_slowdown

    def random_slow_down(self, vel):
        vel[0] -= self.model.car_dec * self.model.time_step
        return vel

    @property
    def length(self):
        return self.model.car_length

    def distance_s(self, distance_abs, vel):
        return road.distance_in_seconds(distance_abs, vel)

    def distance_rel_s(self, distance_abs, vel, other_car):
        return road.distance_in_seconds(distance_abs, vel, other_car.vel)

    def try_steer_to_lane(self, vel, neighbours, lane):
        """Returns whether steering to lane was successful and the new velocity."""
        if not self.model.space.lane_exists(lane):
            return (False, vel)

        # direction of the lane change (-1: left, 0: current, +1: right)
        direction = lane - self.lane

        # get the neighbours (f: front, b: back) in the target lane
        f, b = neighbours.in_direction(direction)
        f_d, b_d = neighbours.distances(direction)

        # check if there is space for overtaking
        if not f or f_d > vel[0] * (self.model.time_step + self.min_distance):
            if not b or b_d > b.vel[0] * (
                    self.model.time_step + self.min_distance):
                movement_direction = Direction.L if self.pos[1] > self.model.space.center_of_lane(
                    lane) else Direction.R
                vel = self.steer(vel, movement_direction)
                self.target_lane = lane
                return (True, vel)

        return (False, vel)

    def steer(self, vel, direction):
        """Steer in the specified direction, such that a complete lane-change can be performed in `lane_change_time`"""
        vel[1] = direction * road.LANE_WIDTH / self.model.lane_change_time
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
