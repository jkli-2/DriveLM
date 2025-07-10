#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Custom 1 Scenario:
Two NPC vehicles are ahead of the ego.
The front vehicle brakes.
The second vehicle brakes, then does a lane change (to avoid rear-ending the first).
The ego needs to handle this chain reaction (braking, swerving, etc.).
"""

import random
import py_trees
import carla
import math
import time

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      LaneChange,
                                                                      ActorDestroy,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp,
                                                                      ChangeActorTargetSpeed,
                                                                      BasicAgent
                                                                      )
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, InTriggerDistanceToNextIntersection, DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance
from srunner.tools.background_manager import LeaveSpaceInFront, ChangeRoadBehavior
from py_trees.blackboard import Blackboard

def convert_dict_to_location(actor_dict):
    """
    Convert a JSON string to a Carla.Location
    """
    location = carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )
    return location

class AgentControllerWrapper:
    def __init__(self, agent, actor):
        self.agent = agent
        self.actor = actor
        self._target_speed = agent._target_speed

    def run_step(self):
        try:
            return self.agent.run_step()
        except RuntimeError as e:
            if "destroyed actor" in str(e):
                # print(f"[AgentWrapper] Actor {self.actor.id} already destroyed. Skipping run_step.")
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                return control
            else:
                raise e

    def update_target_speed(self, speed, start_time=None):
        self._target_speed = speed
        self.agent._target_speed = speed
        if hasattr(self.agent, "get_local_planner"):
            self.agent.get_local_planner().set_speed(speed)
        print(f"[AgentWrapper] {self.actor.id} updated target speed to {speed}")

    def get_last_longitudinal_command(self):
        return self._target_speed  # could be expanded

    def reset(self):
        if hasattr(self.agent, 'reset'):
            self.agent.reset()

class LateralTeleport(py_trees.behaviour.Behaviour):
    def __init__(self, actor, offset=3.5, direction="left"):
        super(LateralTeleport, self).__init__(f"LateralNudge({direction})")
        self._actor = actor
        self._offset = -offset if direction == "left" else offset
        self._done = False

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS

        transform = self._actor.get_transform()
        location = transform.location

        # Calculate perpendicular direction to heading
        yaw_rad = math.radians(transform.rotation.yaw)
        dx = math.cos(yaw_rad + math.pi / 2.0)
        dy = math.sin(yaw_rad + math.pi / 2.0)

        location.x += self._offset * dx
        location.y += self._offset * dy

        new_transform = carla.Transform(location, transform.rotation)
        self._actor.set_transform(new_transform)

        self._done = True
        return py_trees.common.Status.SUCCESS

class SmoothLaneChange(py_trees.behaviour.Behaviour):
    def __init__(self, actor, offset=-3.5, direction="left", duration=2.0):
        super().__init__(f"AnimatedLaneChange({direction})")
        self._actor = actor
        self._offset = offset
        self._direction = direction
        self._duration = duration
        self._start_time = None
        self._initial_transform = None

    def initialise(self):
        self._start_time = time.time()
        self._initial_transform = self._actor.get_transform()

    def update(self):
        now = time.time()
        elapsed = now - self._start_time
        t = min(elapsed / self._duration, 1.0)

        if t >= 1.0:
            return py_trees.common.Status.SUCCESS

        initial_loc = self._initial_transform.location
        initial_rot = self._initial_transform.rotation

        lane_width = self._offset
        forward_distance = 3.0  # how far forward to move during lane change
        max_yaw_offset = 30  # degrees

        # Smooth step function
        def smoothstep(x):
            return 3*x**2 - 2*x**3

        s = smoothstep(t)

        # Compute lateral shift
        yaw_rad = math.radians(initial_rot.yaw)
        dx_lat = math.cos(yaw_rad + math.pi / 2.0)
        dy_lat = math.sin(yaw_rad + math.pi / 2.0)

        dx_fwd = math.cos(yaw_rad)
        dy_fwd = math.sin(yaw_rad)

        loc = carla.Location()
        loc.x = initial_loc.x + s * forward_distance * dx_fwd + s * lane_width * dx_lat
        loc.y = initial_loc.y + s * forward_distance * dy_fwd + s * lane_width * dy_lat
        loc.z = initial_loc.z

        # Rotate smoothly toward yaw offset and back
        yaw_offset = -max_yaw_offset * math.sin(math.pi * s)
        if self._direction == "right":
            yaw_offset *= -1  # right turn

        new_yaw = initial_rot.yaw + yaw_offset
        new_rot = carla.Rotation(pitch=initial_rot.pitch, yaw=new_yaw, roll=initial_rot.roll)

        self._actor.set_transform(carla.Transform(loc, new_rot))

        return py_trees.common.Status.RUNNING

class HardBrake(py_trees.behaviour.Behaviour):
    def __init__(self, actor):
        super().__init__(f"HardBrake({actor.id})")
        self._actor = actor
        self._done = False

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS
        control = carla.VehicleControl(throttle=0.0, brake=1.0)
        self._actor.apply_control(control)
        self._done = True
        print(f"[HardBrake] Actor {self._actor.id} applied full brake.")
        return py_trees.common.Status.SUCCESS

class custom_1(BasicScenario):
    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=1200):

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        self._velocity = 35
        self._delta_velocity = 10
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._other_actor_stop_in_front_intersection = 10

        point = config.trigger_points[0].location
        self._reference_waypoint = self._map.get_waypoint(point)

        # self._start_location = convert_dict_to_location(config.other_parameters['other_actor_location'])

        super(custom_1, self).__init__("custom_1",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)
    
    def _initialize_actors(self, config):
        waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        wp0 = self._reference_waypoint
        wp1 = wp0.next(30.0)[0]    # front car
        wp2 = wp0.next(20.0)[0]    # rear car (closer to ego)

        transform1 = wp1.transform
        transform2 = wp2.transform

        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform1)
        second_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform2)

        self.other_actors.append(first_vehicle)
        self.other_actors.append(second_vehicle)
    
    def _create_behavior(self):
        self.controllers = {}
        for actor in self.other_actors:
            agent = BasicAgent(actor, target_speed=5)
            wrapped = AgentControllerWrapper(agent, actor)
            self.controllers[actor.id] = wrapped
        blackboard = Blackboard()
        blackboard.set("ActorsWithController", self.controllers)

        # === PHASE 1: Drive normally ===
        drive_parallel = py_trees.composites.Parallel(
            "Drive Normally",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        drive_parallel.add_child(WaypointFollower(self.other_actors[0], 5))  # Front car
        drive_parallel.add_child(WaypointFollower(self.other_actors[1], 5))  # Second car
        drive_parallel.add_child(InTriggerDistanceToVehicle(self.ego_vehicles[0], self.other_actors[1], 15))  # Ego gets close

        # === PHASE 2: First vehicle brakes ===
        front_brake = HardBrake(self.other_actors[0])

        # === PHASE 3: Second vehicle slows + changes lane ===
        slow_down_second = ChangeActorTargetSpeed(self.other_actors[1], 3)
        change_lane = SmoothLaneChange(self.other_actors[1], offset=-3.5, direction="left", duration=2.5)

        lane_change_sequence = py_trees.composites.Sequence("Second Vehicle Reacts")
        lane_change_sequence.add_child(slow_down_second)
        lane_change_sequence.add_child(change_lane)

        # === END CONDITION: Ego drives close ===
        end_condition = InTriggerDistanceToVehicle(self.ego_vehicles[0], self.other_actors[0], 10)

        # === CLEANUP ===
        cleanup = py_trees.composites.Parallel("Cleanup", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        cleanup.add_child(ActorDestroy(self.other_actors[0]))
        cleanup.add_child(ActorDestroy(self.other_actors[1]))

        # === MASTER SEQUENCE ===
        scenario_sequence = py_trees.composites.Sequence("Scenario Sequence")
        if self.route_mode:
            scenario_sequence.add_child(LeaveSpaceInFront(200))
        scenario_sequence.add_child(drive_parallel)
        scenario_sequence.add_child(front_brake)
        scenario_sequence.add_child(lane_change_sequence)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(cleanup)

        return scenario_sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []
        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)
        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
