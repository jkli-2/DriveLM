#!/usr/bin/env python3
"""
ChainReactionOvertake scenario

Description
-----------
Two vehicles ahead of the ego (A1 lead, A2 middle) cruise at constant speed. 
A1 brakes. A2 first brakes to avoid rear-ending A1, then performs a lane change 
to overtake A1, and (optionally) merges back. The ego under test must react to
avoid collision and proceed safely.

This file is designed to be drop-in compatible with ScenarioRunner-style setups
and mirrors patterns from built-in obstacle scenarios (e.g., Accident, ParkedObstacle).

Notes
-----
- The exact atomic behavior signatures can differ across SR versions. Where this
  file uses LaneChange/BasicAgentBehavior/WaypointFollower/etc., adjust imports
  or parameters to your local ScenarioRunner version.
- Route-mode helpers (LeaveSpaceInFront/SetMaxSpeed) are optional and only act
  when running in route mode.

"""
from __future__ import annotations

import py_trees
import carla
import logging

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    ScenarioTimeoutTest,
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToLocation,
    InTriggerDistanceToVehicle,
    WaitUntilInFront,
    WaitUntilInFrontPosition,
    DriveDistance,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorDestroy,
    ScenarioTimeout,
    Idle,
    WaitForever,
    HandBrakeVehicle,
    WaypointFollower,
    BasicAgentBehavior,
    LaneChange,
)
from srunner.tools.background_manager import (
    LeaveSpaceInFront,
    SetMaxSpeed,
    ChangeOppositeBehavior,
    ChangeRoadBehavior,
)

# -----------------------------------------------------------------------------
# Parameter getters
# -----------------------------------------------------------------------------

def gv(config, name, p_type, default):
    """
    Get value parameter
    """
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default

def gi(config, name, p_type, default):
    """
    Get interval parameter
    """
    if name in config.other_parameters:
        return [
            p_type(config.other_parameters[name]['from']),
            p_type(config.other_parameters[name]['to'])
        ]
    else:
        return default

def kmh_to_ms(self, v_kmh: float) -> float:
    return v_kmh / 3.6

def _vec_speed_ms(actor) -> float:
    try:
        v = actor.get_velocity()
        return (v.x*v.x + v.y*v.y + v.z*v.z) ** 0.5
    except Exception:
        return -1.0

def _wp_info(self, wp: carla.Waypoint) -> str:
    lane = getattr(wp, "lane_id", None)
    ltype = getattr(wp, "lane_type", None)
    sec  = getattr(wp, "road_id", None)
    return f"road={sec} lane_id={lane} lane_type={ltype} @ ({wp.transform.location.x:.1f},{wp.transform.location.y:.1f})"

def _lane_adjacent(self, wp: carla.Waypoint):
    left = wp.get_left_lane()
    right = wp.get_right_lane()
    ok_l = left and left.lane_type == carla.LaneType.Driving
    ok_r = right and right.lane_type == carla.LaneType.Driving
    return ok_l, left, ok_r, right

# -----------------------------------------------------------------------------
# Scenario
# -----------------------------------------------------------------------------

class ChainReactionOvertake(BasicScenario):
    """
    Multi-vehicle chain reaction:
      - A1 (lead) brakes after trigger.
      - A2 (middle) initially brakes, then performs a lane change to overtake.
      - Optionally A2 merges back.

    The ego must react (brake and/or change lane) to avoid collision and continue.
    """

    def __init__(self,
                 world,
                 ego_vehicles,
                 config,
                 randomize: bool = False,
                 debug_mode: bool = False,
                 criteria_enable: bool = True,
                 timeout: float = 240):
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout

        # Distances (waypoint-forward) used to place A2, A1 from the trigger point (metres)
        self._a2_distance = gv(config, 'a2_distance', float, 20.0)    # distance ahead of trigger
        self._a1_gap = gv(config, 'a1_gap', float, 20.0)              # gap between A2 and A1
        self._end_distance = gv(config, 'end_distance', float, 60.0)

        # Trigger
        self._trigger_distance = gv(config, 'trigger_distance', float, 15.0)
        self._brake_delay = gv(config, 'brake_delay', float, 0.8)     # seconds after trigger

        # Directions / lane change
        self._lc_direction = gv(config, 'lc_direction', str, 'left')  # 'left' or 'right'
        if self._lc_direction not in ('left', 'right'):
            raise ValueError(f"lc_direction must be 'left' or 'right', got {self._lc_direction}")
        self._lc_distance = gv(config, 'lc_distance', float, 25.0)    # forward distance during LC
        self._merge_back = bool(gv(config, 'merge_back', int, 1))
        self._merge_back_after = gv(config, 'merge_back_after', float, 20.0)  # distance after pass

        # Speeds (km/h)
        self._a1_speed = gv(config, 'a1_speed', float, 10.0)
        self._a2_speed = gv(config, 'a2_speed', float, 15.0)
        self._a1_brake_speed = gv(config, 'a1_brake_speed', float, 2.0)  # target after braking
        self._a2_brake_speed = gv(config, 'a2_brake_speed', float, 7.0)
        self._a2_overtake_speed = gv(config, 'a2_overtake_speed', float, 20.0)

        # Braking style for A1
        self._a1_hard_brake = bool(gv(config, 'a1_hard_brake', int, 0))
        self._a1_hard_brake_time = gv(config, 'a1_hard_brake_time', float, 1.0)

        # Scenario timeout
        self._scenario_timeout = gv(config, 'scenario_timeout', float, 240.0)

        # Corridor reservation for route-mode (keep background traffic out)
        self._extra_space = gv(config, 'extra_space', float, 60.0)

        self.a1 = None
        self.a2 = None
        self._start_wp = None
        self._a2_wp = None
        self._a1_wp = None
        self._end_wp = None

        self._log = logging.getLogger("ChainReactionOvertake")
        if debug_mode:
            # Keep it local to this scenario; don’t globally reconfigure logging
            self._log.setLevel(logging.DEBUG)
            if not self._log.handlers:
                _h = logging.StreamHandler()
                _h.setFormatter(logging.Formatter("[%(levelname)s] %(asctime)s %(name)s: %(message)s"))
                self._log.addHandler(_h)
        else:
            self._log.setLevel(logging.WARNING)
        super().__init__("ChainReactionOvertake",
                         ego_vehicles,
                         config,
                         world,
                         randomize,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _move_waypoint_forward(self, wp: carla.Waypoint, distance: float) -> carla.Waypoint:
        dist = 0.0
        next_wp = wp
        step = 1.0
        while dist < distance:
            next_wps = next_wp.next(step)
            if not next_wps or next_wps[0].is_junction:
                break
            next_wp = next_wps[0]
            dist += step
        return next_wp

    def _spawn_in_lane(self, wp, blueprint='vehicle.*', rolename: str = 'scenario'):
        """
        Spawn at the lane centre (no lateral offset). Keep it simple so the lane is clearly blocked.
        """
        world = self._world
        spawn_transform = wp.transform
        spawn_transform.location.z += 1.0
        self._log.debug(f"Spawning '{rolename}' at {_wp_info(self, wp)} using {blueprint}")
        actor = CarlaDataProvider.request_new_actor(blueprint, spawn_transform, rolename=rolename)
        if not actor:
            self._log.error(f"Spawn failed for '{rolename}' at {_wp_info(self, wp)}")
            raise ValueError("Couldn't spawn the vehicle in lane")
        bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        actor_controller = world.try_spawn_actor(bp, spawn_transform)
        if actor_controller is None:
            print(f"Failed at {spawn_transform.location} (likely overlap).")
            self._log.warning(f"Secondary spawn (controller) failed at {spawn_transform.location} (likely overlap). "
                              f"This is usually fine; primary actor id={getattr(actor,'id',None)}")
        return actor

    def _spawn_on_lane_center(self, wp: carla.Waypoint, blueprint: str = 'vehicle.*', rolename: str = 'scenario'):
        """Robust spawner that searches nearby lanes, advances along the road,
        applies lateral nudges, and tries higher Z to avoid collisions with map mesh.
        Returns the spawned actor or raises.
        """
        world = self._world
        candidates = [
            'vehicle.tesla.model3',
            'vehicle.audi.tt',
            'vehicle.lincoln.mkz_2017',
            'vehicle.nissan.patrol',
            'vehicle.mini.cooper_s'
        ]

        forward_scan = 30.0  # meters to scan forward
        step = 1.0
        z_offsets = [0.5, 0.8, 1.0, 1.2]  # modest lifts (too high can look invisible from ego)
        lateral_scales = [0.0, 0.2, -0.2, 0.35, -0.35]  # fractions of lane_width

        def _try_spawn_at_waypoint(test_wp: carla.Waypoint):
            lane_w = max(2.5, float(test_wp.lane_width))
            base_tf = test_wp.transform
            right_vec = base_tf.get_right_vector()
            for lat_k in lateral_scales:
                for z_up in z_offsets:
                    tf = carla.Transform(base_tf.location, base_tf.rotation)
                    tf.location += carla.Location(x=right_vec.x * lane_w * lat_k,
                                                  y=right_vec.y * lane_w * lat_k,
                                                  z=z_up)
                    for bp in candidates:
                        actor = CarlaDataProvider.request_new_actor(
                            bp, tf, rolename='scenario')
                        if actor:
                            return actor
            return None

        # Ensure actors are on a valid driving lane, not junctions
        cur = wp
        while cur and (cur.is_junction or cur.lane_type != carla.LaneType.Driving):
            nxt = cur.next(step)
            if not nxt:
                break
            cur = nxt[0]
        if not cur:
            raise ValueError('No valid driving waypoint ahead to spawn vehicle')

        # Lane candidates: current and immediate left/right lane
        lane_candidates = [cur]
        if cur.get_left_lane() and cur.get_left_lane().lane_type == carla.LaneType.Driving:
            lane_candidates.append(cur.get_left_lane())
        if cur.get_right_lane() and cur.get_right_lane().lane_type == carla.LaneType.Driving:
            lane_candidates.append(cur.get_right_lane())

        # Scan forward along each lane candidate
        for lane_wp in lane_candidates:
            advanced = 0.0
            test_wp = lane_wp
            while advanced <= forward_scan:
                if not test_wp.is_junction:
                    actor = _try_spawn_at_waypoint(test_wp)
                    if actor:
                        return actor
                nxt = test_wp.next(step)
                if not nxt:
                    break
                test_wp = nxt[0]
                advanced += step

        raise ValueError("Couldn't spawn vehicle actor after exhaustive nearby search")

    # ------------------------------ lifecycle --------------------------------
    def _initialize_actors(self, config):
        self._start_wp = self._map.get_waypoint(config.trigger_points[0].location)
        self._log.debug(f"Trigger start at {_wp_info(self, self._start_wp)}")
        # Nudge start forward if trigger is too close to junction/non-driving lane
        # if self._start_wp.is_junction or self._start_wp.lane_type != carla.LaneType.Driving:
        #     self._start_wp = self._move_waypoint_forward(self._start_wp, 5.0)

        self._a2_wp = self._move_waypoint_forward(self._start_wp, self._a2_distance)
        self._a1_wp = self._move_waypoint_forward(self._a2_wp, self._a1_gap)
        self._end_wp = self._move_waypoint_forward(self._a1_wp, self._end_distance)
        self._log.debug(f"A2 wp: {_wp_info(self, self._a2_wp)} | A1 wp: {_wp_info(self, self._a1_wp)} | END wp: {_wp_info(self, self._end_wp)}")

        ok_l_a2, left_a2, ok_r_a2, right_a2 = _lane_adjacent(self, self._a2_wp)
        ok_l_a1, left_a1, ok_r_a1, right_a1 = _lane_adjacent(self, self._a1_wp)
        self._log.debug(f"A2 adjacents -> left={ok_l_a2} right={ok_r_a2}")
        self._log.debug(f"A1 adjacents -> left={ok_l_a1} right={ok_r_a1}")

        self.a2 = self._spawn_in_lane(self._a2_wp, 'vehicle.*', rolename='middle')
        self.a1 = self._spawn_in_lane(self._a1_wp, 'vehicle.*', rolename='lead')

        self.a1.apply_control(carla.VehicleControl(hand_brake=False))
        self.a2.apply_control(carla.VehicleControl(hand_brake=False))
        self._log.debug(f"Spawned A1 id={self.a1.id} A2 id={self.a2.id}; speeds km/h: "
                        f"A1={self._a1_speed} A2={self._a2_speed}; LC dir='{self._lc_direction}' "
                        f"merge_back={self._merge_back} after={self._merge_back_after} m")
        self.other_actors.append(self.a1)
        self.other_actors.append(self.a2)

        # register for expert planner (autopilot) visibility.
        # lane change wont work if not registered.
        # unpatched autopilot also doesnt recognize custom scenario types.
        active_scenario_name = type(self).__name__
        try:
            now_ts = self._world.get_snapshot().timestamp.elapsed_seconds
        except Exception:
            now_ts = 0.0

        scenario_data = [
            self.a1,                   # 0 SD_A1
            self.a2,                   # 1 SD_A2
            False,                     # 2 SD_CHANGED
            -1,                        # 3 SD_FROM_IDX
            -1,                        # 4 SD_TO_IDX
            False,                     # 5 SD_PATH_CLEAR
            True,                      # 6 SD_SHIFT_LEFT (default; will be set from adjacency)
            None,                      # 7 SD_A1_LANE0
            None,                      # 8 SD_A2_LANE0
            None,                      # 9 SD_RESERVED_0
            None,                      # 10 SD_RESERVED_1
        ]
        CarlaDataProvider.active_scenarios.append(
            ("ChainReactionOvertake", scenario_data)
        )
        self._log.debug("Registered active_scenario entry; autopilot will see the actors.")

    def _create_behavior(self):
        root = py_trees.composites.Sequence(name="ChainReactionOvertakeRoot")

        if self.route_mode:
            total_dist = self._trigger_distance + self._a2_distance + self._a1_gap + self._end_distance + 100.0
            root.add_child(LeaveSpaceInFront(total_dist))
            root.add_child(ChangeRoadBehavior(extra_space=self._extra_space))

        # Main returns SUCCESS only when end_condition (timeout OR ego reached END_WP) succeeds.
        main = py_trees.composites.Parallel(name="MainParallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        main.add_child(ScenarioTimeout(self._scenario_timeout, self.config.name))
        main.add_child(WaitUntilInFrontPosition(self.ego_vehicles[0], self._end_wp.transform, False))

        # --- A1: cruise, then brake after trigger ---
        a1_seq = py_trees.composites.Sequence(name="A1_Cruise")
        a1_seq.add_child(WaypointFollower(self.a1, target_speed=self._a1_speed))

        # Brake
        a1_brake = py_trees.composites.Sequence(name="A1_Brake")
        a1_brake.add_child(InTriggerDistanceToVehicle(self.ego_vehicles[0], self.a2, self._trigger_distance))
        a1_brake.add_child(Idle(self._brake_delay))
        if self._a1_hard_brake:
            a1_brake.add_child(HandBrakeVehicle(self.a1, self._a1_hard_brake_time))
        else:
            a1_brake.add_child(WaypointFollower(self.a1, target_speed=self._a1_brake_speed))
        a1_brake.add_child(WaitForever())

        a1_parallel = py_trees.composites.Parallel(name="A1", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        a1_parallel.add_child(a1_seq)
        a1_parallel.add_child(a1_brake)
        main.add_child(a1_parallel)

        # --- A2: cruise, then brake → lane change → overtake → (opt) merge back ---
        a2_cruise = py_trees.composites.Sequence(name="A2_Cruise")
        a2_cruise.add_child(WaypointFollower(self.a2, target_speed=self._a2_speed))
        a2_cruise.add_child(WaitForever())

        # Brake and react
        a2_react = py_trees.composites.Sequence(name="A2_React")
        a2_react.add_child(InTriggerDistanceToVehicle(self.a1, self.a2, self._trigger_distance))
        a2_react.add_child(WaypointFollower(self.a2, target_speed=self._a2_brake_speed))
        a2_react.add_child(Idle(0.3))
        a2_react.add_child(LaneChange(self.a2, direction=self._lc_direction, distance_lane_change=self._lc_distance, speed=self._a2_overtake_speed))
        a2_react.add_child(WaypointFollower(self.a2, target_speed=self._a2_overtake_speed))
        # ensure pass is complete before optional merge back
        a2_react.add_child(WaitUntilInFront(self.a2, self.a1, check_distance=True))
        a2_react.add_child(DriveDistance(self.a2, self._merge_back_after))

        # Merge back
        if self._merge_back:
            opposite = 'left' if self._lc_direction == 'right' else 'right'
            a2_react.add_child(LaneChange(self.a2, direction=opposite, distance_lane_change=self._lc_distance, speed=self._a2_speed))
            a2_react.add_child(WaypointFollower(self.a2, target_speed=self._a2_speed))

        a2_react.add_child(WaitForever())

        a2_parallel = py_trees.composites.Parallel(name="A2", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        a2_parallel.add_child(a2_cruise)
        a2_parallel.add_child(a2_react)
        main.add_child(a2_parallel)

        # --- Ego-centric arming for completeness (kept simple) ---
        # If you want ego speed control near the scene in route mode, you can add SetMaxSpeed here.
        # ego_arm = py_trees.composites.Sequence(name="EgoArm")
        # ego_arm.add_child(InTriggerDistanceToLocation(self.ego_vehicles[0], self._a2_wp.transform.location, self._trigger_distance))
        # ego_arm.add_child(Idle(0.01))
        # ego_arm.add_child(WaitForever())
        # main.add_child(ego_arm)

        root.add_child(main)

        # Route-mode cleanup or speed reset
        if self.route_mode:
            root.add_child(SetMaxSpeed(0))

        return root

    def _create_test_criteria(self):
        criteria = [ScenarioTimeoutTest(self.ego_vehicles[0], self.config.name)]
        if not self.route_mode:
            criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        try:
            for a in list(self.other_actors):
                if a.is_alive:
                    a.destroy()
        except Exception:
            pass
        self.remove_all_actors()
