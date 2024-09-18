import random

import carla

import py_trees

import time
import sys

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower, LaneChange, WaitForever, \
    AtomicBehavior, BasicAgentBehavior, GetOverAgentBehavior
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.timer import TimeOut

from basic_scenario import BasicScenario  # Assuming the BasicScenario code is saved as basic_scenario.py
from srunner.tests.carla_mocks.agents.navigation.basic_agent import BasicAgent
from srunner.tests.carla_mocks.agents.navigation.local_planner import LocalPlanner
from srunner.tests.carla_mocks.agents.tools.misc import is_within_distance, distance_vehicle, get_speed

from srunner.tools.scenario_helper import generate_target_waypoint_list_multilane

SPAWN_LANE_Y_COORDINATES = [26.7, 30.2, 33.7, 37.3]
RIGHTMOST_LANE_ID = 6

NUM_LANES = 4 # excluding the shoulder/exit
NUM_X_SPAWN_POINTS = 4
MAX_X_SPAWN = -180
IN_BETWEEN_SPAWN_DISTANCE_X = 25
SPAWN_Z_COORDINATE = 6.0
EXIT_LOCATION = carla.Location(x=120, y=60.2)
CONTINUE_LOCATION = carla.Location(x=198.3, y=39.8)

MIN_SPEED = 12
MAX_SPEED = 22
MIN_LEADING_DIST = 10.0
SAFE_DISTANCE = 1.0

NUM_VEHICLES = 10

BLACKBOARD = set()
exiting_vehicles = []

def lane_safe_to_merge(actor, direction='right', num_waypoints = 2, step_dist = 2, dist_threshold=1.0):
    """
    Check if the lane is safe to merge into
    """
    world = actor.get_world()
    map = world.get_map()
    vehicle_list = world.get_actors().filter("*vehicle*")
    ego_transform = actor.get_transform()
    ego_location = ego_transform.location
    ego_wpt = map.get_waypoint(ego_location)
    adjacent_waypoints_to_check = []

    # Get the right offset
    if direction == 'left':
        adjacent_wpt = ego_wpt.get_left_lane()
    elif direction == 'right':
        adjacent_wpt = ego_wpt.get_right_lane()
    else:
        raise ValueError("Invalid direction. Choose 'left' or 'right'.")

    adjacent_lane_id = adjacent_wpt.lane_id
    # check lane change not allowed in given direction
    if not adjacent_wpt:
        return False, None

    for i in range(1, num_waypoints + 1):
        adjacent_waypoints_to_check.append(adjacent_wpt.previous(step_dist * i)[-1])
        adjacent_waypoints_to_check.append(adjacent_wpt)
        adjacent_waypoints_to_check.append(adjacent_wpt.next(step_dist * i)[-1])

    for target_vehicle in vehicle_list:
        if target_vehicle.id == actor.id:
            continue

        target_transform = target_vehicle.get_transform()
        target_wpt = map.get_waypoint(target_transform.location)

        if target_wpt.lane_id != adjacent_lane_id:
            continue

        for wp in adjacent_waypoints_to_check:
            if distance_vehicle(wp, target_transform) < dist_threshold:
                return False, target_vehicle.id

    return True, None

def set_random_speed(vehicle, tm):
    """
    Set a random velocity for the vehicle.
    """
    # make the random choice not be a normal distribution, instead a random distribution
    speed = random.randint(MIN_SPEED, MAX_SPEED)
    tm.set_desired_speed(vehicle, speed)

def get_vehicle_lane_id(vehicle):
    waypoint = vehicle.get_world().get_map().get_waypoint(vehicle.get_location())
    return waypoint.lane_id


class ThesisScenario(BasicScenario):

    def __init__(self, ego_vehicles, config, world,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=False):
        # Set the timeout for the scenario
        self.timeout = 120  # seconds
        super(ThesisScenario, self).__init__("ThesisScenario", ego_vehicles, config, world,
                                             debug_mode, terminate_on_failure, criteria_enable)


    def _initialize_actors(self, config):
        """
        Initialize the vehicles and set them to autopilot.
        """
        client = carla.Client()
        tm = client.get_trafficmanager()
        tm_port = tm.get_port()

        # destroy all vehicles that exist in the world already
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            vehicle.destroy()

        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = [
            blueprint_library.find('vehicle.tesla.model3'),
            blueprint_library.find('vehicle.volkswagen.t2'),
            blueprint_library.find('vehicle.audi.tt'),
            blueprint_library.find('vehicle.bmw.grandtourer'),
            blueprint_library.find('vehicle.chevrolet.impala'),
            blueprint_library.find('vehicle.mercedes.sprinter'),
            blueprint_library.find('vehicle.nissan.patrol'),
            blueprint_library.find('vehicle.toyota.prius'),
            blueprint_library.find('vehicle.seat.leon'),
            blueprint_library.find('vehicle.tesla.cybertruck')
        ]
        tesla_bp = blueprint_library.find('vehicle.tesla.model3')
        ambulance_bp = blueprint_library.find('vehicle.ford.ambulance')
        # create list of all potential spawn points
        potential_spawn_points = []
        for i in range(NUM_LANES):
            for j in range(NUM_X_SPAWN_POINTS):
                potential_spawn_points.append(carla.Transform(carla.Location(x=MAX_X_SPAWN - j * IN_BETWEEN_SPAWN_DISTANCE_X,
                                                                             y=SPAWN_LANE_Y_COORDINATES[i],
                                                                             z=SPAWN_Z_COORDINATE)))

        # randomly select NUM_VEHICLES spawn points
        spawn_points = random.sample(potential_spawn_points, NUM_VEHICLES)
        #spawn_points = [p for p in potential_spawn_points if p.location.y == LANE_Y_COORDINATES[3]]

        for i in range(NUM_VEHICLES):
            if i < NUM_VEHICLES / 3: # spawn 1 ambulance in every simulation
                vehicle = self.world.spawn_actor(ambulance_bp, spawn_points[i])
                exiting_vehicles.append(vehicle.id)
            else:
                # bp = random.choice(vehicle_blueprints)
                bp = tesla_bp
                vehicle = self.world.spawn_actor(bp, spawn_points[i])
                # vehicle = self.world.spawn_actor(tesla_blueprint, spawn_points[i])
            self.other_actors.append(vehicle)

            # vehicle.set_autopilot(True, tm_port)
            #             #
            #             # # traffic manager settings
            #             # tm.set_path(vehicle, [EXIT_LOCATION])
            #             # tm.update_vehicle_lights(vehicle, True)
            #             # tm.auto_lane_change(vehicle, False)
            #             # tm.distance_to_leading_vehicle(vehicle, MIN_LEADING_DIST)
            #             # tm.set_desired_speed(vehicle, 20.0)
        vehicle_ids = [vehicle.id for vehicle in self.other_actors]
        print("Spawned vehicles with ids: {}".format(vehicle_ids))


    def _create_behavior(self):
        """
        Define the behavior of the scenario.
        """
        tm = carla.Client().get_trafficmanager()
        root = py_trees.composites.Parallel("AutopilotScenarioBehavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        for vehicle in self.other_actors:
            vehicle_root = py_trees.composites.Parallel(f"Vehicle_{vehicle.id}_Root_Parallel",
                                                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
            vehicle_setup = py_trees.composites.Sequence(f"Vehicle_{vehicle.id}_Root_Sequence")
            # only set destination argument in SetupVehicle if it is ambulance, or 50% otherwise
            if vehicle.id == self.other_actors[0].id or vehicle.id in exiting_vehicles:
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=EXIT_LOCATION, speed=12.0, min_leading_distance=12.0))
                num_lanes_to_change = RIGHTMOST_LANE_ID - get_vehicle_lane_id(vehicle)
                for i in range(num_lanes_to_change):
                    lane_id_when_starting_change = get_vehicle_lane_id(vehicle) + i
                    vehicle_setup.add_child(
                        ForceTMLaneChange(vehicle, lane_id_when_starting_change, tm, exiting=True, direction='right'))
            else:
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=CONTINUE_LOCATION, speed=12.0, min_leading_distance=12.0))
                num_lanes_to_change = RIGHTMOST_LANE_ID - get_vehicle_lane_id(vehicle)
                for i in range(num_lanes_to_change):
                    lane_id_when_starting_change = get_vehicle_lane_id(vehicle) + i
                    vehicle_setup.add_child(ForceTMLaneChange(vehicle, lane_id_when_starting_change, tm, exiting=False))
            #vehicle_root.add_child(WaitForever())
            vehicle_root.add_children([vehicle_setup, SlowDownIfNeeded(vehicle, tm, original_speed=12.0, slow_speed=5.0, slow_duration=5.0)])
            root.add_child(vehicle_root)
        root.add_child(WaitForever())

        return root

    def _create_test_criteria(self):
        """
        Define the evaluation criteria for the scenario. Here we check for collisions.
        """
        criteria = []
        for vehicle in self.other_actors:
            collision_criterion = CollisionTest(vehicle)
            criteria.append(collision_criterion)
        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class ReachedDestination(AtomicBehavior):
    """
    Never returns success
    """
    def __init__(self, actor, destination, name="ReachedDestination"):
        super(ReachedDestination, self).__init__(name)
        self.actor = actor
        self.destination = destination

    def update(self):
        if self.actor.get_transform().location == self.destination:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ForceTMLaneChange(AtomicBehavior):
    """
    using tm to lane change right
    """
    def __init__(self, vehicle, start_lane_id, tm, exiting=False, direction='right', slow_duration=5.0, name="ForceTMLaneChange"):
        self.vehicle = vehicle
        self.start_lane_id = start_lane_id
        self.tm = tm
        self.exiting = exiting
        self.direction = direction
        self.slow_duration = slow_duration
        self.slow_down_time = None
        super(ForceTMLaneChange, self).__init__(name)

    def update(self):
        # if self.vehicle.id in BLACKBOARD:
        #     if self.slow_down_time is None:
        #         print("Vehicle {} says: I'M IN THE WAY! MY BAD!".format(self.vehicle.id))
        #         self.slow_down_time = time.time()
        #         self.tm.set_desired_speed(self.vehicle, 5.0)
        #         BLACKBOARD.remove(self.vehicle.id)
        #         #print("Vehicle {} has slowed to 5km/hr and the new blackboard is: {}".format(self.vehicle.id, BLACKBOARD))
        #
        #
        # if self.slow_down_time and time.time() - self.slow_down_time > self.slow_duration:
        #     self.tm.set_desired_speed(self.vehicle, 15.0)
        #     self.slow_down_time = None

        # if not in rightmost lane
        if self.exiting and get_vehicle_lane_id(self.vehicle) != RIGHTMOST_LANE_ID:
            if self.direction == 'right':
                #print("Vehicle {} is forcing lane change to the right, into {}".format(self.vehicle, get_vehicle_lane_id(self.vehicle) + 1))
                safe, blocking_vehicle_id = lane_safe_to_merge(self.vehicle, 'right', num_waypoints=6)
                if safe:
                    self.tm.force_lane_change(self.vehicle, True)
                else:
                    BLACKBOARD.add(blocking_vehicle_id)
                    #print("Vehicle {}: IT IS NOT SAFE TO MERGE RIGHT!".format(self.vehicle.id))
                    print("Blackboard is: {}".format(BLACKBOARD))
            elif self.direction == 'left' and not self.waiting:
                self.tm.force_lane_change(self.vehicle, False)
            if get_vehicle_lane_id(self.vehicle) != self.start_lane_id:
                #print("Vehicle {} successfully merged, SUCCESS!")
                #self.waiting = False
                return py_trees.common.Status.SUCCESS
            #print("Vehicle {} is still merging from lane {}".format(self.vehicle.id, get_vehicle_lane_id(self.vehicle)))
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING


class SlowDownIfNeeded(AtomicBehavior):
    def __init__(self, vehicle, tm, original_speed=15.0, slow_speed=7.0, slow_duration=5.0, name="SlowDownIfNeeded"):
        self.vehicle = vehicle
        self.tm = tm
        self.original_speed = original_speed
        self.slow_speed = slow_speed
        self.slow_duration = slow_duration
        self.slow_down_time = None
        self.slowed = False
        super(SlowDownIfNeeded, self).__init__(name)

    def update(self):
        if self.vehicle.id in BLACKBOARD:
            print("I, {} am in the blackboard!!".format(self.vehicle.id))
            if self.slow_down_time is None:
                print("Vehicle {} says: I'M IN THE WAY! MY BAD!".format(self.vehicle.id))
                self.slow_down_time = time.time()
                self.tm.set_desired_speed(self.vehicle, self.slow_speed)
                print("Vehicle {} has slowed to {}km/hr and the new blackboard is: {}".format(self.vehicle.id, self.slow_speed, BLACKBOARD))
                return py_trees.common.Status.RUNNING

            elapsed_time = time.time() - self.slow_down_time

            if elapsed_time > self.slow_duration:
                BLACKBOARD.remove(self.vehicle.id)
                self.tm.set_desired_speed(self.vehicle, self.original_speed)
                print("Vehicle {} has sped back up to original speed".format(self.vehicle.id))
                self.slow_down_time = None

        return py_trees.common.Status.RUNNING
class JustDrive(AtomicBehavior):
    """
    Use traffic manager to set up vehicle
    """
    def __init__(self, actor, target_speed, lane_change_chance=0, name="JustDrive"):
        self.actor = actor
        self.target_speed = target_speed
        self.lane_change_chance = lane_change_chance
        self.tm = carla.Client().get_trafficmanager()
        self.tm_port = self.tm.get_port()

        super(JustDrive, self).__init__(name, actor)

    def update(self):
        control = self.actor.get_control()
        control.speed = self.target_speed
        control.direction = self.actor.get_transform().rotation.get_forward_vector()
        self.actor.apply_control(control)
        # self.actor.set_autopilot(True, self.tm_port)
        # self.tm.set_desired_speed(self.actor, self.target_speed)
        # self.tm.auto_lane_change(self.actor, False)
        # self.tm.set_path(self.actor, [EXIT_LOCATION])
        return py_trees.common.Status.SUCCESS


class SetupVehicle(AtomicBehavior):
    """
    Using traffic manager
    """
    def __init__(self, vehicle, tm, destination=None, speed=20.0, min_leading_distance=5.0, name="SetupVehicle"):
        self.vehicle = vehicle
        self.destination = destination
        self.tm = tm
        self.tm_port = self.tm.get_port()
        self.speed = speed
        self.min_leading_dist = min_leading_distance

        super(SetupVehicle, self).__init__(name, vehicle)

    def update(self):
        self.vehicle.set_autopilot(True, self.tm_port)

        # traffic manager settings
        if self.destination:
            self.tm.set_path(self.vehicle, [self.destination])
        self.tm.update_vehicle_lights(self.vehicle, True)
        self.tm.auto_lane_change(self.vehicle, False)
        self.tm.distance_to_leading_vehicle(self.vehicle, self.min_leading_dist)
        self.tm.set_desired_speed(self.vehicle, self.speed)
        #set_random_speed(self.vehicle, self.tm)
        return py_trees.common.Status.SUCCESS


# class SetExitDestination(AtomicBehavior):
#     """
#     Using traffic manager
#     """
#     def __init__(self, actor, exit_location, tm, name="SetExitDestination"):
#         self.actor = actor
#         self.exit_location = exit_location
#         self.tm = tm
#         self.tm_port = self.tm.get_port()
#
#         super(SetExitDestination, self).__init__(name, actor)
#
#     def update(self):
#         # if actor makes it to the exit location, return success
#         if self.actor.get_transform().location == EXIT_LOCATION:
#             return py_trees.common.Status.SUCCESS
#         agent = BasicAgent(self.actor)
#         agent.set_destination(self.exit_location)
#         #self.actor.set_autopilot(True, self.tm_port)
#         #self.tm.set_path(self.actor, [self.exit_location])
#         return py_trees.common.Status.RUNNING

# class IsInRightmostLane(py_trees.behaviour.Behaviour):
#     def __init__(self, vehicle, name="IsInRightmostLane"):
#         super(IsInRightmostLane, self).__init__(name)
#         self.vehicle = vehicle
#
#     def update(self):
#         # Check if the vehicle is not in the rightmost lane
#         print("Vehicle {} is in lane {}".format(self.vehicle, get_vehicle_lane_id(self.vehicle)))
#         if get_vehicle_lane_id(self.vehicle) == RIGHTMOST_LANE_ID:
#             print("VEHICLE {} IS IN THE RIGHTMOST LANE!!".format(self.vehicle))
#             return py_trees.common.Status.SUCCESS
#         else:
#             #print("Vehicle {} is NOT in the rightmost lane".format(self.vehicle))
#             return py_trees.common.Status.RUNNING
#
# class NotInRightmostLane(py_trees.behaviour.Behaviour):
#     def __init__(self, vehicle, name="NotInRightmostLane"):
#         super(NotInRightmostLane, self).__init__(name)
#         self.vehicle = vehicle
#
#     def update(self):
#         # Check if the vehicle is not in the rightmost lane
#         # print("Vehicle {} is in lane {}".format(self.vehicle, get_vehicle_lane_id(self.vehicle)))
#         if get_vehicle_lane_id(self.vehicle) == RIGHTMOST_LANE_ID:
#             # print("VEHICLE {} IS IN THE RIGHTMOST LANE!!".format(self.vehicle))
#             return py_trees.common.Status.RUNNING
#         else:
#             #print("Vehicle {} is NOT in the rightmost lane".format(self.vehicle))
#             return py_trees.common.Status.SUCCESS
# class IsSafeToChangeLane(py_trees.behaviour.Behaviour):
#     def __init__(self, vehicle, direction='right', name="IsSafeToChangeLane"):
#         super(IsSafeToChangeLane, self).__init__(name)
#         self.vehicle = vehicle
#         self.direction = direction
#
#     def update(self):
#         # print("Checking if vehicle {} is safe to change lanes".format(self.vehicle))
#         lane_offset = 1 if self.direction == 'right' else -1
#         # Check if it is safe to change lanes
#         if not is_vehicle_in_lane(self.vehicle, lane_offset):
#             print("Vehicle {} is safe to change lanes".format(self.vehicle))
#             return py_trees.common.Status.SUCCESS
#         else:
#             print("Vehicle {} is not safe to change lanes".format(self.vehicle))
#             return py_trees.common.Status.RUNNING


# class DriveStraightAndCheckLaneChange(AtomicBehavior):
#     def __init__(self, vehicle, name="DriveStraightAndCheckLaneChange"):
#         super(DriveStraightAndCheckLaneChange, self).__init__(name)
#         self.vehicle = vehicle
#
#     def update(self):
#         pass


# Usage example
if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Define ego vehicles and config for the scenario
    ego_vehicles = []
    config = {}  # Define your config appropriately

    scenario = ThesisScenario("AutopilotScenario", ego_vehicles, config, world)
    scenario_tree = scenario.scenario_tree

    try:
        # Tick the scenario tree
        while not scenario_tree.status == py_trees.common.Status.SUCCESS:
            world.tick()
            scenario_tree.tick_once()
    finally:
        scenario.terminate()
        scenario.remove_all_actors()
