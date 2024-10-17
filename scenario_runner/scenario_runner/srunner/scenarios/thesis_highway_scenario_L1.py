import random
import carla
import py_trees
import time

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest

from basic_scenario import BasicScenario
from srunner.tests.carla_mocks.agents.tools.misc import distance_vehicle
from srunner.metrics.tools.write_thesis_results_to_csv import log_trial_results

SPAWN_LANE_Y_COORDINATES = [26.7, 30.2, 33.7, 37.3]
RIGHTMOST_LANE_ID = 6

NUM_LANES = 4 # excluding the shoulder/exit
NUM_X_SPAWN_POINTS = 4
MAX_X_SPAWN = -180
IN_BETWEEN_SPAWN_DISTANCE_X = 25
SPAWN_Z_COORDINATE = 6.0
EXIT_LOCATION = carla.Location(x=120, y=60.2)
EXIT_LOCATION_WITH_Z = carla.Location(x=120, y=60.2, z=10.0)

MIN_SPEED_OTHER = 10.0
MAX_SPEED_OTHER = 14.0
AMBULANCE_SPEED = 12.0
AMBULANCE_SLOW_SPEED = 9.0
MIN_LEADING_DIST = 25.0

NUM_VEHICLES = 11

BLACKBOARD = set()

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

def get_vehicle_lane_id(vehicle):
    waypoint = vehicle.get_world().get_map().get_waypoint(vehicle.get_location())
    return waypoint.lane_id


class ThesisScenarioL1(BasicScenario):

    def __init__(self, ego_vehicles, config, world,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=True):
        self.trial_number = config.trial_number
        if self.trial_number:
            random.seed(self.trial_number)
        # Set the timeout for the scenario
        self.timeout = 140  # seconds
        self.world = world
        self.start_time = None
        self.ambulance = None
        self.blocking_vehicles_encountered = set()
        self.first_ambulance_arrival_time = None
        self.ambulance_arrived = False
        self.total_collisions = 0
        self.ambulance_collisions = 0
        new_settings = world.get_settings()
        new_settings.fixed_delta_seconds = 0.033333
        self.world.apply_settings(new_settings)
        print("set world's fixed_delta_seconds to {}".format(self.world.get_settings().fixed_delta_seconds))
        print("framerate is {} fps.".format(1.0 / self.world.get_settings().fixed_delta_seconds))
        super(ThesisScenarioL1, self).__init__("ThesisScenarioL1", ego_vehicles, config, world,
                                               debug_mode, terminate_on_failure, criteria_enable)


    def _initialize_actors(self, config):
        """
        Initialize the vehicles and set them to autopilot.
        """
        # destroy all vehicles that exist in the world already
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            vehicle.destroy()

        blueprint_library = self.world.get_blueprint_library()
        tesla_bp = blueprint_library.find('vehicle.tesla.model3')
        ambulance_bp = blueprint_library.find('vehicle.ford.ambulance')

        # create list of all potential spawn points
        potential_spawn_points = []
        for i in range(NUM_LANES):
            for j in range(NUM_X_SPAWN_POINTS):
                potential_spawn_points.append(carla.Transform(carla.Location(x=MAX_X_SPAWN - j * IN_BETWEEN_SPAWN_DISTANCE_X,
                                                                             y=SPAWN_LANE_Y_COORDINATES[i],
                                                                             z=SPAWN_Z_COORDINATE)))

        # Ambulance will always spawn at the second spot in the leftmost lane
        ambulance_spawn_point = potential_spawn_points[1]
        potential_spawn_points.remove(ambulance_spawn_point)
        # randomly select NUM_VEHICLES spawn points
        other_spawn_points = random.sample(potential_spawn_points, NUM_VEHICLES - 1) # excluding ambulance

        # spawn ambulance first
        vehicle = self.world.spawn_actor(ambulance_bp, ambulance_spawn_point)
        self.ambulance = vehicle
        self.other_actors.append(vehicle)
        for i in range(NUM_VEHICLES - 1):
            bp = tesla_bp
            vehicle = self.world.spawn_actor(bp, other_spawn_points[i])
            self.other_actors.append(vehicle)

        vehicle_ids = [vehicle.id for vehicle in self.other_actors]
        print("Spawned vehicles with ids: {}".format(vehicle_ids))


    def _create_behavior(self):
        """
        Define the behavior of the scenario.
        """
        tm = carla.Client().get_trafficmanager()
        tm.set_synchronous_mode(True)
        root = py_trees.composites.Parallel("AutopilotScenarioBehavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        self.start_time = time.time()

        for vehicle in self.other_actors:
            vehicle_root = py_trees.composites.Parallel(f"Vehicle_{vehicle.id}_Root_Parallel",
                                                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
            vehicle_setup = py_trees.composites.Sequence(f"Vehicle_{vehicle.id}_Root_Sequence")

            # only set destination argument in SetupVehicle if it is ambulance
            if vehicle.id == self.ambulance.id:
                tm.set_desired_speed(vehicle, AMBULANCE_SPEED)
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=EXIT_LOCATION, speed=AMBULANCE_SPEED,
                                                     min_leading_distance=MIN_LEADING_DIST))
                num_lanes_to_change = RIGHTMOST_LANE_ID - get_vehicle_lane_id(vehicle)
                for i in range(num_lanes_to_change):
                    lane_id_when_starting_change = get_vehicle_lane_id(vehicle) + i
                    vehicle_setup.add_child(
                        L1ForceTMLaneChange(vehicle, lane_id_when_starting_change, tm, self,
                                            original_speed=AMBULANCE_SPEED, slow_speed=AMBULANCE_SLOW_SPEED,
                                            exiting=True))
                vehicle_setup.add_child(CheckAmbulanceArrival(vehicle, EXIT_LOCATION_WITH_Z, self.start_time, self))
            else:
                random_speed = random.uniform(MIN_SPEED_OTHER, MAX_SPEED_OTHER)
                tm.set_desired_speed(vehicle, random_speed)
                # print("Initializing vehicle {} with random speed of {}".format(vehicle.id, random_speed))
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=None, speed=random_speed,
                                                     min_leading_distance=MIN_LEADING_DIST))
            vehicle_root.add_child(vehicle_setup)
            root.add_child(vehicle_root)

        return root

    def _create_test_criteria(self):
        """
        Define the evaluation criteria for the scenario. Here we check for collisions.
        """
        criteria_tree = py_trees.composites.Parallel(name="Collision Criteria", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        for vehicle in self.other_actors:
            collision_criterion = CollisionTest(vehicle)
            criteria_tree.add_child(collision_criterion)
        return criteria_tree

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    def terminate(self):
        """
        Called at the end of the simulation to print collision statistics.
        """
        total_collisions = 0
        for criteria in self.criteria_tree.children:
            #print(f"Vehicle {criteria.actor.id} collisions: {criteria.actual_value}")
            total_collisions += criteria.actual_value
            if criteria.actor.id == self.ambulance.id:
                self.ambulance_collisions += criteria.actual_value
                #print(f"Ambulance collisions: {criteria.actual_value}")

        #print(f"Total collisions: {total_collisions/2.0}")
        self.total_collisions = total_collisions/2.0
        # log_trial_results(1, self.trial_number, NUM_VEHICLES, self.first_ambulance_arrival_time,
        #                   self.ambulance_arrived, self.total_collisions, self.ambulance_collisions,
        #                   blocking_vehicles_encountered=len(self.blocking_vehicles_encountered),
        #                   csv_filename="thesis_L1_11_simulation_results30fps.csv"
        #                   )
        super().remove_all_actors()
        super().terminate()

    def set_ambulance_time(self, time_taken):
        if self.first_ambulance_arrival_time is None:
            self.first_ambulance_arrival_time = time_taken
            self.ambulance_arrived = True
            self.terminate()

class L1ForceTMLaneChange(AtomicBehavior):
    """
    using tm to lane change right
    """
    def __init__(self, vehicle, start_lane_id, tm, scenario, exiting=False, direction='right', slow_duration=0.5,
                 original_speed=12.0, slow_speed=10.0, name="L1ForceTMLaneChange"):
        self.vehicle = vehicle
        self.start_lane_id = start_lane_id
        self.tm = tm
        self.scenario = scenario
        self.exiting = exiting
        self.direction = direction
        self.slow_duration = slow_duration
        self.original_speed = original_speed
        self.slow_speed = slow_speed
        self.slow_down_time = None
        super(L1ForceTMLaneChange, self).__init__(name)

    def update(self):
        # if not in rightmost lane
        #print(f"Vehicle {self.vehicle.id} has current velocity of: {get_speed(self.vehicle)} km/hr")
        if self.exiting and get_vehicle_lane_id(self.vehicle) != RIGHTMOST_LANE_ID:
            if self.direction == 'right':
                #print("Vehicle {} is forcing lane change to the right, into {}".format(self.vehicle, get_vehicle_lane_id(self.vehicle) + 1))
                safe, blocking_vehicle_id = lane_safe_to_merge(self.vehicle, 'right', num_waypoints=3)
                if safe:
                    self.tm.force_lane_change(self.vehicle, True)
                else:
                    if self.slow_down_time is None:
                        self.scenario.blocking_vehicles_encountered.add(blocking_vehicle_id)
                        self.slow_down_time = time.time()
                        self.tm.set_desired_speed(self.vehicle, self.slow_speed)
                        # print("Vehicle {} has slowed to {}km/hr and the new blackboard is: {}".format(self.vehicle.id, self.slow_speed, BLACKBOARD))
                        return py_trees.common.Status.RUNNING

                    elapsed_time = time.time() - self.slow_down_time

                    if elapsed_time > self.slow_duration:
                        #BLACKBOARD.remove(self.vehicle.id)
                        self.tm.set_desired_speed(self.vehicle, self.original_speed)
                        #print("Vehicle {} has sped back up to original speed".format(self.vehicle.id))
                        self.slow_down_time = None
            if get_vehicle_lane_id(self.vehicle) != self.start_lane_id:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING


class CheckAmbulanceArrival(py_trees.behaviour.Behaviour):
    """
    Check if the vehicle has arrived at the destination.
    """
    def __init__(self, vehicle, destination, start_time, scenario, name="CheckArrival"):
        super(CheckAmbulanceArrival, self).__init__(name)
        self.vehicle = vehicle
        self.destination = destination
        self.start_time = start_time
        self.scenario = scenario

    def update(self):
        # Check if the vehicle has arrived at the destination
        if self.vehicle_has_arrived():
            duration = time.time() - self.start_time
            self.scenario.set_ambulance_time(duration)
            # print(f"Ambulance arrived! Duration: {duration} seconds")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def vehicle_has_arrived(self):
        # Example logic to check if the vehicle is within a certain distance of the destination
        vehicle_location = self.vehicle.get_location()
        distance = vehicle_location.distance(self.destination)
        if distance < 1.0:  # Arrival threshold
            return True
        return False


class SetupVehicle(AtomicBehavior):
    """
    Using traffic manager
    """
    def __init__(self, vehicle, tm, destination=None, speed=12.0, min_leading_distance=25.0, name="SetupVehicle"):
        self.vehicle = vehicle
        self.destination = destination
        self.tm = tm
        self.tm_port = self.tm.get_port()
        self.speed = speed
        self.min_leading_dist = min_leading_distance

        super(SetupVehicle, self).__init__(name, vehicle)

    def update(self):
        self.vehicle.set_autopilot(True, self.tm_port)

        if self.destination:
            self.tm.set_path(self.vehicle, [self.destination])
        self.tm.update_vehicle_lights(self.vehicle, True)
        self.tm.auto_lane_change(self.vehicle, False)
        self.tm.distance_to_leading_vehicle(self.vehicle, self.min_leading_dist)
        return py_trees.common.Status.SUCCESS

