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
AMBULANCE_SLOW_DURATION = 0.5
OTHER_SLOW_SPEED = 9.0
OTHER_SLOW_DURATION = 5.0

NUM_VEHICLES = 16
BENEVOLENT_PROPORTION = 0.5

BLACKBOARD = set()
DECISIONS = {}

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


class ThesisScenarioL2(BasicScenario):

    def __init__(self, ego_vehicles, config, world,
                 debug_mode=False, terminate_on_failure=False, criteria_enable=True):
        self.trial_number = config.trial_number
        if self.trial_number:
            random.seed(self.trial_number)
        # Set the timeout for the scenario
        self.timeout = 140  # seconds
        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()
        self.start_time = None
        self.ambulance = None
        self.blocking_vehicles_encountered = set()
        self.first_ambulance_arrival_time = None
        self.ambulance_arrived = False
        self.total_collisions = 0
        self.ambulance_collisions = 0
        self.benevolent_vehicles = {}
        self.accepted_slowdown_ids = set()
        self.rejected_slowdown_ids = set()
        self.benevolent_vehicle_count = 0
        self.selfish_vehicle_count = 0
        self.num_accepted_slowdowns = 0
        self.num_rejected_slowdowns = 0
        new_settings = world.get_settings()
        new_settings.fixed_delta_seconds = 0.033333
        self.world.apply_settings(new_settings)
        print("set world's fixed_delta_seconds to {}".format(self.world.get_settings().fixed_delta_seconds))
        print("frame rate is {} fps.".format(1.0 / self.world.get_settings().fixed_delta_seconds))
        super(ThesisScenarioL2, self).__init__("ThesisScenarioL2", ego_vehicles, config, world,
                                               debug_mode, terminate_on_failure, criteria_enable)


    def _initialize_actors(self, config):
        # destroy all vehicles that exist in the world already
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            vehicle.destroy()

        blueprint_library = self.world.get_blueprint_library()
        tesla_bp = blueprint_library.find('vehicle.tesla.model3')
        prius_bp = blueprint_library.find('vehicle.toyota.prius')
        ambulance_bp = blueprint_library.find('vehicle.ford.ambulance')

        # create list of all 16 potential spawn points
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
        self.benevolent_vehicles[vehicle.id] = False

        # choose 20% of the vehicles to be benevolent
        benevolent_count = int((NUM_VEHICLES-1) * BENEVOLENT_PROPORTION)
        benevolent_spawns = random.sample(range(NUM_VEHICLES-1), benevolent_count)

        for i in range(NUM_VEHICLES - 1):
            if i in benevolent_spawns:
                self.benevolent_vehicle_count += 1
                bp = prius_bp
            else:
                self.selfish_vehicle_count += 1
                bp = tesla_bp

            vehicle = self.world.spawn_actor(bp, other_spawn_points[i])
            self.other_actors.append(vehicle)

            if i in benevolent_spawns:
                self.benevolent_vehicles[vehicle.id] = True
            else:
                self.benevolent_vehicles[vehicle.id] = False

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
                desired_speed = AMBULANCE_SPEED
                tm.set_desired_speed(vehicle, desired_speed)
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=EXIT_LOCATION, speed=desired_speed,
                                                     min_leading_distance=MIN_LEADING_DIST))
                num_lanes_to_change = RIGHTMOST_LANE_ID - get_vehicle_lane_id(vehicle)
                for i in range(num_lanes_to_change):
                    lane_id_when_starting_change = get_vehicle_lane_id(vehicle) + i
                    vehicle_setup.add_child(
                        ForceTMLaneChange(vehicle, lane_id_when_starting_change, tm, self, exiting=True,
                                          direction='right', slow_speed=AMBULANCE_SLOW_SPEED,
                                          slow_duration=AMBULANCE_SLOW_DURATION))
                vehicle_setup.add_child(CheckAmbulanceArrival(vehicle, EXIT_LOCATION_WITH_Z, self.start_time, self))
            else:
                desired_speed = random.uniform(MIN_SPEED_OTHER, MAX_SPEED_OTHER)
                tm.set_desired_speed(vehicle, desired_speed)
                # print("Initializing vehicle {} with random speed of {}".format(vehicle.id, desired_speed))
                vehicle_setup.add_child(SetupVehicle(vehicle, tm, destination=None, speed=desired_speed,
                                                     min_leading_distance=MIN_LEADING_DIST))

            is_benevolent = self.benevolent_vehicles[vehicle.id]
            vehicle_root.add_children([vehicle_setup, SlowDownIfNeeded(vehicle, tm, self,
                                                                       benevolent=is_benevolent,
                                                                       original_speed=desired_speed,
                                                                       slow_speed=OTHER_SLOW_SPEED,
                                                                       slow_duration=OTHER_SLOW_DURATION)])
            root.add_child(vehicle_root)

        return root

    def _create_test_criteria(self):
        """
        Define the evaluation criteria for the scenario. Here we check for collisions.
        """
        criteria_tree = py_trees.composites.Parallel(name="Collision Criteria",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
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
            # print(f"Vehicle {criteria.actor.id} collisions: {criteria.actual_value}")
            total_collisions += criteria.actual_value
            if criteria.actor.id == self.ambulance.id:
                self.ambulance_collisions += criteria.actual_value
                # print(f"Ambulance collisions: {criteria.actual_value}")

        self.total_collisions = total_collisions/2.0
        self.num_accepted_slowdowns = len(self.accepted_slowdown_ids)
        self.num_rejected_slowdowns = len(self.rejected_slowdown_ids)
        # log_trial_results(2, self.trial_number, NUM_VEHICLES, self.first_ambulance_arrival_time,
        #                   self.ambulance_arrived,
        #                   self.total_collisions, self.ambulance_collisions,
        #                   blocking_vehicles_encountered=len(self.blocking_vehicles_encountered),
        #                   benevolent_count=self.benevolent_vehicle_count,
        #                   selfish_count=self.selfish_vehicle_count, blocking_accepted=self.num_accepted_slowdowns,
        #                   blocking_rejected=self.num_rejected_slowdowns,
        #                   csv_filename="DOUBLECHECKED_thesis_L2C_" + str(NUM_VEHICLES) + "_simulation_results30fps.csv")
        super().remove_all_actors()
        super().terminate()

    def set_ambulance_time(self, time_taken):
        if self.first_ambulance_arrival_time is None:
            self.first_ambulance_arrival_time = time_taken
            self.ambulance_arrived = True
            self.terminate()

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
        #print("CHECKING IF AMBERLAMB HAS ARRIVED")
        # Check if the vehicle has arrived at the destination
        if self.vehicle_has_arrived():
            duration = time.time() - self.start_time
            self.scenario.set_ambulance_time(duration)
            print(f"Ambulance arrived! Duration: {duration} seconds")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def vehicle_has_arrived(self):
        # Example logic to check if the vehicle is within a certain distance of the destination
        vehicle_location = self.vehicle.get_location()
        #print("Vehicle location: {}".format(vehicle_location))
        distance = vehicle_location.distance(self.destination)
        #print("Distance to destination: {}".format(distance))
        if distance < 1.0:  # Arrival threshold
            return True
        return False


class ForceTMLaneChange(AtomicBehavior):
    def __init__(self, vehicle, start_lane_id, tm, scenario, exiting=False, direction='right', slow_duration=0.5,
                 original_speed=12.0, slow_speed=9.0, name="ForceTMLaneChange"):
        self.vehicle = vehicle
        self.start_lane_id = start_lane_id
        self.tm = tm
        self.scenario = scenario
        self.exiting = exiting
        self.direction = direction
        self.slow_duration = slow_duration
        self.original_speed = original_speed
        self.slow_speed = slow_speed
        self.current_speed = original_speed
        self.slow_down_time = None
        self.blocking_vehicle_id = None
        self.blocking_vehicle_response = None
        super(ForceTMLaneChange, self).__init__(name)

    def update(self):
        # Add logging for debugging
        #print(f"Ambulance {self.vehicle.id}: update called, current lane {get_vehicle_lane_id(self.vehicle)}")

        if self.exiting and get_vehicle_lane_id(self.vehicle) != RIGHTMOST_LANE_ID:
            if self.direction == 'right':
                # Check if lane is safe to merge
                safe, new_blocking_vehicle_id = lane_safe_to_merge(self.vehicle, 'right', num_waypoints=3)

                # If lane is safe, perform lane change and remove from BLACKBOARD
                if safe:
                    print(f"Ambulance {self.vehicle.id}: lane is safe, performing lane change.")
                    if self.current_speed != self.original_speed:
                        self.tm.set_desired_speed(self.vehicle, self.original_speed)
                        self.current_speed = self.original_speed
                        print(f"Ambulance {self.vehicle.id} is speeding up to {self.original_speed} DURING LANECHANGE.")
                    self.tm.force_lane_change(self.vehicle, True)
                    # Remove the blocking vehicle from BLACKBOARD since lane change is successful
                    if self.blocking_vehicle_id and self.blocking_vehicle_id in BLACKBOARD:
                        BLACKBOARD.remove(self.blocking_vehicle_id)
                    self.blocking_vehicle_id = None
                    self.blocking_vehicle_response = None  # Reset the response
                    #return py_trees.common.Status.SUCCESS

                # If not safe, add the blocking vehicle to BLACKBOARD and check DECISIONS
                else:
                    self.scenario.blocking_vehicles_encountered.add(new_blocking_vehicle_id)
                    if new_blocking_vehicle_id not in BLACKBOARD:
                        BLACKBOARD.add(new_blocking_vehicle_id)
                        self.blocking_vehicle_id = new_blocking_vehicle_id
                        # print(
                        #     f"Ambulance {self.vehicle.id}: adding blocking vehicle {new_blocking_vehicle_id} to BLACKBOARD.")

                    # Check DECISIONS for blocking vehicle's slowdown decision
                    if self.blocking_vehicle_id in DECISIONS:
                        self.blocking_vehicle_response = DECISIONS[self.blocking_vehicle_id]

                        # If blocking vehicle accepted slowdown, continue waiting and retry lane change
                        if self.blocking_vehicle_response:
                            self.scenario.accepted_slowdown_ids.add(self.blocking_vehicle_id)
                            print(
                                f"Blocking vehicle {self.blocking_vehicle_id} accepted the slowdown. Ambulance {self.vehicle.id} proceeding.")
                            return py_trees.common.Status.RUNNING

                        # If blocking vehicle rejected slowdown, slow down the ambulance temporarily
                        elif not self.blocking_vehicle_response:
                            self.scenario.rejected_slowdown_ids.add(self.blocking_vehicle_id)
                            print(
                                f"Ambulance {self.vehicle.id}: Blocking vehicle {self.blocking_vehicle_id} rejected slowdown.")

                            if self.slow_down_time is None:
                                self.scenario.num_rejected_slowdowns += 1
                                self.slow_down_time = time.time()
                                self.tm.set_desired_speed(self.vehicle, self.slow_speed)
                                self.current_speed = self.slow_speed
                                print(f"Ambulance {self.vehicle.id} is slowing down to {self.slow_speed}.")

                            # Check if the slowdown period is over
                            elapsed_time = time.time() - self.slow_down_time
                            print(
                                f"Ambulance {self.vehicle.id}: slowdown time elapsed = {elapsed_time:.2f}s (required {self.slow_duration}s).")

                            if elapsed_time > self.slow_duration:
                                self.tm.set_desired_speed(self.vehicle,
                                                          self.original_speed)  # Speed back up after waiting
                                self.current_speed = self.original_speed
                                self.slow_down_time = None
                                self.blocking_vehicle_response = None  # Reset to try again
                                print(
                                    f"Ambulance {self.vehicle.id} speeding up to {self.original_speed} after slowdown.")
                            return py_trees.common.Status.RUNNING

            # If lane change is complete, remove the vehicle from BLACKBOARD and succeed
            if get_vehicle_lane_id(self.vehicle) != self.start_lane_id:
                print(
                    f"Ambulance {self.vehicle.id} successfully changed lanes from {self.start_lane_id} to {get_vehicle_lane_id(self.vehicle)}.")
                # Remove blocking vehicle from BLACKBOARD once lane change is complete
                if self.blocking_vehicle_id in BLACKBOARD:
                    BLACKBOARD.remove(self.blocking_vehicle_id)
                self.blocking_vehicle_id = None
                self.blocking_vehicle_response = None  # Reset the response
                if self.current_speed != self.original_speed:
                    self.tm.set_desired_speed(self.vehicle, self.original_speed)
                    self.current_speed = self.original_speed
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


class SlowDownIfNeeded(AtomicBehavior):
    def __init__(self, vehicle, tm, scenario, benevolent=True, original_speed=15.0, slow_speed=9.0, slow_duration=5.0, name="SlowDownIfNeeded"):
        self.vehicle = vehicle
        self.tm = tm
        self.scenario = scenario
        self.benevolent = benevolent
        self.original_speed = original_speed
        self.slow_speed = slow_speed
        self.slow_duration = slow_duration
        self.slow_down_time = None
        self.slowed = False
        self.made_decision = False
        self.agreed = False # Keep track of response to ambulance
        super(SlowDownIfNeeded, self).__init__(name)

    def update(self):
        if self.vehicle.id in BLACKBOARD:
            if not self.made_decision:
                # decide if vehicle will agree to slow down
                agreement_probability = .85 if self.benevolent else .15
                if random.random() < agreement_probability:
                    self.agreed = True
                    DECISIONS[self.vehicle.id] = True
                    print("Vehicle {} has agreed to slow down".format(self.vehicle.id))
                    self.slow_down_time = time.time()
                    self.tm.set_desired_speed(self.vehicle, self.slow_speed)
                else:
                    print("Vehicle {} has refused to slow down".format(self.vehicle.id))
                    DECISIONS[self.vehicle.id] = False
                    self.agreed = False
                self.made_decision = True

        if self.made_decision and self.agreed:
            elapsed_time = time.time() - self.slow_down_time
            if elapsed_time > self.slow_duration:
                if self.vehicle.id in BLACKBOARD:
                    BLACKBOARD.remove(self.vehicle.id)
                self.tm.set_desired_speed(self.vehicle, self.original_speed)
                print("Vehicle {} has sped back up to original speed".format(self.vehicle.id))
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        elif self.made_decision and not self.agreed:
            return py_trees.common.Status.SUCCESS
            #return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING


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
        return py_trees.common.Status.SUCCESS

