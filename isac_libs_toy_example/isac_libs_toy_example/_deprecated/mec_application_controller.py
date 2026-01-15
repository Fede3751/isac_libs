import os
import time
import random
import math
import json
from threading import Thread
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

import isac_libs_main.utils.math_utils as math_utils
from isac_libs_toy_example.orchestrators.application_controller import ApplicationController
from utils.sensor_data import SensorData
from structs.cache_strategy import CacheType, BSUseCase

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from isac_libs_interfaces.msg import SensorPositions, BSQuery
from isac_libs_interfaces.action import Query




DATA = ["temperature", "blood pressure", "heartbeat"]
ERRORS = ["CacheExpiredError", "KeyNotFoundError", "DelayedDataResult"]
STRATEGY = BSUseCase.PROXIMITY
PROXIMITY_RANGE = 1
SIMPLIFIED_SENSOR_SIM = True


class QueryState(Enum):
    DISCOVERY = auto()
    PROXIMITY = auto()
    

class MECApplicationController(ApplicationController):

    """
        Class which dictates how the whole MEC application behaves.
        This class is additionally used to perform logging activities and produce evaluation data in form
        of json files.
    """

    def __init__(self):

        super().__init__("application_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("sensors", 3),
                ('cache_type', CacheType.FIFO.name),
                ('cache_size', 10),
                ('cache_expiration', 10),
                ('query_rate', 10),
                ('out_path', 'output_tests'),
                ('agents', 1)
            ],
        )

        self.no_edge_devices    = (self.get_parameter("edge_devices").get_parameter_value().integer_value)
        self.no_sensors         = self.get_parameter("sensors").get_parameter_value().integer_value
        self.no_agents          = self.get_parameter("agents").get_parameter_value().integer_value
        
        self.cache_type         = self.get_parameter('cache_type').get_parameter_value().string_value
        self.cache_size         = self.get_parameter('cache_size').get_parameter_value().integer_value
        self.cache_expiration   = self.get_parameter('cache_expiration').get_parameter_value().integer_value
        self.query_rate         = self.get_parameter('query_rate').get_parameter_value().integer_value
        self.out_path           = self.get_parameter('out_path').get_parameter_value().string_value
        

        self.goal_id            = 0
        self.query_x_range      = (50,50)
        self.query_y_range      = (50,50)
        self.edge_devices_action_clients     = {}
        self.sensor_positions   = {}
        self.agent_positions    = {}
        self.last_agent         = 0
        
        self.last_query_time    = time.time()
        self.out_file           = f"{time.time()}.json"

        self.active_query       = {
            "status": False,
            "timestamp": None
        }
        self.query_responses    = {
            policy.name: {}
            for policy in CacheType
        }
        self.last_pos_queried   = {
            policy.name : {
                "query_state": QueryState.DISCOVERY,
                "last_point": Point(x=50.0, y=50.0, z=0.0),
                "lost_track_count": 0
            } for policy in CacheType
        }


        self.query_announcer    = self.create_publisher(
            BSQuery,
            "/ApplicationController/LastQueriedPosition",
            100
        )

        # Edge Devices query action servers
        for i in range(self.no_edge_devices):

            self.edge_devices_action_clients[f"EdgeDevice_{i}"] = ActionClient(
                self,
                Query,
                f"/EdgeDevice_{i}/query_sensors",
            )

        # Sensor position topics. In case of simplified sensors, there is only
        # one topic from the NetworkManager.
        if SIMPLIFIED_SENSOR_SIM:
            
            self.create_subscription(
                SensorPositions,
                "/NetworkManager/SensorPositions",
                self.store_simple_sensor_position,
                10
            )

        else:
            
            for idx in range(self.no_sensors):
                self.create_subscription(
                    Odometry,
                    f"/ActiveSensor_{idx}/odometry",
                    lambda msg, id=idx: self.store_sensor_position(id, msg),
                    100,
                )

        for idx in range(self.no_agents):

            self.create_subscription(
                Odometry,
                f"/Agent_{idx}/odometry",
                lambda msg, id=idx: self.store_agent_position(id, msg),
                10
            )
        
        
        # Initialize output files
        for policy in CacheType:
            
            out_policy = os.path.join(self.out_path, policy.name)
            
            if not os.path.exists(out_policy):
                os.makedirs(out_policy)

            with open(f"{self.out_path}/{policy.name}/{self.out_file}", "w") as f:
                json.dump({
                    "sensors"           : self.no_sensors,
                    "edge_devices"      : self.no_edge_devices,
                    "cache_type"        : policy.name,
                    "cache_expiration"  : self.cache_expiration,
                    "cache_size"        : self.cache_size,
                    "query_rate"        : self.query_rate,
                    "queries"           : []
                }, f)


    def store_simple_sensor_position(self, position_msg: SensorPositions):
        
        """
            Store position callback. Used for the simple simulation environment.
            Reads the odometry msg and update position fields of all the sensors
            accordingly.
        """

        for (sensor_id, sensor_pos) in enumerate(position_msg.positions):
            self.sensor_positions[sensor_id] = sensor_pos
            

    def store_sensor_position(self, id, msg: Odometry):


        """
            Store position callback. Used for the Gazebo environment.
            Reads the odometry msg and update position fields of the single
            sensor accordingly.
        """

        self.sensor_positions[id] = msg.pose.pose.position

        # Redefine max boundaries given known position of sensors.
        # TO BE DEPRECATED: Query is going to be only agent based.
        self.query_x_range = (min(self.query_x_range[0], msg.pose.pose.position.x - 10), max(self.query_x_range[1], msg.pose.pose.position.x + 10))
        self.query_y_range = (min(self.query_y_range[0], msg.pose.pose.position.y - 10), max(self.query_y_range[1], msg.pose.pose.position.y + 10))


    def store_agent_position(self, id, msg: Odometry):

        """
            Store position callback. Used for both environments.
            Reads the odometry msg and update position fields agent
            accordingly.
        """

        self.agent_positions[id] = msg.pose.pose.position
        
    # --- QUERY CACHE METHODS ----

    def send_queries(self):

        """
            Method which defines the main behavior of the application.
            Custom application needs should reimplement starting from this method to
            define the querying behavior.
            
            The default scheduler runs all caching policies in parallel, meaning that 
            the same target will be the same for every caching policy, branching
            behaviors (i.e. BS queries defined by cache results) may need for single
            non-parallelized cache execution.
        """

        def send_queries_inner():

            # Check if everything in the simulation is working correctly
            while not self._perform_startup_check():
                self.get_logger().info("Waiting for all devices to be initialized")
                time.sleep(1)

            self.get_logger().info("Application Controller initialized and query thread started.")
    
            while True:

                if (
                    not self.active_query["status"]
                    and time.time() - self.last_query_time >= self.query_rate
                ):
                    
                    chosen_position = Point(    
                        x = float(random.randint(self.query_x_range[0], self.query_x_range[1])),
                        y = float(random.randint(self.query_y_range[0], self.query_y_range[1]))
                    )
    
                    for policy in CacheType:
                        self.query_policy(policy, chosen_position)

                    self.last_agent += 1

                    self.last_query_time = time.time()
                    self.active_query["status"] = True

                time.sleep(0.1)


        Thread(target=send_queries_inner).start()


    def query_policy(self, policy, chosen_position):

        """
            Internal Query method which queries a single cache strategy.
        """

        self.active_query["timestamp"] = time.time()

        sensor_id, target = self.choose_query_target(policy, chosen_position)
        self.goal_id += 1
        
        if sensor_id is not None:
            self._send_query_goal(sensor_id, policy, target)


    def choose_query_target(self, policy, random_position):

        """
            Method which defines the Query behavior of the BS.
            Application-defined behaviors can be defined by modifying this method.

            By default, a random agent is chosen, and the request is directly
            translated for the closest sensor to the agent.
        """

        target_chosen = random.randint(0, self.no_agents - 1)
        
        if target_chosen in self.agent_positions:
            target_position = self.agent_positions[target_chosen]
        else:
            self.get_logger().info("BS tried to query a target which is not known. This query will be skipped.")
            return None, None

        chosen_position = Point(x=target_position.x, y=target_position.y, z=0.0) #self.last_pos_queried[policy.name]["last_point"]
        announcer_msg = BSQuery(target = target_chosen, pos = Point(x=chosen_position.x, y=chosen_position.y))
        self.query_announcer.publish(announcer_msg)

        sensor_id = self._get_closest_sensor(chosen_position)      

        return sensor_id, target_chosen
    

    def _send_query_goal(self, queried_sensor, policy, target):

        query_goal_msg = Query.Goal(
            id = self.goal_id,
            sensor_id = queried_sensor,
            strategy = policy.name,
            target = target
        )
        
        # Send query actions to all edge services
        for edge_device in range(self.no_edge_devices):
            self._send_goal_future = self.edge_devices_action_clients[f"EdgeDevice_{edge_device}"].send_goal_async(query_goal_msg)
            self._send_goal_future.add_done_callback(self._query_response_callback)


    def _query_response_callback(self, future):


        """
            Internal method which checks the outcome of a Query request.
        """

        try:
            goal_handle = future.result()

        except RuntimeError:
            self.get_logger().info("An error occurred while trying to send a Query request to a Target")
            return self._conclude_query()
        
        if not goal_handle.accepted:
            self.get_logger().info(f'A query request for goal {self.goal_id} was rejected by the Edge Devices')
            self.active_query["status"] = False
            self.active_query["timestamp"] = None
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_query_result_callback)


    def _get_query_result_callback(self, query_future):

        """
            Internal Method. Used to retrieve the result of a Query Action.
            Concludes the whole query execution after all cache policies have answered.
        """

        result = query_future.result().result

        for response in result.sensor_response:

            response = json.loads(response)

            if response["data"] not in ERRORS :

                result_data = SensorData(
                        in_range        = response["data"]['in_range'],
                        misuration_ts   = response["data"]['timestamp'],
                        position        = response["data"]['position']
                )

                self.query_responses[response["policy"]][f"Balloon_{result.edge_device_id}"] = result_data
            else:
                self.query_responses[response["policy"]][f"Balloon_{result.edge_device_id}"] = response
            

        # Wait for all the policies to have received an answer by all edge devices, before writing results.
        for policy in self.query_responses.values():
            if len(policy) < self.no_edge_devices:
                return
            
        self._store_query_result()
        self._conclude_query()


    def _store_query_result(self):

        """
            Method used to write to file the results of a Query action.
        """

        for policy_name, policy in self.query_responses.items():

            out_file = f"{self.out_path}/{policy_name}/{self.out_file}"

            serialized = {"timestamp_request_start": self.active_query["timestamp"]}

            for policy_key, policy_result in policy.items():

                if type(policy_result) == SensorData:
                    serialized[policy_key] = policy_result.to_dict()
                else:
                    serialized[policy_key] = policy_result

            with open(out_file, "r") as f:
                try:
                    to_update = json.load(f)
                except:
                    self.get_logger().info(f"An error has prevented from reading results from {out_file}")
                    continue

            with open(out_file, "w") as f:
                to_update["queries"].append(serialized)
                json.dump(to_update, f)


    def _conclude_query(self):

        """
            Final method in the Query action.
            Used to reset all fields for the next Query.
        """


        self.active_query["status"] = False
        self.active_query["timestamp"] = None

        self.last_query_time = time.time()
        self.query_responses = {policy.name: {} for policy in CacheType}

    # ----------------------------

    def _get_closest_sensor(self, pos):

        """
            Inner method. Used to compute the closest sensor to the queried target.
        """

        min_dist = math.inf
        closest_sensor = -1

        for sensor_id, sensor_pos in self.sensor_positions.items():
            dist = math_utils.point_distance(pos, sensor_pos)
            if dist < min_dist:
                min_dist = dist
                closest_sensor = sensor_id

        return closest_sensor

    def _perform_startup_check(self):

        """
            Internal method. Used to perform a check on all entities on startup.
            Additionally resubmits to all device topics, as sometimes ROS submission
            may not work properly.
            
            Returns True when everything is ready.
        """

        missing_agents = []
        for idx in range(self.no_agents):
            
            if idx not in self.agent_positions:

                missing_agents.append(idx)

                self.create_subscription(
                    Odometry,
                    f"/Agent_{idx}/odometry",
                    lambda msg, id=idx: self.store_agent_position(id, msg),
                    100,
                )

        
        missing_edge_devices = []
        for agent_id, agent in self.edge_devices_action_clients.items():
            if not agent.wait_for_server(0.1):
                missing_edge_devices.append(agent_id)
    
        if (
            len(self.sensor_positions) < self.no_sensors or
            len(missing_agents) > 0 or
            len(missing_edge_devices) > 0 or
            len(self.agent_positions) < self.no_agents
        ):
            return False
        
        return True

    
def main():

    rclpy.init()
    executor = MultiThreadedExecutor(64)
    bs = MECApplicationController()
    executor.add_node(bs)
    bs.send_queries()

    executor.spin()

    executor.shutdown()
    bs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()