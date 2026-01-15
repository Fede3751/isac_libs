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
from isac_libs_toy_example.utils.sensor_data import SensorData
from isac_libs_toy_example.cache_types.cache_strategy import CacheType, BSUseCase
from isac_libs_toy_example.structs.msg_types import MsgType, MsgPayload

from std_msgs.msg import String
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
    

class ApplicationController(Node):

    """
        Class which dictates how the whole sensing application behaves.
        This class is additionally used to perform logging activities and produce evaluation data in form
        of json files.
    """

    def __init__(self):

        super().__init__("application_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("relays", 3),
                ('cache_size', 10),
                ('cache_expiration', 10),
                ('query_rate', 10),
                ('out_path', 'test_results'),
                ('agents', 1)
            ],
        )

        self.name                   = "ApplicationController"

        self.no_edge_devices        = self.get_parameter("edge_devices").get_parameter_value().integer_value
        self.no_relays              = self.get_parameter("relays").get_parameter_value().integer_value
        self.no_agents              = self.get_parameter("agents").get_parameter_value().integer_value
        
        self.cache_size             = self.get_parameter('cache_size').get_parameter_value().integer_value
        self.cache_expiration       = self.get_parameter('cache_expiration').get_parameter_value().integer_value
        self.out_path               = self.get_parameter('out_path').get_parameter_value().string_value

        self.edge_devices_tx_data               = {}
        self.cache_misses                       = {}
        self.cache_hits                         = {}
        self.total_agent_requests               = 0
        self.total_agent_requests_unsatisfied   = 0

        self.out_file                           = f"{time.time()}.json"


        self.query_responses    = {
            policy.name: {}
            for policy in CacheType
        }


        # Edge Devices query action servers
        for i in range(self.no_edge_devices):

            self.edge_devices_tx_data[f"EdgeDevice_{i}"] = self.create_subscription(
                String,
                f"/EdgeDevice_{i}/tx_data",
                lambda msg, id=i: self.handle_edge_device_events(id, msg),
                100,
            )


        for i in range(self.no_agents):

            self.create_subscription(
                String,
                f"/Agent_{i}/tx_data",
                lambda msg, id=i: self.update_agent_request_count(id, msg),
                100,
            )
            self.create_subscription(
                String,
                f"/Agent_{i}/rx_data",
                lambda msg, id=i: self.update_agent_request_count(id, msg, has_telemetry=True),
                100,
            )
        
        # Initialize output files
        for policy in CacheType:

            self.cache_misses[policy.name] = 0
            self.cache_hits[policy.name] = 0

            out_policy = os.path.join(self.out_path, policy.name)
            
            if not os.path.exists(out_policy):
                os.makedirs(out_policy)

            with open(f"{self.out_path}/{policy.name}/{self.out_file}", "w") as f:
                json.dump(
                    ""
                , f)


        self.create_timer(5, self.print_evaluation)


    def handle_edge_device_events(self, edge_device_id, string_msg : String):
        
        payload = MsgPayload.from_json(string_msg.data)

        if payload.msg_dest != self.name:
            return        

        if payload.msg_type == MsgType.EDGE_DEVICE_CACHE_MISS:
            self.cache_misses[payload.msg_content] += 1
        elif payload.msg_type == MsgType.EDGE_DEVICE_CACHE_HIT:
            self.cache_hits[payload.msg_content] += 1

    def print_evaluation(self):

        if self.total_agent_requests == 0:
            return 
        
        miss_rate = {}

        for policy in self.cache_misses.keys():
            miss_rate[policy] = self.cache_misses[policy] / max(1, self.cache_hits[policy] + self.cache_misses[policy])

        self.get_logger().info(f"{miss_rate}")

        for policy in CacheType:

            out_policy = os.path.join(self.out_path, policy.name)
            
            if not os.path.exists(out_policy):
                os.makedirs(out_policy)

            with open(f"{self.out_path}/{policy.name}/{self.out_file}", "w") as f:
                json.dump({
                    "miss_rate"             : self.cache_misses[policy.name] / max(1, self.cache_hits[policy.name] + self.cache_misses[policy.name]),
                    "total_misses"          : self.cache_misses[policy.name],
                    "total_hits"            : self.cache_hits[policy.name],
                    "total_requests"        : self.total_agent_requests,
                    "unsatisfied_requests"  : self.total_agent_requests_unsatisfied
                }, f)

    def update_agent_request_count(self, agent_id, string_msg : String, has_telemetry = False):


        if has_telemetry:
            
            payload = MsgPayload.from_json(json.loads(string_msg.data)["payload"])
            
            if payload.msg_type == MsgType.SESSION_DATA_AGENT_CONTENT and payload.msg_dest == f"Agent_{agent_id}":
                self.total_agent_requests_unsatisfied -= 1

        else:
            
            payload = MsgPayload.from_json(string_msg.data)

            if payload.msg_type == MsgType.SESSION_DATA_AGENT_REQ:
                self.total_agent_requests += 1
                self.total_agent_requests_unsatisfied += 1


    
def main():

    rclpy.init()
    executor = MultiThreadedExecutor(64)
    bs = ApplicationController()
    executor.add_node(bs)
    #bs.send_queries()

    executor.spin()

    executor.shutdown()
    bs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()