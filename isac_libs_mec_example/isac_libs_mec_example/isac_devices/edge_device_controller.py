import math
import json
from enum import Enum, auto
import random

from isac_libs_mec_example.structs.msg_types import MsgType, MsgPayload
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from isac_libs_main.isac_device_controller import ISACDeviceController
import isac_libs_main.utils.math_utils as math_utils
from isac_libs_mec_example.structs.cache_strategy import CacheType, CacheFactory, CacheExpiredError, KeyNotFoundError
from isac_libs_mec_example.utils.sensor_data import SensorData


from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry

from isac_libs_interfaces.msg import BSQuery
from isac_libs_interfaces.srv import SenseRequest
from isac_libs_interfaces.action import Query




class SensingType(Enum):
    PASSIVE = auto()
    ACTIVE = auto()

SENSING_TYPE = SensingType.PASSIVE


class EdgeDeviceController(ISACDeviceController):

    """
        Class used to define of the Wireless Edge Caching devices.
        The device implements caching strategy to store in its memory the data produced
        by the sensors on the field.
    """

    def __init__(self):
        super().__init__("edge_device_controller")
        self.get_logger().info("...Done")



        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("agents", 1)
                # ("focus", 1.0)
            ],
        )

        self.name = f"EdgeDevice_{self.id}"

        # --- Cache Info topic ---
        # Used to broadcast the cache content of the Edge Device
        # and display it to screen.
        self.cache_info_publisher = self.create_publisher(
            String,
            'cache_content',
            10
        )

        self._init_cache()



    def search_response(self, cache_strategy, cache_entry):

        """
            Method used by the Edge Device to look for an entry in its cache.

            Returns a string message with the response message already formatted.
        """

        # Start creating response message
        # response_msg = {
        #     "sensor_id": cache_entry,
        #     "policy": cache_strategy
        # }
                
        try:

            #self.get_logger().info(f"Looking for {cache_entry}")
            cached_data = self.caches[cache_strategy].__getitem__(cache_entry)
            return True
            # response_msg["data"] = cached_data.to_dict()


        except CacheExpiredError:
            
            # response_msg["data"] = "CacheExpiredError"
            #self.request_data(cache_entry, req_cache=cache_strategy)
            #self.sensing_requests[cache_strategy] += 1
            #self.get_logger().info("Cache entry has expired")
            return False

        except KeyNotFoundError:
            
            # response_msg["data"] = "KeyNotFoundError"
            # self.request_data(cache_entry, req_cache=cache_strategy)                
            #self.sensing_requests[cache_strategy] += 1
            #self.get_logger().info("Cache entry was not found")
            return False

        #return json.dumps(response_msg)    


    def _init_cache(self):

        self.declare_parameters(
            namespace='',
            parameters=[
                ('cache_size', 5),
                ('cache_expiration', 100),
            ]
        )

        cache_size          = self.get_parameter('cache_size').get_parameter_value().integer_value
        cache_expiration    = self.get_parameter('cache_expiration').get_parameter_value().integer_value

        # Create the cache table of all the standard cache strategies
        self.caches = {
            cache_type.name: CacheFactory.create(
                cache_type.name,
                cache_size,
                cache_expiration,
            ) for cache_type in CacheType
        }
        

    def rx_callback(self, string_msg : String):

        """
            The main method which defines how a caching devices behaves when receiving data to its interface.
            The default behavior handles all the caching strategies together, to optimize testing on a single run.
            
            After updating to the cache tables, the new content is published to /EdgeDevice_{i}/cache_content
        """


        msg_dict = json.loads(string_msg.data)

        if not msg_dict["payload"]:
            return
        
        payload = MsgPayload.from_json(msg_dict["payload"])
        telemetry = msg_dict["telemetry"]

        if payload.msg_type == MsgType.HELLO_REPLY:

            payload_content_parsed = json.loads(payload.msg_content)
            cache_entry = payload_content_parsed["tx_id"] #(telemetry["tx_id"], payload_content_parsed["tx_id"])

            if cache_entry in self.caches[CacheType.CIFO.name].cache:
            # for cache in self.caches.values():
            #     if cache_entry in cache.cache.keys():
                    self.caches[CacheType.CIFO.name][cache_entry].strength = payload_content_parsed["strength"]

        if payload.msg_dest != "EdgeDevice" and payload.msg_dest != self.name:
            return
        

        if payload.msg_type == MsgType.RELAY_NEGOTIATE_EDGE_DEVICE_REQ:
            
            msg = MsgPayload(
                MsgType.RELAY_NEGOTIATE_EDGE_DEVICE_REPLY,
                msg_content = "",
                msg_dest = telemetry["tx_id"]
            )

            self.tx_data.publish(String(data=msg.to_msg()))

            return

        if payload.msg_type == MsgType.SESSION_DATA_RELAY_REQ:

            payload_content_parsed = json.loads(payload.msg_content)
        #self.get_logger().info(f"Msg received: {payload}")

            for cache_type, cache_table in self.caches.items():

                cache_entry = payload_content_parsed["tx_id"]#(telemetry["tx_id"], payload_content_parsed["tx_id"])

                # self.get_logger().info(f"{cache_entry}")

                if not self.search_response(cache_type, cache_entry): #CACHE_MISS

                    msg = MsgPayload(
                        MsgType.EDGE_DEVICE_CACHE_MISS,
                        msg_content=f"{cache_type}",
                        msg_dest = "ApplicationController"
                    )
        
                    self.caches[cache_type][cache_entry] = SensorData(payload_content_parsed["strength"])
                    self.tx_data.publish(String(data=msg.to_msg()))
            

                else:

                    msg = MsgPayload(
                        MsgType.EDGE_DEVICE_CACHE_HIT,
                        msg_content=f"{cache_type}",
                        msg_dest = "ApplicationController"
                    )
                    
                    self.caches[cache_type][cache_entry] = SensorData(payload_content_parsed["strength"])
                    self.tx_data.publish(String(data=msg.to_msg()))
        

        

            msg = MsgPayload(
                MsgType.SESSION_DATA_RELAY_CONTENT,
                msg_content=payload.msg_content,
                msg_dest = telemetry["tx_id"]
            )

            self.tx_data.publish(String(data=msg.to_msg()))


        # # Data conversion is stored inside a try-except control. This prevents unexpected messages
        # # from crashing the device. Unhandled data is printed to console.
        # try:
        #     sensor_data_dict = json.loads(msg.data)
        #     sensor_id = sensor_data_dict.get("sensor_id", "Unknown")

        #     # Sensing requests are ignored by Edge Devices
        #     if sensor_data_dict.get("type", "Unknown") == "sensing_request":
        #         return
            
        #     sensor_data = self.convert_dict_to_sensordata(sensor_data_dict)
        # except:
        #     self.get_logger().info(f"Unexpected data received. Content:\n\t{msg}")
        #     return


        # return 
    
        # # Due to parallel execution between caches, some sensing requests may have been evoked by
        # # a single cache strategy. We distinct the two cases here, and add new entries to the caches accordingly.
        # if sensor_data.req_cache is None:

        #     for cache in self.caches.values():
        #         cache[sensor_id] = sensor_data
        #     for target_cache in self.loc_cache.values():
        #         target_cache[sensor_id] = sensor_data

        # else:

        #     # For the case of CIFO data, we run the assignment to every sub-table.
        #     # Single CIFO tables will handle the data accordingly.
        #     if sensor_data.req_cache == CacheType.CIFO:

        #         for target_cache in self.loc_cache.values():
        #             target_cache[sensor_id] = sensor_data

        #     else:
        #         self.caches[sensor_data.req_cache][sensor_id] = sensor_data
    
        # self.publish_cache_content()


    def publish_cache_content(self):

        """
            Method used to publish a formatted message to the cache_content topic
        """

        cache_msg = String()
        cache_formatted = {}

        for cache_key, cache_obj in self.caches.items():
            cache_formatted[cache_key] = [key for key in cache_obj.cache.keys()]

        cache_msg.data=json.dumps(cache_formatted)

        # The device broadcasts its new cache entry to the cache info topic.
        self.cache_info_publisher.publish(cache_msg)


    def convert_dict_to_sensordata(self, sensor_data_dict: dict):

        sensor_data = SensorData(
            in_range=sensor_data_dict['in_range'],
            misuration_ts=sensor_data_dict['timestamp'],
            position=sensor_data_dict['position'],
            req_cache = sensor_data_dict['req_cache'] if 'req_cache' in sensor_data_dict else None
        )
        return sensor_data


def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor(32)
    edge_device = EdgeDeviceController()
    executor.add_node(edge_device)

    executor.spin()

    edge_device.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()