from time import sleep
import random
import json
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from isac_libs_mec_example.utils.toy_example_utils import compute_area_coverage, edge_device_placement, compute_random_covered_positions
from isac_libs_mec_example._deprecated.simplified_sensor import SimplifiedSensor

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from isac_libs_interfaces.msg import SensorPositions, SensorDataArray
from isac_libs_interfaces.srv import SenseRequest
from isac_libs_interfaces.action import Patrol



SIMPLIFIED_SENSOR_SIM = True
SIMPLIFIED_SIM_PUBLISH_INTERVAL = 0.5
SENSING_RATE = [10,10]


class SensorNetworkManager(Node):
    """
    Sensor Manager class, used the manage the whole network of Sensors.
    
    It dictates how sensors behave collectively, and can be used to interrogate single sensors depending
    on particular tasks or requirements by the central BS.

    Two main behaviors implemented in this class:
        -Simplified Manager:
            The Manager simulates the fleet of sensors on its own, meaning that the sensors on the field
            are not actual ROS nodes. This behavior is compatible only with the 2D view execution, suitable
            for testing new caching strategies in terms of numerical evaluation.
        -Complete Manager:
            The Manager communicates with sensors through the ROS channel. This behavior can be used
            for running the 3D Gazebo simulation, allowing to implement solutions which can be translated
            more easily to a real-case scenario.
            This scenario additionally allows for movement to be implemented on top of the sensors, 
            suitable for more complex scenarios.

    Behavior is decided on runtime depending on the type of simulation chosen (--sim-type [2d, 3d])
    """


    def __init__(self):

        super().__init__("sensor_manager")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("relays", 3),
                ("sensors_range", 50),
                ("field_width", 100),
                ("field_height", 100),
                ("agents", 1)
            ],
        )

        self.edge_devices  = self.get_parameter("edge_devices").get_parameter_value().integer_value
        self.relays        = self.get_parameter("relays").get_parameter_value().integer_value
        self.sensors_range  = self.get_parameter("sensors_range").get_parameter_value().integer_value
        self.field_width    = self.get_parameter("field_width").get_parameter_value().integer_value
        self.field_height   = self.get_parameter("field_height").get_parameter_value().integer_value
        self.agents         = self.get_parameter("agents").get_parameter_value().integer_value
        self.behavior       = "2d"




        #----- edge device topics ---








class SensorState(Enum):
    IDLE = 1
    SENSING = 2
    MOVING = 3



def main():

    rclpy.init()

    executor = MultiThreadedExecutor(64)
    sensor_network_manager = SensorNetworkManager()

    executor.add_node(sensor_network_manager)
    #movement_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    sensor_network_manager.destroy_node()
    rclpy.shutdown()
