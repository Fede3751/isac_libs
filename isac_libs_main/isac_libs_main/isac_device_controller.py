import math
import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import isac_libs_main.utils.math_utils as math_utils

from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry

from isac_libs_interfaces.msg import BSQuery
from isac_libs_interfaces.srv import SenseRequest
from isac_libs_interfaces.action import Query



class ISACDeviceController(Node):

    """
        This is a base class which represents an ISAC device.
        By launching this class in the proper namespace, the device is automatically linked
        with the Gazebo simulation.

        The main links are in the movement topics (cmd_vel and odometry)
        and in the transmission topics (tx_data, rx_data).

        Automatic callback functions are already linked for storing odometry in a position field.

        rx_data callback is given empty, and expected to be reimplemented based on application
        behavior.
    """

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameters(
            namespace="",
            parameters=[
            ],
        )

        self.id             = self.declare_parameter('id', -1).get_parameter_value().integer_value
        self.position       = Point(x=0.0, y=0.0, z=0.0)
        self.orientation    = 0

        

    

        # --- Movement Topics ---
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.odometry_subscriber = self.create_subscription(
            Odometry,
            'odometry',
            self.odometry_callback,
            10
        )

        # --- Transmission Topics ---
        self.rx_data = self.create_subscription(
            String,
            'rx_data',
            self.rx_callback,
            100
        )

        self.tx_data = self.create_publisher(
            String,
            'tx_data',
            100
        )


    """
        Default behavior for odometry topic callback. The function simply stores
        the read data from the odometry message to a "position" field. Additionally,
        current quaternion rotation is automatically converted in degrees and stored in
        "orientation"
    """
    def odometry_callback(self, odometry_msg: Odometry):

        """
            Store position callback.
            Used to update the position of the Edge Device.
        """

        self.position = odometry_msg.pose.pose.position

        self.orientation = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )


    def rx_callback(self, string_msg : String):
        pass

