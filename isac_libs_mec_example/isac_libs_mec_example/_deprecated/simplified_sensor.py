import json
import random

import isac_libs_main.utils.math_utils as math_utils
from utils.sensor_data import SensorData


from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

from isac_libs_interfaces.srv import SenseRequest
from isac_libs_interfaces.action import Patrol

SENSING_RATE = (10, 10)
SENSING_RANGE = 10


class SimplifiedSensor():

    """
        Class used to simulate a Sensor without using ROS resources.

        It simulates "offline" a SensorClass, which can be used only by
        the MovementCoordinator when working in SIMPLIFIED_SENSOR_MODE.
        
        This Class can be used to extend numerical tests to more intense scenarios on
        low resource computers, where multiple ROS topics may prevent the simulation from
        working correctly.
    """


    def __init__(self, id, position=(0.0,0.0,0.0), agents=1):

        self.id                     = id
        self.agents                 = agents
        self.last_sensed_data       = None
        self.generated_data         = 0
        self.yaw                    = 0
        self.simulation_started     = False
        self.target_positions       = {}
        self.simple_publish_timer   = None
        self.patrol_thread          = None
        self.position               = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2])
        )


    def data_requested(self, request : SenseRequest.Request, response : SenseRequest.Response):
        cache_policy = request.req_cache
        self.publish_sensor_data(req_cache = cache_policy)
        return response


    def publish_sensor_data(self, req_cache = None):
        """Publishes randomly generated sensor data as a ROS2 message."""
        sensor_data = self.generate_random_sensor_data()

        sensor_id = self.id
        sensor_data_dict = sensor_data.to_dict()
        sensor_data_dict['sensor_id'] = sensor_id
        sensor_data_dict['type'] = "sensing_data"
        
        if req_cache is not None:
            sensor_data_dict['req_cache'] = req_cache
        
        sensor_data_serialized = json.dumps(sensor_data_dict) 

        # Publish the sensor data
        msg = String()
        msg.data = sensor_data_serialized
        self.last_sensed_data = msg
        
        return msg.data

    def generate_random_sensor_data(self):

        """Generate random sensor data"""
 
        # Create SensorData object
        sensor_data = SensorData(
            in_range=self.any_target_in_range(),
            position=(self.position.x, self.position.y, self.position.z))
        return sensor_data

    def any_target_in_range(self):
        
        for target in self.target_positions.values():
            if math_utils.point_distance(self.position, target) < SENSING_RANGE:
                return True
            
        return False

    def store_position(self, odometry_msg: Odometry):

        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w,
        )
        if not self.simulation_started and self.position.x != -1.0:
            self.simulation_started = True


    def store_target_position(self, id, msg: Odometry):
        self.target_positions[id] = msg.pose.pose.position
