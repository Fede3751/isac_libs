import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import isac_libs_main.utils.math_utils as math_utils


from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

from isac_libs_interfaces.msg import SensorPositions, SensorDataArray


SIMPLIFIED_SENSOR_SIM = True

class SimulationManager(Node):

    """
        Main ROS class which manages the execution of the whole simulation.
        The main task of the class is the connection of the TX and RX topics of the whole network.
    """


    def __init__(self):
        super().__init__("simulation_manager")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("edge_devices", 3),
                ("sensors", 3),
                ("sensors_range", 15),
            ],
        )
        self.cache_devices = (
            self.get_parameter("edge_devices").get_parameter_value().integer_value
        )
        self.sensors = self.get_parameter("sensors").get_parameter_value().integer_value
        self.sensors_range = (
            self.get_parameter("sensors_range").get_parameter_value().integer_value
        )
        self.sensor_positions = {}
        self.edge_device_positions = {}

        if SIMPLIFIED_SENSOR_SIM:

            self.create_subscription(
                SensorPositions,
                "/NetworkManager/SensorPositions",
                    self.store_simple_sensor_position,
                10
            )

            self.create_subscription(
                SensorDataArray,
                "/NetworkManager/SensorDataTX",
                self.forward_sensor_data_array,
                10
            )

            self.network_manager_data_topic = self.create_publisher(
                SensorDataArray,
                "/NetworkManager/SensorDataRX",
                10
            )


        else:
            
            self.sensors_rx = {}

            for i in range(self.sensors):
                
                self.create_subscription(
                    Odometry,
                    f"ActiveSensor_{i}/odometry",
                    lambda odometry_msg, sensor_id=i: (
                        self.store_sensor_position(sensor_id, odometry_msg)
                    ),
                    10,
                )

                self.create_subscription(
                    String,
                    f"ActiveSensor_{i}/tx_data",
                    lambda string_msg, sensor_id=i: (
                        self.forward_data(sensor_id, string_msg)
                    ),
                    10,
                )

                self.sensors_rx[i] = self.create_publisher(
                    String, f"ActiveSensor_{i}/rx_data", 10
                )

        self.edge_devices_rx = {}
        self.odometry_subs = []
        for i in range(self.cache_devices):
            self.odometry_subs.append(
                self.create_subscription(
                    Odometry,
                    f"EdgeDevice_{i}/odometry",
                    lambda odometry_msg, edge_device_id=i: (
                        self.store_edge_device_position(edge_device_id, odometry_msg)
                    ),
                    10,
                )
            )

            self.create_subscription(
                    String,
                    f"EdgeDevice_{i}/tx_data",
                    lambda string_msg, edge_device=i: (
                        self.forward_data(edge_device, string_msg)
                    ),
                    10,
                )

            self.edge_devices_rx[i] = self.create_publisher(
                String, f"EdgeDevice_{i}/rx_data", 10
            )


    def store_simple_sensor_position(self, position_msg: SensorPositions):
        for (sensor_id, sensor_pos) in enumerate(position_msg.positions):
            self.sensor_positions[sensor_id] = sensor_pos
            

    def store_sensor_position(self, sensor_id, position: Odometry):
        self.sensor_positions[sensor_id] = position.pose.pose.position

    def store_edge_device_position(self, edge_device_id, position: Odometry):

        self.edge_device_positions[edge_device_id] = position.pose.pose.position
        
        if len(self.edge_device_positions) >= self.cache_devices:
            for sub in self.odometry_subs:
                self.destroy_subscription(sub)

    def forward_data(self, device_id, msg: String):

        """
        Main method used to handle message between interfaces in the simulation.

        This behavior can be replaced by the Gazebo sensing plugin to have a
        configurable interface which resembles more accurately real case scenarios.
        """

        # Forward data to edge devices
        for i in range(self.cache_devices):
            
            if device_id in self.sensor_positions and i in self.edge_device_positions:
                
                pt_distance = math_utils.point_distance(
                    self.sensor_positions[device_id], self.edge_device_positions[i]
                )

                if pt_distance < self.sensors_range:
                    self.edge_devices_rx[i].publish(msg)

        # Forward data to sensors
        if SIMPLIFIED_SENSOR_SIM:

            msg_parsed = json.loads(msg.data)

            msg = SensorDataArray()
            msg.data = []

            for i in range(self.sensors):

                # Skip message to self
                if device_id == i:
                    continue

                if device_id in self.edge_device_positions and i in self.sensor_positions:
                    
                    pt_distance = math_utils.point_distance(
                        self.edge_device_positions[device_id], self.sensor_positions[i]
                    )

                    if pt_distance < self.sensors_range:

                        msg_parsed["sensor_id"] = i    
                        msg.data.append(json.dumps(msg_parsed))

            self.network_manager_data_topic.publish(msg)


        else:
            for i in range(self.sensors):

                # Skip message to self
                if device_id == i:
                    continue

                if device_id in self.edge_device_positions and i in self.sensor_positions:
                    
                    pt_distance = math_utils.point_distance(
                        self.edge_device_positions[device_id], self.sensor_positions[i]
                    )

                    if pt_distance < self.sensors_range:

                        self.sensors_rx[i].publish(msg)






    def forward_sensor_data_array(self, data_msg : SensorDataArray):


        for data_entry in data_msg.data:
            
            data_obj = json.loads(data_entry)
            
            for i in range(self.cache_devices):

                data_obj_position = Point(x=data_obj["position"][0], y=data_obj["position"][1], z=data_obj["position"][2])
                pt_distance = math_utils.point_distance(
                    data_obj_position, self.edge_device_positions[i]
                )
                if pt_distance < self.sensors_range:
                    self.edge_devices_rx[i].publish(String(data=data_entry))

            

def main():
    rclpy.init()
    simulationManager = SimulationManager()
    executor = MultiThreadedExecutor(64)
    executor.add_node(simulationManager)
    executor.spin()
    executor.shutdown()
    simulationManager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
