import math
import random
import json
from enum import Enum, auto

from isac_libs_mec_example.structs.msg_types import MsgPayload
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from isac_libs_main.isac_device_controller import ISACDeviceController
import isac_libs_main.utils.math_utils as math_utils
from isac_libs_main.utils.event_scheduler import EventScheduler
from utils.sensor_data import SensorData
from isac_libs_mec_example.structs.msg_types import MsgType


from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

from isac_libs_interfaces.srv import SenseRequest
from isac_libs_interfaces.action import Patrol


WORLD_NAME = ""
SENSING_RATE = (10, 10)
SENSING_RANGE = 10

class SensingType(Enum):
    PASSIVE = auto()
    ACTIVE = auto()

SENSING_TYPE = SensingType.PASSIVE

class RelayDeviceController(ISACDeviceController):
    def __init__(self):
        super().__init__("antenna_relay_controller")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("agents", 1),
                ("world_name", "simple")
                
            ],
        )

        self.name = f"AntennaRelay_{self.id}"

        self.associated_edge_device = "EdgeDevice"
        self.associated_edge_device_bw = 0

        self.agents     = self.get_parameter("agents").get_parameter_value().integer_value
        self.world_name = self.get_parameter("world_name").get_parameter_value().string_value


        self.last_sensed_data = None

        self.generated_data = 0


        self.stop_msg = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular = Vector3(x=0.0, y=0.0, z=0.0)
        )

        #self.tx_topic = self.create_publisher(String, "tx_data", 10)

        self.event_scheduler = EventScheduler()

        self.clock_topic = self.create_subscription(
            Clock, f"/world/{self.world_name}/clock", self.event_scheduler.routine, 10
        )

        self.patrol_action_server = ActionServer(
            self, Patrol, "patrol", self.execute_patrol_action
        )

        self.simulation_started = False

        # Create a timer to check for simulation start
        
        self.create_timer(1, self.negotiate_edge_device)

        self.simple_publish_timer = None
        self.patrol_thread = None


    def negotiate_edge_device(self):

        if self.associated_edge_device == "EdgeDevice":

            #self.get_logger().info("Looking for a suitable edge device")

            msg = MsgPayload(
                MsgType.RELAY_NEGOTIATE_EDGE_DEVICE_REQ,
                msg_content = "",
                msg_dest = "EdgeDevice"
            )

            self.tx_data.publish(String(data=msg.to_msg()))

    def execute_patrol_action(self, goal: ServerGoalHandle):
        command_goal: Patrol.Goal = goal.request

        target = command_goal.targets[0]
        self.rotate_to_target(target)
        self.move_to_target(target)
        goal.succeed()

        result = Patrol.Result()
        result.result = "Movement completed"

        return result

    def rotate_to_target(self, target, eps=0.5):

        target_angle = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = target_angle - self.yaw

        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi

        rotation_dir = 1 if angle_to_rotate < 0 else -1

        # Prepare the cmd_vel message
        move_msg = Twist()
        move_msg.linear = Vector3(x=1.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=0.5 * rotation_dir)
        self.cmd_vel_publisher.publish(move_msg)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        # Note that here the eps also helps us stop the drone and not overshoot the target, as
        # the drone will keep moving for a while after it receives a stop message
        # Also note that rotating the drone too fast will make it loose altitude.
        # You can account for that by also giving some z linear speed to the rotation movement.
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.yaw

            # No sleep here. We don't want to miss the angle by sleeping too much. Even 0.1 seconds
            # could make us miss the given epsilon interval

        # When done, send a stop message to be sure that the drone doesn't
        # overshoot its target
        self.cmd_vel_publisher.publish(self.stop_msg)

    def move_to_target(self, target, eps=0.5, angle_eps=0.02):
        # Save the target position and compute the distance
        distance = math_utils.point_distance(self.position, target)

        # Keep publishing movement while the distance is greater than the given EPS
        while distance > eps:

            # Compute the move vector with the given position and target
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0]
            twist_msg.linear.z = mv[1]

            # Check if Balloon is still facing the target correctly, otherwise add angular
            # velocity to the Twist msg
            target_angle = math_utils.angle_between_points(self.position, target)

            if not (target_angle - angle_eps < self.yaw < target_angle + angle_eps):
                angle_diff = self.yaw - target_angle
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))

            # Publish msg
            self.cmd_vel_publisher.publish(twist_msg)

            # Update position and distance after finishing
            distance = math_utils.point_distance(self.position, target)

        # After reaching the target, publish a stop msg
        self.cmd_vel_publisher.publish(self.stop_msg)

    def rx_callback(self, string_msg: String):

        msg_dict = json.loads(string_msg.data)

        if not msg_dict["payload"]:
            return
        
        payload = MsgPayload.from_json(msg_dict["payload"])
        telemetry = msg_dict["telemetry"]

        if payload.msg_dest != "" and payload.msg_dest != self.name:
            #self.get_logger().info(f"Received a msg for someone else ({payload.msg_dest}). Ignoring")
            return

        if payload.msg_type == MsgType.HELLO:

            msg = MsgPayload(
                MsgType.HELLO_REPLY,
                msg_content = json.dumps({
                    "strength": telemetry["achievable_rate_bps"],
                    "tx_id": telemetry["tx_id"]
                })
            )
            self.tx_data.publish(String(data=msg.to_msg()))

        if payload.msg_type == MsgType.RELAY_NEGOTIATE_EDGE_DEVICE_REPLY:

            if telemetry["achievable_rate_bps"] > self.associated_edge_device_bw:

                self.associated_edge_device_bw = telemetry["achievable_rate_bps"]
                self.associated_edge_device = telemetry["tx_id"]

                self.get_logger().info(f"Associating a new edge device ({telemetry['tx_id']})")

        if payload.msg_type == MsgType.SESSION_DATA_AGENT_REQ:

            msg = MsgPayload(
                MsgType.SESSION_DATA_RELAY_REQ,
                msg_content = json.dumps({
                    "strength": telemetry["achievable_rate_bps"],
                    "tx_id": telemetry["tx_id"]
                }),
                msg_dest = self.associated_edge_device
            )
            #self.get_logger().info("Data was requested")

            self.tx_data.publish(String(data=msg.to_msg()))

        if payload.msg_type == MsgType.SESSION_DATA_RELAY_CONTENT:

            msg = MsgPayload(
                MsgType.SESSION_DATA_AGENT_CONTENT,
                msg_content = "Session data",
                msg_dest = json.loads(payload.msg_content)["tx_id"]
            )

            self.tx_data.publish(String(data=msg.to_msg()))


def main():
    rclpy.init()

    executor = MultiThreadedExecutor(32)
    sensor_controller = RelayDeviceController()
    executor.add_node(sensor_controller)
    executor.spin()
    executor.shutdown()
    sensor_controller.destroy_node()

    rclpy.shutdown()
