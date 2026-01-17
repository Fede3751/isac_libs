import math
import random
import json
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from isac_libs_main.utils.event_scheduler import EventScheduler
from isac_libs_main.isac_device_controller import ISACDeviceController
import isac_libs_main.utils.math_utils as math_utils
from isac_libs_mec_example.structs.msg_types import MsgType, MsgPayload

from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

from isac_libs_interfaces.action import Patrol



STOP_MSG = Twist(
    linear  =   Vector3(x=0.0, y=0.0, z=0.0),
    angular =   Vector3(x=0.0, y=0.0, z=0.0)
)

MIN_WAIT_BETWEEN_REQUEST = 2
MAX_WAIT_BETWEEN_REQUEST = 4
WORLD_NAME = "sensing_world"

class AgentController(ISACDeviceController):
    
    """
        Class used to manage the Agents on the field.

        No simplified version of this class is given, as Agents are moving and therefore can implement more
        complex behaviors than sensors.
    """

    def __init__(self):

        super().__init__("agent_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("agent_speed", 1),
                ("world_name", "empty")
            ],
        )

        self.speed      = self.get_parameter("agent_speed").get_parameter_value().integer_value
        
        self.event_scheduler    = EventScheduler()
        self.simulation_started = False
        self.relay_status = {}

        self.start_x = 0


        # ---Subscription Topics---
        # An Agent Controller subscribes to its odometry topic to keep its position in
        # the simulation updated, and to the clock of Gazebo world, to keep in sync with the
        # simulation clock when performing events.
        self.create_subscription(
            Clock,
            f"/world/{WORLD_NAME}/clock",
            self.event_scheduler.routine,
            10
        )


        # ---Action Servers---
        # An Agent Controller exposes to ROS an Action Server for imparting movement orders.
        self.patrol_action_server = ActionServer(
            self, Patrol, "patrol", self.execute_patrol_action
        )
        self.patrol_action_client = ActionClient(
            self, Patrol, "patrol"
        )

        self.event_scheduler.schedule_event(
            random.randint(MIN_WAIT_BETWEEN_REQUEST, MAX_WAIT_BETWEEN_REQUEST-1) + random.random(),
            self.request_session_data,
            repeat=False
        )

        self.event_scheduler.schedule_event(0.5, self.send_hello_packet)
        self.event_scheduler.schedule_event(1, self.submit_next_move, repeat=False)
        self.event_scheduler.schedule_event(20, self.submit_next_move)


    def odometry_callback(self, odometry_msg: Odometry):

        """
            Store position callback. Reads the odometry msg and update position fields of the 
            Agent Class accordingly.
        """
        
        super().odometry_callback(odometry_msg)
        
        # A simulation is considered started as soon as the first odometry
        # msg is received. Default behavior of a Gazebo simulation.
        if not self.simulation_started:
            self.simulation_started = True
            self.start_x = odometry_msg.pose.pose.position.x


    def submit_next_move(self):

        goal = Patrol.Goal()
        goal.target = Point(x=self.start_x, y = 10.0 if self.position.y < 0 else -10.0, z=0.0)

        self.patrol_action_client.send_goal_async(goal)

        pass

    def execute_patrol_action(self, patrol_req: ServerGoalHandle):

        """
            Method responsible for handling patrol actions.

            A patrol action takes in input a target point and imparts commands to the Agent
            Controller by using the cmd_vel topic.

            Complex simulation environment with obstacles should implement their
            movement strategy by modifying this method accordingly.
        """

        self._rotate_to_target(patrol_req.request.target)
        self._move_to_target(patrol_req.request.target, eps = self.speed)

        patrol_req.succeed()

        result = Patrol.Result()
        result.result = "Movement completed"

        return result


    def _rotate_to_target(self, target, eps=0.5):

        """
            Internal method. Used by the Patrol Action.

            This method performs the rotation towards the target.
        """

        target_angle    = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = (target_angle - self.orientation + math.pi) % (2 * math.pi) - math.pi
        rotation_dir    = 1 if angle_to_rotate < 0 else -1

        # Prepare the cmd_vel message
        move_msg = Twist(
            linear = Vector3(x=0.0, y=0.0, z=0.0),
            angular = Vector3(x=0.0, y=0.0, z=0.1 * rotation_dir)
        )
        self.cmd_vel_publisher.publish(move_msg)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.orientation

        # When done, send a stop message to be sure that the drone doesn't
        # overshoot its target
        self.cmd_vel_publisher.publish(STOP_MSG)


    def _move_to_target(self, target, eps=0.5, angle_eps=0.02):

        """
            Internal Method. Used by the Patrol Action.

            This method performs the linear movement after the target is correctly
            pointed. Some angular speed is added to account for rounding errors
            during the rotation phase.
        """

        # Save the target position and compute the distance
        distance = math_utils.point_distance(self.position, target)

        # Keep publishing movement while the distance is greater than the given EPS
        while distance > eps:

            # Compute the move vector with the given position and target
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0] * min(distance, self.speed)
            twist_msg.linear.z = mv[1] * min(distance, self.speed)

            # Check if the Agent is still facing the target correctly, otherwise add angular
            # velocity to the Twist msg
            target_angle = math_utils.angle_between_points(self.position, target)

            if not (target_angle - angle_eps < self.orientation < target_angle + angle_eps):
                angle_diff = self.orientation - target_angle
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))

            # Publish msg
            self.cmd_vel_publisher.publish(twist_msg)

            # Update position and distance after finishing
            distance = math_utils.point_distance(self.position, target)

        # After reaching the target, publish a stop msg
        self.cmd_vel_publisher.publish(STOP_MSG)


    def request_session_data(self):
        
        
        if len(self.relay_status) == 0:
            return
        
        highest_rate = max(
            self.relay_status,
            key=lambda k: self.relay_status[k]["achievable_rate_bps"]
        )

        msg_payload = MsgPayload(MsgType.SESSION_DATA_AGENT_REQ, "", msg_dest=highest_rate)

        self.tx_data.publish(String(data = msg_payload.to_msg()))

        self.event_scheduler.schedule_event(
            random.randint(MIN_WAIT_BETWEEN_REQUEST, MAX_WAIT_BETWEEN_REQUEST-1) + random.random(),
            self.request_session_data,
            repeat=False
        )

    def send_hello_packet(self):

        msg = String(data = MsgPayload(MsgType.HELLO).to_msg())
        self.tx_data.publish(msg)
        
    def rx_callback(self, string_msg: String):

        # A message received is expected to have telemetry and payload.
        # We extrapolate value from the payload to understand the type of message

        msg_dict = json.loads(string_msg.data)

        if not msg_dict["payload"]:
            return
        
        payload = MsgPayload.from_json(msg_dict["payload"])
        telemetry = msg_dict["telemetry"]
                
        
        
        if payload.msg_type == MsgType.HELLO_REPLY:
            self.relay_status[telemetry['tx_id']] = telemetry

        if payload.msg_type == MsgType.SESSION_DATA_AGENT_CONTENT:

            # Perform action with data received (currently no behavior, as we just
            # evaluate caching).
            
            pass


def main():
    
    rclpy.init()

    executor = MultiThreadedExecutor(32)
    target_controller = AgentController()
    executor.add_node(target_controller)
    executor.spin()
    executor.shutdown()
    target_controller.destroy_node()

    rclpy.shutdown()
