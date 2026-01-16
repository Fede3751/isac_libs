from time import sleep
from threading import Thread
from enum import Enum
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from isac_libs_interfaces.action import Patrol
from geometry_msgs.msg import Pose, Point, Quaternion



class AgentNetworkManager(Node):

    """
    Agent Coordinator class, used the manage in a centralized approach all
    the targets.
    
    This class communicates with all the targets in the simulation, in order to 
    perform a common behavior among targets.

    It's main use is to move agents in the simulation.
    """

    def __init__(self):

        super().__init__("agent_network_manager")

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

        self.edge_devices   = self.get_parameter("edge_devices").get_parameter_value().integer_value
        self.relays         = self.get_parameter("relays").get_parameter_value().integer_value
        self.no_agents      = self.get_parameter("agents").get_parameter_value().integer_value

        self.sensors_range  = self.get_parameter("sensors_range").get_parameter_value().integer_value
        self.field_width    = self.get_parameter("field_width").get_parameter_value().integer_value
        self.field_height   = self.get_parameter("field_height").get_parameter_value().integer_value

        
        self.odometry_subs          = []    
        self.agent_action_clients   = {}
        self.agent_positions        = {}
        self.agent_states           = {}   
    



        # ---Agent Topics---    
        # An Agent Manager subscribes to the odometry topics of the agents, to keep 
        # their position always up to date, and to their Patrol Action server, to 
        # impart patrol actions to targets.
        for i in range(self.no_agents):

            self.agent_states[i] = SensorState.IDLE

            self.create_subscription(
                Odometry,
                f"/Agent_{i}/odometry",
                lambda msg, id=i: self.store_agent_position(id, msg),
                100,
            )

            self.agent_action_clients[i] = ActionClient(
                self,
                Patrol,
                f"/Agent_{i}/patrol"
            )
            

            

    def store_agent_position(self, id, msg: Odometry):
        
        """
            Store position callback. Reads the odometry msg and update position fields of the 
            Agent Manager accordingly.
        """
        
        self.agent_positions[id] = msg.pose.pose.position




    def random_walk(self):
        """
        Method used to keep the network of Agents constantly moving in a random mode.
        Agents are redirected towards new points as soon as they reach their target
        """

        def random_walk_inner():
            while True:
                for i in range(self.no_agents):
                    # Do not resubmit tasks to already moving sensors
                    if not self.agent_states[i] is SensorState.MOVING:
                        self._random_walk_task(i)

        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=random_walk_inner).start()


    def _random_walk_task(self, agent_id: int):

        """
            Internal method. Used for the submission of Walk tasks.
        """

        # Set the Sensors to moving state
        self.agent_states[agent_id] = SensorState.MOVING

        goal = Patrol.Goal()

        # Target is chosen randomly inside the predefined field
        goal.target = Point(
            x = float(random.randint(-self.field_width/2, self.field_width/2 )),
            y = float(random.randint(-self.field_height/2, self.field_height/2 )),
            z = 0.0
        )

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.agent_action_clients[agent_id].send_goal_async(goal)
        patrol_future.add_done_callback(
            lambda future, agent_id=agent_id: self._random_walk_submitted_callback(agent_id, future)
        )


    def _random_walk_submitted_callback(self, agent_id, future):

        """
            Internal method. Used to handle the patrol behavior.
        """

        # Check if the patrol action was accepted
        goal_handle = future.result()

        if not goal_handle.accepted:
            # If not, set the sensor back to sensing, and return
            self.agent_states[agent_id] = SensorState.SENSING
            return

        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(
            lambda future, agent_id=agent_id: self._random_walk_completed_callback(agent_id, future)
        )


    def _random_walk_completed_callback(self, edge_device_id, future):

        self.agent_states[edge_device_id] = SensorState.SENSING


class SensorState(Enum):
    IDLE = 1
    SENSING = 2
    MOVING = 3


def main():

    rclpy.init()

    executor = MultiThreadedExecutor(64)
    agent_network_manager = AgentNetworkManager()

    executor.add_node(agent_network_manager)
    agent_network_manager.random_walk()

    executor.spin()

    executor.shutdown()
    agent_network_manager.destroy_node()
    rclpy.shutdown()
