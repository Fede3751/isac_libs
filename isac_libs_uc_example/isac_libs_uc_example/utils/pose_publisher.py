import random
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose


# ============================================================
# CONFIGURATION SECTION
# ============================================================

AGENTS = 4

# Random spawn area
X_MIN = 0.0
X_MAX = 6.0

Y_MIN = 0.0
Y_MAX = 6.0

Z_POSITION = 0.0

# Seconds between updates
PUBLISH_INTERVAL = 1.0

# ============================================================


class PosePublisher(Node):

    def __init__(self):
        super().__init__("random_pose_publisher")

        self.publishers_dict = {}

        # Create one publisher per agent
        for agent_idx in range(AGENTS):


            publisher = self.create_publisher(
                Pose,
                f"/Agent_{agent_idx}/set_pose",
                10
            )

            self.publishers_dict[agent_idx] = publisher



    def set_agent_pose(self, agent_id : int, pose : Pose):

        self.publishers_dict[agent_id].publish(pose)

        self.get_logger().info(
            f"Agent_{agent_id} -> "
            f"x={pose.position.x:.2f}, "
            f"y={pose.position.y:.2f}, "
            f"z={pose.position.z:.2f}"
        )





def create_random_pose():

    pose = Pose()

    # Random position
    pose.position.x = random.uniform(X_MIN, X_MAX)
    pose.position.y = random.uniform(Y_MIN, Y_MAX)
    pose.position.z = Z_POSITION

    # Neutral orientation
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    return pose



def main(args=None):

    rclpy.init(args=args)

    pose_publisher = PosePublisher()

    try:

        while rclpy.ok():


            for agent_idx in range(AGENTS):

            # Inject here your coordinates, and pass them as an argument
            # instead of using random coords.

                pose = create_random_pose()
                
                pose_publisher.set_agent_pose(agent_idx, pose)


            rclpy.spin_once(pose_publisher, timeout_sec=0.1)
            time.sleep(PUBLISH_INTERVAL)

    except KeyboardInterrupt:
        pass

    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()