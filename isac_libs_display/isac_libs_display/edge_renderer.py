import math
import random

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry



class EdgeRenderer(Node):
    def __init__(self):
        super().__init__('edge_device')

        self.declare_parameters(
            namespace="",
            parameters=[
                ("start_x", 0),
                ("start_y", 0)
            ]
        )


        self.start_x = (
            self.get_parameter("start_x").get_parameter_value().integer_value
        )
        self.start_y = (
            self.get_parameter("start_y").get_parameter_value().integer_value
        )


        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            'odometry',
            10
        )


        # Initial position and orientation
        if self.start_x == 0 and self.start_y == 0:

            self.x = float(random.randint(-10, 10))
            self.y = float(random.randint(-10,10))

        else:
            self.x = self.start_x
            self.y = self.start_y

        self.theta = 0.0  # Orientation in radians

        # Update rate for position calculations
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.update_position)

        # Store linear and angular velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


        self.get_logger().info("Starting edge device!")

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel topic to update velocities."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_position(self):
        """Update the robot's position based on the velocities."""
        dt = self.timer_period

        # Update orientation
        self.theta += self.angular_velocity * dt
        self.theta = math.fmod(self.theta, 2 * math.pi)  # Normalize to [0, 2*pi)

        # Update position
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt

        odom_msg = Odometry()
        odom_msg.pose.pose.position = Point(x=self.x, y=self.y)
        self.odom_publisher.publish(odom_msg)



def main(args=None):

    rclpy.init(args=args)

    edgeDeviceRenderer = EdgeRenderer()

    try:
        rclpy.spin(edgeDeviceRenderer)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    edgeDeviceRenderer.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()

