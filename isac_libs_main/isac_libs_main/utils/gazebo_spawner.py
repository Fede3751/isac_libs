import os
import re
from typing import Tuple

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity


class GazeboSpawner(Node):
    """
    Spawns models once and shuts down immediately.
    """

    def __init__(self):
        super().__init__("gazebo_spawner")

        # Parameters
        self.declare_parameter("world_name", "empty")
        self.declare_parameter("models", "")

        self.world_name = self.get_parameter("world_name").value
        self.models = self.get_parameter("models").value

        self.get_logger().info(self.models)

        if not self.models:
            self.get_logger().warn("No models specified to spawn. Shutting down.")
            rclpy.shutdown()
            return

        self.spawn_client = self.create_client(
            SpawnEntity, f"/world/{self.world_name}/create"
        )



        while not self.spawn_client.wait_for_service(3):
            self.get_logger().info(
                f"Waiting for Gazebo service /world/{self.world_name}/create..."
            )
            
            self.spawn_client = self.create_client(
                SpawnEntity, f"/world/{self.world_name}/create"
            )
            
            self.get_logger().info("Gazebo ready. Spawning models...")

        # Spawn everything
        self.spawn_all_models()

        # Once done, shutdown
        self.get_logger().info("All models spawned. Shutting down spawner node.")
        rclpy.shutdown()

    # ------------------------------------------------
    def spawn_all_models(self):
        for model in self.models:
            try:
                self.spawn_model(**model)
            except Exception as e:
                self.get_logger().error(str(e))

    # ------------------------------------------------
    def spawn_model(
        self,
        sdf_path: str,
        name: str | None = None,
        position: Tuple[float, float, float] = (0, 0, 0),
        external: bool = False,
    ):
        """
        Spawn a single model.
        """

        if not external:
            sdf_path = os.path.join(
                os.environ["PKG_ISAC_LIBS_RESOURCES"], "models", sdf_path, "model.sdf"
            )

        if not os.path.exists(sdf_path):
            raise FileNotFoundError(sdf_path)

        with open(sdf_path) as f:
            sdf_string = f.read()

        # Parse original model name
        match = re.search(r"<model\s+name=['\"]([^'\"]+)['\"]", sdf_string)
        if not match:
            raise RuntimeError("Could not parse model name from SDF")

        original_name = match.group(1)

        if name is None:
            name = original_name
        else:
            sdf_string = sdf_string.replace(original_name, name)

        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf_string
        request.robot_namespace = name
        request.reference_frame = "world"

        request.initial_pose.position.x = float(position[0])
        request.initial_pose.position.y = float(position[1])
        request.initial_pose.position.z = float(position[2])

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"Failed to spawn {name}")
        else:
            self.get_logger().info(f"Spawned {name}")


# ---------------------------
def main(args=None):
    rclpy.init(args=args)
    GazeboSpawner()
    # Node shuts down itself, so no need to spin here
