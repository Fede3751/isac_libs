from launch import LaunchDescription

from isac_libs_main.utils.launch_utils import create_isac_device_launch_description, spawn_sdf, create_gazebo_launch_description
from launch_ros.actions import Node

WORLD_NAME = "sensing_world"


def generate_launch_description():

    nodes = []

    nodes.append(create_gazebo_launch_description(WORLD_NAME))

    # Spawn wall in the middle of the simulation
    nodes.append(spawn_sdf("wall", 0, (0,0,0), WORLD_NAME))


    # Spawn the two users
    nodes.extend(create_isac_device_launch_description(
            package="isac_libs_sensing_example",
            executable="agent_controller",
            namespace=f"Agent_1",
            parameters=[
            ],
        ))
    
    nodes.append(spawn_sdf("agent_wifi", 1, (3, -10, 0), WORLD_NAME))


    # Spawn the second users
    nodes.extend(create_isac_device_launch_description(
            package="isac_libs_sensing_example",
            executable="agent_controller",
            namespace=f"Agent_2",
            parameters=[
            ],
        ))
    
    nodes.append(spawn_sdf("agent_wifi", 2, (-3, -10, 0), WORLD_NAME))


    nodes.append(
        Node(
            package="isac_libs_sensing_example",
            executable="isac_signal_display"
        )
    )


    return LaunchDescription(nodes)

    