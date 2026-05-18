from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from isac_libs_main.utils.launch_utils import (
    create_isac_device_launch_description,
    spawn_sdf,
    create_gazebo_launch_description
)

WORLD_NAME = "cantabria_world"


def generate_agents(context, *args, **kwargs):

    num_agents = int(LaunchConfiguration('agents').perform(context))

    nodes = []

    nodes.append(create_gazebo_launch_description(WORLD_NAME))

    for i in range(0, num_agents):

        # ----------------------------
        # agent_controller
        # ----------------------------
        nodes.extend(
            create_isac_device_launch_description(
                package="isac_libs_uc_example",
                executable="agent_controller",
                namespace=f"Agent_{i}",
                parameters=[{"id": i}],
            )
        )

        # ----------------------------
        # sdf spawn
        # ----------------------------
        nodes.append(
            spawn_sdf(
                "agent_uc",
                i,
                (i%6, int(i/6), 0),
                WORLD_NAME
            )
        )

    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'agents',
            default_value='2',
            description='Number of agents to spawn'
        ),

        OpaqueFunction(function=generate_agents)
    ])