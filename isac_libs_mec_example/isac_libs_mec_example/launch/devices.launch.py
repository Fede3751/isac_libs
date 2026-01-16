import os
import random

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from isac_libs_main.utils.launch_utils import (
    spawn_sdf,
    create_isac_device_launch_description,
)

from isac_libs_mec_example.utils.toy_example_utils import (
    base_station_placement,
    edge_device_placement,
    compute_area_coverage,
    compute_random_covered_positions,
)


main_pkg_name = "isac_libs_toy_example"
display_pkg_name = "isac_libs_display"
resources_pkg_name ="isac_libs_resources"

MAX_EDGE_DISTANCE = 50
RELAYS_MARGIN_RANGE = 5

def _setup_robots(context: LaunchContext, *args, **kwargs):
    
    edge_device_sdf_file = "edge_device"
    agent_sdf_file = "agent"
    antenna_relay_file = "antenna_relay"

    world_name = LaunchConfiguration("world_name").perform(context)

    field_width = int(LaunchConfiguration("field_width").perform(context))
    field_height = int(LaunchConfiguration("field_height").perform(context))

    edge_devices = int(LaunchConfiguration("edge_devices").perform(context))
    cache_size = int(LaunchConfiguration("cache_size").perform(context))
    cache_expiration = int(LaunchConfiguration("cache_expiration").perform(context))

    relays = int(LaunchConfiguration("relays").perform(context))
    
    agents = int(LaunchConfiguration("agents").perform(context))
    focus = float(LaunchConfiguration("focus").perform(context))
    agent_speed = int(LaunchConfiguration("agent_speed").perform(context))

    # Compute the initial placement of the edge devices.
    edge_devices_coords = edge_device_placement(
        edge_devices, field_width, field_height, MAX_EDGE_DISTANCE - RELAYS_MARGIN_RANGE
    )

    # Compute the area coverage of the field.
    grid_coverage, grid_x_coords, grid_y_coords = compute_area_coverage(
        field_width, field_height, edge_devices_coords, MAX_EDGE_DISTANCE
    )

    # Compute the initial placement of the sensors.
    relays_coords = compute_random_covered_positions(
        relays, grid_coverage, grid_x_coords, grid_y_coords
    )

    # Define the list of nodes to be spawned.
    nodes = []

    bridge_args = []
    
    # Spawn the edge devices and their related topics (cmd_vel, odometry)
    for i, coord in enumerate(edge_devices_coords):

        device, bridges = create_isac_device_launch_description(
            package="isac_libs_toy_example",
            executable="edge_device_controller",
            namespace=f"EdgeDevice_{i}",
            parameters=[
                        {"start_x": coord[0]},
                        {"start_y": coord[1]},
                        {"id": i},
                        {"cache_size": cache_size},
                        {"cache_expiration": cache_expiration},
                        {"relays": relays},
                        {"agents": agents},
                        {"focus": focus}
                    ],
            return_node_for_bridges=False
        )

        nodes.append(device)
        nodes.append(spawn_sdf(edge_device_sdf_file, i, coord, world_name))

        bridge_args.extend(bridges)



    for i, coord in enumerate(relays_coords):

        coord = [random.randint(-field_width*2, field_width*2), random.randint(-field_height*2, field_height*2), 0]

        device, bridges = create_isac_device_launch_description(
            package="isac_libs_toy_example",
            executable="relay_device_controller",
            namespace=f"AntennaRelay_{i}",
            parameters=[
                    {"id": i},
                    {"agents": agents}
            ],
            return_node_for_bridges=False
        )

        nodes.append(device)
        nodes.append(spawn_sdf(antenna_relay_file, i, coord, world_name))

        bridge_args.extend(bridges)
            


    for i in range(agents):


        random_position = [random.randint(-field_width, field_width), random.randint(-field_height, field_height), 0]

        device, bridges = create_isac_device_launch_description(
            package="isac_libs_toy_example",
            executable="agent_controller",
            namespace=f"Agent_{i}",
            parameters=[
                    {"id": i},
                    {"agent_speed": agent_speed}
            ],
            return_node_for_bridges=False
        )
        
        nodes.append(device)
        nodes.append(spawn_sdf(agent_sdf_file, i, (random_position[0], random_position[1], 0), world_name))

        bridge_args.extend(bridges)


    nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=bridge_args,
                output="screen",
            )
    )

    return nodes


def generate_launch_description():


    edge_devices_launch_arg = DeclareLaunchArgument(
        "edge_devices", default_value="3", description="Number of edge devices"
    )

    relays_launch_arg = DeclareLaunchArgument(
        "relays", default_value="3", description="Number of antenna relays"
    )


    field_width_launch_arg = DeclareLaunchArgument(
        "field_width", default_value="100", description="Field width"
    )

    field_height_launch_arg = DeclareLaunchArgument(
        "field_height", default_value="100", description="Field height"
    )

    cache_size_launch_arg = DeclareLaunchArgument(
        "cache_size", default_value="10", description="Edge device cache size"
    )

    cache_expiration_launch_arg = DeclareLaunchArgument(
        "cache_expiration", default_value="10", description="Edge device cache size"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(edge_devices_launch_arg)
    ld.add_action(relays_launch_arg)
    ld.add_action(field_width_launch_arg)
    ld.add_action(field_height_launch_arg)
    ld.add_action(cache_size_launch_arg)
    ld.add_action(cache_expiration_launch_arg)

    # Add the opaque function for setting up the robots
    ld.add_action(OpaqueFunction(function=_setup_robots))

    return ld
