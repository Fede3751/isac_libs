from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



main_pkg_name = "isac_libs_toy_example"
display_pkg_name = "isac_libs_display"
description_pkg_name ="isac_libs_resources"





MAX_EDGE_DISTANCE = 50
RELAYS_MARGIN_RANGE = 5


SENSOR_MARGIN_RANGE = 5

def _setup_nodes(context: LaunchContext):
    field_width = int(LaunchConfiguration("field_width").perform(context))
    field_height = int(LaunchConfiguration("field_height").perform(context))
    edge_devices = int(LaunchConfiguration("edge_devices").perform(context))
    relays = int(LaunchConfiguration("relays").perform(context))
    agents = int(LaunchConfiguration("agents").perform(context))
    cache_size = int(LaunchConfiguration("cache_size").perform(context))
    cache_expiration = int(LaunchConfiguration("cache_expiration").perform(context))
    query_rate = int(LaunchConfiguration("query_rate").perform(context))
    out_path = str(LaunchConfiguration("out_path").perform(context))
    world_name = str(LaunchConfiguration("world_name").perform(context))


    return [
        Node(
            package=main_pkg_name,
            executable="agent_network_manager",
            parameters=[
                {"edge_devices": edge_devices},
                {"relays": relays},
                {"agents": agents},
                {"field_width": field_width},
                {"field_height": field_height},
            ],
        ),
        Node(
            package=main_pkg_name,
            executable="application_controller",
            parameters=[
                {"edge_devices": edge_devices},
                {"relays": relays},
                {"agents": agents},
                {"cache_size": cache_size},
                {"cache_expiration": cache_expiration},
                {"query_rate": query_rate},
                {"out_path": out_path}
            ],
        )
    ]



def generate_launch_description():
    # Declare the launch arguments
    edge_devices_launch_arg = DeclareLaunchArgument(
        "edge_devices", default_value="3", description="Number of edge devices"
    )
    sensors_launch_arg = DeclareLaunchArgument(
        "sensors", default_value="3", description="Number of sensors"
    )
    sensors_range_launch_arg = DeclareLaunchArgument(
        "sensors_range", default_value="50", description="Range of sensors"
    )
    agents_launch_arg = DeclareLaunchArgument(
        "agents", default_value="1", description="Number of agents"
    )

    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(edge_devices_launch_arg)
    ld.add_action(sensors_launch_arg)
    ld.add_action(sensors_range_launch_arg)
    ld.add_action(agents_launch_arg)

    # Add the opaque function to the launch description
    ld.add_action(OpaqueFunction(function=_setup_nodes))

    return ld
