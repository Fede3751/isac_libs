from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from isac_libs_main.utils.launch_utils import create_gazebo_launch_description


"""
    Main launch file for the toy demo example.
    This file can be used as a reference to see how to launch an ISAC simulation.

    The main launch branches in three different launch files:
    -managers.launch: handles managers and global classes launched only once
    -devices.launch: handles the launch of all the ISAC devices
    -display.launch: handles the launch of a custom 2D interface

"""


main_pkg_name = "isac_libs_mec_example"
display_pkg_name = "isac_libs_display"

def generate_launch_description():

    pkg_main = FindPackageShare(main_pkg_name)
    pkg_gui_display = FindPackageShare(display_pkg_name)

    # ---------------------------------------
    # --- Launch arguments configuration ----
    #----------------------------------------

    world_name = LaunchConfiguration("world_name", default="empty")
    out_path = LaunchConfiguration("out_path", default="test_results")

    edge_devices = LaunchConfiguration("edge_devices", default="3")
    relays = LaunchConfiguration("relays", default="3")
    agents = LaunchConfiguration("agents", default="1")
    agent_speed = LaunchConfiguration("agent_speed", default="1")

    field_width = LaunchConfiguration("field_width", default="100")
    field_height = LaunchConfiguration("field_height", default="100")

    cache_size = LaunchConfiguration("cache_size", default="10")
    focus = LaunchConfiguration("focus", default="1.0")
    query_rate = LaunchConfiguration("query_rate", default="10")



    world_name_launch_arg = DeclareLaunchArgument(
        "world_name", default_value="empty", description="Gazebo world to load"
    )
    out_path_launch_arg = DeclareLaunchArgument(
        "out_path", default_value="output_tests", description="Output folder for the results"
    )    


    edge_devices_launch_arg = DeclareLaunchArgument(
        "edge_devices", default_value="3", description="Number of edge devices"
    )
    relays_launch_arg = DeclareLaunchArgument(
        "relays", default_value="3", description="Number of antenna relays"
    )
    agents_launch_arg = DeclareLaunchArgument(
        "agents", default_value="1", description="Number of agents on the field"
    )
    agent_speed_launch_arg = DeclareLaunchArgument(
        "agent_speed", default_value="1", description="Speed of the agent in the simulation"
    )



    field_width_launch_arg = DeclareLaunchArgument(
        "field_width", default_value="100", description="Field width"
    )
    field_height_launch_arg = DeclareLaunchArgument(
        "field_height", default_value="100", description="Field height"
    )


    cache_size_launch_arg = DeclareLaunchArgument(
        "cache_size", default_value="10", description="Edge Device cache size"
    )
    cache_expiration_launch_arg = DeclareLaunchArgument(
        "cache_expiration", default_value="10", description="Edge Device cache expiration seconds"
    )
    query_rate_launch_arg = DeclareLaunchArgument(
        "query_rate", default_value="10", description="Base Station query rate (seconds)"
    )
    focus_launch_arg = DeclareLaunchArgument(
        "focus", default_value="1.0", description="Focus lambda value"
    )



    # ---------------------------------------
    # ----- Single launch instructions ------
    #----------------------------------------

    gazebo_launch_include = create_gazebo_launch_description(world_name)

    managers_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_main, "launch", "managers.launch.py"])
        ),
        launch_arguments={
            "edge_devices": edge_devices,
            "relays": relays,
            "agents": agents,
            "focus": focus,
            "field_width": field_width,
            "field_height": field_height,
            "cache_size": cache_size,
            "query_rate": query_rate,
            "agent_speed": agent_speed,
            "out_path": out_path,
            "world_name": world_name
        }.items(),
    )

    devices_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_main, "launch", "devices.launch.py"])
        ),
        launch_arguments={
            "world_name": world_name,
            "edge_devices": edge_devices,
            "field_width": field_width,
            "field_height": field_height,
            "relays": relays,
            "agents": agents,
            "focus": focus,
            "cache_size": cache_size,
            "query_rate": query_rate,
            "agent_speed": agent_speed,
            "out_path": out_path
        }.items(),
    )


    # 2d display view is currently not maintained. It may come back in future versions

    # display_launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([pkg_gui_display, "launch", "display.launch.py"])
    #     ),
    #     launch_arguments={
    #         "edge_devices": edge_devices,
    #         "relays": relays,
    #         "agents": agents,
    #         "focus": focus,
    #         "field_width": field_width,
    #         "field_height": field_height,
    #         "cache_type": cache_type,
    #         "cache_size": cache_size,
    #         "query_rate": query_rate,
    #         "agent_speed": agent_speed,
    #         "out_path": out_path
    #     }.items(),
    # )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(edge_devices_launch_arg)
    ld.add_action(relays_launch_arg)
    ld.add_action(agents_launch_arg)
    ld.add_action(focus_launch_arg)
    ld.add_action(field_width_launch_arg)
    ld.add_action(field_height_launch_arg)
    ld.add_action(cache_size_launch_arg)
    ld.add_action(cache_expiration_launch_arg)
    ld.add_action(query_rate_launch_arg)
    ld.add_action(agent_speed_launch_arg)
    ld.add_action(out_path_launch_arg)
    ld.add_action(world_name_launch_arg)


    ld.add_action(devices_launch_include)
    ld.add_action(managers_launch_include)
    ld.add_action(gazebo_launch_include)    
    # ld.add_action(display_launch_include)
    

    return ld
