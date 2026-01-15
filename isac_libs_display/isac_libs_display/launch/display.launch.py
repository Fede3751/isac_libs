from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


main_pkg_name = "isac_libs_main"
display_pkg_name = "isac_libs_display"
description_pkg_name ="isac_libs_resources"


def _setup_screen(context):

    pkg_project_gazebo = FindPackageShare("project_gazebo")

    field_width = int(LaunchConfiguration("field_width").perform(context))
    field_height = int(LaunchConfiguration("field_height").perform(context))
    edge_devices = int(LaunchConfiguration("edge_devices").perform(context))
    sensors = int(LaunchConfiguration("sensors").perform(context))
    agents = int(LaunchConfiguration("agents").perform(context))
    sensors_range = int(LaunchConfiguration("sensors_range").perform(context))
    cache_type = LaunchConfiguration("cache_type").perform(context)
    cache_size = int(LaunchConfiguration("cache_size").perform(context))
    cache_expiration = int(LaunchConfiguration("cache_expiration").perform(context))
    query_rate = int(LaunchConfiguration("query_rate").perform(context))




    nodes = [
        Node(
            package=display_pkg_name,
            executable="display_controller",
            parameters=[
                {"edge_devices": edge_devices},
                {"sensors": sensors},
                {"agents": agents}
            ]
        )
    ]
    
    # for s in range(sensors):
    #     nodes.append(Node(
    #         package="project_viewer",
    #         executable="sensor",
    #         namespace=f"ActiveSensor_{s}"
    #     )
    #     )

    return nodes


def generate_launch_description():


    # Define the robot namespaces
    # robot_namespaces = [f'ActiveSensor_{i}' for i in range(NO_SENSORS)]

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=_setup_screen))

    print(ld.entities)

    # return LaunchDescription([
    #     # Launch multiple MovementSimulator nodes with namespaces using a loop
    #     *[Node(
    #         package='project_viewer',
    #         executable='sensor',
    #         namespace=ns,
    #         name='active_sensor'
    #     ) for ns in robot_namespaces],

    #     # Launch the display node
    #     Node(
    #         package='project_viewer',  # Replace with your package name
    #         executable='display',
    #         name='display'
    #     )
    # ])

    return ld
