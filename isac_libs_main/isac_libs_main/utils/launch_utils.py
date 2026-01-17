import os
import math
import re
import numpy as np
import json
import xml.etree.ElementTree as ET
from typing import Callable, List, Dict, Any
import subprocess
from threading import Thread
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, GroupAction, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, PythonExpression


from launch_ros.actions import Node


PKG_ISAC_LIBS_RESOURCES = get_package_share_directory("isac_libs_resources")
PKG_ROS_GZ_SIM = get_package_share_directory("ros_gz_sim")



def create_gazebo_launch_description(world_name):

    world_file = PathJoinSubstitution([
        PKG_ISAC_LIBS_RESOURCES,
        "worlds",
        world_name
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG_ROS_GZ_SIM, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "--headless-rendering ",
                world_file
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_bridge_ctrl_world = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            PathJoinSubstitution([
                TextSubstitution(text="/world"),
                world_name,
                TextSubstitution(text="control]ros_gz_interfaces/srv/ControlWorld"),
            ])
        ],
    )

    gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            PathJoinSubstitution([
                TextSubstitution(text="/world"),
                world_name,
                TextSubstitution(text="clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"),
            ])
        ],
    )

    Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
            PathJoinSubstitution([
                TextSubstitution(text="/world"),
                world_name,
                TextSubstitution(text="/control@ros_gz_interfaces/srv/ControlWorld"),
            ])
            ]
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge_ctrl_world,
        gz_bridge_clock,
    ])

def create_isac_device_launch_description(package: str, executable: str, namespace: str = "", parameters: List[Dict[str, str]] = [{}], return_node_for_bridges = True):
    
    """
        Auxiliary function used to launch a class extending an ISAC device. It launches the class and the bridges used by an
        ISAC device all at once.

            package:
                the package of the class extending the ISAC device
            executable:
                the ROS executable associated with the class
            namespace:
                namespace to launch the ISAC device. This should be set to the id of the device being launched
            parameters:
                standard ROS parameters to pass to the ISAC device
            return_node_for_bridges:
                if set to False, bridges are returned as arguments for ros_gz_bridge, instead of a node. This is
                to be used when a lot of ISAC devices are spawned, as it some bridges may fail to be populated

            Return value:
                either two nodes, or nodes + a string array if return_node_for_bridges is set to False
    """

    actions = []

    # ISACDeviceController Node
    actions.append(
        Node(
            package=package,
            executable=executable,
            namespace=namespace,
            parameters=parameters,
            output="screen",
        )
    )

    ros_gz_bridge_arguments = [
                    f"/{namespace}/tx_data@std_msgs/msg/String]ignition.msgs.StringMsg",
                    f"/{namespace}/rx_data@std_msgs/msg/String[ignition.msgs.StringMsg",
                    f"/{namespace}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                    f"/{namespace}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
                ]

    if return_node_for_bridges:
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=namespace,
                arguments=ros_gz_bridge_arguments,
                output="screen",
            )
        )

        return actions

    return actions[0], ros_gz_bridge_arguments


def wait_for_ign_service(context, service_name: str, timeout: float = 30.0):
    """
    Busy-waits for an ignition service to appear.
    We use `ign service -l` to check.
    """


    start_time = time.time()
    while True:
        try:
            out = subprocess.check_output(["ign", "service", "-l"]).decode()
            if service_name in out:
                return
        except subprocess.CalledProcessError:
            pass

        if time.time() - start_time > timeout:
            raise RuntimeError(f"Service {service_name} didn't appear in time")

        time.sleep(0.1)




def spawn_sdf(sdf_input, id: int = None, pos: tuple = (0, 0, 0), world_name: str = "empty", external : bool = False):
    """
    Function used to dynamically spawn a new sdf object at the start of the simulation.
    Use this to spawn multiple times the same object with different ids in your launch file.
    This function additionally modifies all the occurrences of the model name inside
    the SDF, keeping plugins such as OdometryPublisher consistent even without namespaces set.
    All occurrences of [MODEL_NAME] inside the sdf will be replaced with [MODEL_NAME]_[ID].
    A position can additionally be specified to choose where to place the given model.

        sdf_input:
            the path to the sdf_file to spawn (relative to pwd or absolute)
        id:
            the id of the new model, should be different by all the ids of object with the same sdf
        pos:
            the position where to spawn the model
        world_name:
            the name of the world to spawn the model to
        external:
            set to True if the file is outside the package (default False)

    Models spawned with this function are expected to be found in separated folders in:
        src/isac_libs_resources/models/
    The function will look for the files compiled from:
        src/isac_libs_resources/models/[sdf_input]/model.sdf

    Alternatively, a relative/absolute path to the sdf can be given if the option external is True.
    """

    def spawn_sdf_inner(context):

        model_input = sdf_input

        if not external:
            model_input =  os.path.join(PKG_ISAC_LIBS_RESOURCES, "models", model_input, "model.sdf")

        with open(model_input) as sdf_file:
            sdf_string = sdf_file.read()

            model_name = None

            # use re to look for the model name and save it
            model_name_pattern = r"<model\s+name='([^']+)'"
            match = re.search(model_name_pattern, sdf_string)
            if match:
                model_name = match.group(1)

            if model_name is None:
                raise Exception(f"Error while trying to parse model name from file {sdf_input}")

            # replace all the occurrences of model_name with model_name_id,
            # and set the model name accordingly, if an id was given
            if id is None:
                model_name_with_id = model_name
            else:
                sdf_string = sdf_string.replace(model_name, f"{model_name}_{id}")
                model_name_with_id = f"{model_name}_{id}"

            return [Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", world_name,
                    "-string", sdf_string,
                    "-name", model_name_with_id,
                    "-x", str(pos[0]),
                    "-y", str(pos[1]),
                    "-z", str(pos[2]),
                ],
                output="screen",
            )]
        
    return TimerAction(period=5.0, actions=[OpaqueFunction(function=spawn_sdf_inner)])


# def spawn_sdf(
#     sdf_input: str,
#     id: int | None = None,
#     pos: tuple[float, float, float] = (0, 0, 0),
#     world_name: str = "empty",
#     namespace: str = "",
#     external: bool = False,
# ):
#     """
#     Create a Node that calls the one-shot gazebo spawner executable.
#     """

#     # Resolve SDF path
#     if not external:
#         sdf_input = os.path.join(PKG_ISAC_LIBS_RESOURCES, "models", sdf_input, "model.sdf")

#     # Read SDF file and parse model name
#     with open(sdf_input) as f:
#         sdf_string = f.read()

#     match = re.search(r"<model\s+name=['\"]([^'\"]+)['\"]", sdf_string)
#     if not match:
#         raise RuntimeError(f"Could not parse model name from {sdf_input}")
#     original_name = match.group(1)

#     # Build model name with id
#     model_name = f"{original_name}_{id}" if id is not None else original_name

#     # Build JSON config to pass as argument
#     config = {

        
#     }

#     # Save temporary JSON file or pass as string
#     file_config = json.dumps(config)

#     # Return Node
#     from launch_ros.actions import Node
#     return Node(
#         package="isac_libs_main",
#         executable="gazebo_spawner",
#         namespace=namespace,
#         parameters=[{
#             "world_name":   world_name,
#             "models":       json.dumps({
#                                 "sdf_path": sdf_input,
#                                 "name": model_name,
#                                 "position": pos,
#                                 "external": external,  # full path
#                             })
#             }],
#         output="screen",
#     )


