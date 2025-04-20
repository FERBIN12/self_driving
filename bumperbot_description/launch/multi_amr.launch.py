import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_robot_entities(robot_name, urdf_file_path, spawn_position, is_ignition):
    """
    Returns a list of launch actions for a single robot: state publisher and spawn entity
    """
    robot_namespace = robot_name
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file_path, " is_ignition:=", is_ignition]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen"
    )

    robot_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", robot_name,
            "-x", str(spawn_position[0]),
            "-y", str(spawn_position[1]),
            "-z", str(spawn_position[2]),
            "-topic", f"/{robot_namespace}/robot_description"
        ]
    )

    return [robot_state_publisher, robot_spawn]


def generate_launch_description():
    # Get package paths
    bumperbot_description = get_package_share_directory("bumperbot_description")
    model_dir = os.path.join(bumperbot_description, "urdf")

    # Robot URDFs
    robots = [
        {
            "name": "robot1",
            "urdf": os.path.join(model_dir, "bumperbot.urdf.xacro"),
            "spawn_position": [0, 0, 0.1],
        },
        {
            "name": "robot2",
            "urdf": os.path.join(model_dir, "bumperbot_2.urdf.xacro"),
            "spawn_position": [2, 0, 0.1],
        }
    ]

    # Set GZ_SIM_RESOURCE_PATH to find models
    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(bumperbot_description, 'models')
    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    # Get ROS version to check if Ignition is needed
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={
            "gz_args": "-r -v 4"
        }.items()
    )

    # Bridge for clock (you can add more topics later)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # Gather all robot launch actions
    robot_launch_actions = []
    for robot in robots:
        robot_launch_actions += generate_robot_entities(
            robot_name=robot["name"],
            urdf_file_path=robot["urdf"],
            spawn_position=robot["spawn_position"],
            is_ignition=is_ignition
        )

    # Final launch description
    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        gz_ros2_bridge,
        *robot_launch_actions
    ])
