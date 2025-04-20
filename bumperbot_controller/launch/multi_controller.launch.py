#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()
    
    bumperbot_description = get_package_share_directory("bumperbot_description")

    # Common arguments
    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty"
    )
    ld.add_action(world_name_arg)

    # Path configurations
    world_path = PathJoinSubstitution([
        bumperbot_description,
        "worlds",
        PythonExpression(["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])

    model_path = os.path.join(bumperbot_description, 'models')
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )
    ld.add_action(gazebo_resource_path)
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            )
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )
    ld.add_action(gazebo)

    # Robot configurations - each with different URDF and namespace
    robots = [
        {
            'name': 'bumperbot1',
            'namespace': 'amr1',
            'x': '0.0',
            'y': '0.0',
            'urdf': os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro")
        },
        {
            'name': 'bumperbot2',
            'namespace': 'amr2',
            'x': '2.0',
            'y': '0.0',
            'urdf': os.path.join(bumperbot_description, "urdf", "bumperbot_2.urdf.xacro")
        }
    ]

    # Add a small delay after Gazebo starts
    delay_after_gazebo = TimerAction(period=3.0, actions=[])
    ld.add_action(delay_after_gazebo)
    last_action = delay_after_gazebo

    for robot in robots:
        robot_description = ParameterValue(
            Command([
                "xacro ", 
                robot['urdf'],
                " is_ignition:=", is_ignition,
                " namespace:=", robot['namespace']
            ]),
            value_type=str
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot['namespace'],
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
                "frame_prefix": f"{robot['namespace']}/"
            }],
            output="screen"
        )

        spawn_entity = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", f"/{robot['namespace']}/robot_description",
                "-name", robot['name'],
                "-x", robot['x'],
                "-y", robot['y'],
                "-z", "0.1"
            ],
            output="screen"
        )

        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"/{robot['namespace']}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"/{robot['namespace']}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                f"/{robot['namespace']}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
            ],
            remappings=[
                (f'/{robot["namespace"]}/imu', f'/{robot["namespace"]}/imu/out'),
            ],
            output="screen"
        )

        # Chain the actions properly
        ld.add_action(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_state_publisher]
                )
            )
        )
        ld.add_action(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_state_publisher,
                    on_exit=[spawn_entity]
                )
            )
        )
        ld.add_action(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[bridge]
                )
            )
        )
        
        last_action = spawn_entity

    return ld