import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(
            bumperbot_description, "urdf", "bumperbot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )
    
    model2_arg = DeclareLaunchArgument(
        name="model2", 
        default_value=os.path.join(
            bumperbot_description, "urdf", "bumperbot_2.urdf.xacro"
        ),
        description="Absolute path to second robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty"
    )

    world_path = PathJoinSubstitution([
            bumperbot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("bumperbot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    # Robot 1 description
    robot1_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " namespace:=bumperbot"
        ]),
        value_type=str
    )

    # Robot 2 description
    robot2_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model2"),
            " is_ignition:=",
            is_ignition,
            " namespace:=bumperbot_2"
        ]),
        value_type=str
    )

    # Robot 1 state publisher
    robot1_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot1_description,
            "use_sim_time": True,
            "frame_prefix": "bumperbot/"
        }],
        output="screen"
    )

    # Robot 2 state publisher
    robot2_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="bumperbot_2",
        parameters=[{
            "robot_description": robot2_description,
            "use_sim_time": True,
            "frame_prefix": "bumperbot_2/"
        }],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    # Spawn Robot 1
    gz_spawn_entity1 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "bumperbot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1"
        ],
    )

    # Spawn Robot 2
    gz_spawn_entity2 = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/bumperbot_2/robot_description",
            "-name", "bumperbot_2",
            "-x", "1.0",
            "-y", "0.0",
            "-z", "0.1"
        ],
    )

    # Controller manager for Robot 1
    controller_manager1 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot1_description},
            os.path.join(
                bumperbot_description, "config", "bumperbot_controllers.yaml"
            ),
            {"use_sim_time": True}
        ],
        output="screen",
    )
    
    # Controller manager for Robot 2
    controller_manager2 = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="bumperbot_2",
        parameters=[
            {"robot_description": robot2_description},
            os.path.join(
                bumperbot_description, "config", "bumperbot_controllers.yaml"
            ),
            {"use_sim_time": True}
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge for Robot 1
    gz_ros2_bridge_bot1 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/bumperbot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/bumperbot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/bumperbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
        ],
        remappings=[
            ('/bumperbot/imu', '/bumperbot/imu/data'),
            ('/bumperbot/scan', '/bumperbot/scan'),
        ],
        output="screen"
    )

    # ROS-Gazebo Bridge for Robot 2
    gz_ros2_bridge_bot2 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/bumperbot_2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/bumperbot_2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/bumperbot_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
        ],
        remappings=[
            ('/bumperbot_2/imu', '/bumperbot_2/imu/data'),
            ('/bumperbot_2/scan', '/bumperbot_2/scan'),
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        model2_arg,
        world_name_arg,
        gazebo_resource_path,
        robot1_state_publisher_node,
        robot2_state_publisher_node,
        controller_manager1,
        controller_manager2,
        gazebo,
        gz_spawn_entity1,
        gz_spawn_entity2,
        gz_ros2_bridge_bot1,
        gz_ros2_bridge_bot2
    ])