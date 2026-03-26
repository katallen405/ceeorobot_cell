from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="130.64.17.95",
            description="IP address by which the robot can be reached.",
        ),
    ]

    # Start the robot driver, controllers, and RSP
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ceeorobot_control"),
                "launch",
                "start_robot.launch.py",
            ])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": "false",
        }.items(),
    )

    # Start MoveIt move_group
    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ceeorobot_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ]),
    )

    return LaunchDescription(declared_arguments + [robot_bringup, moveit_bringup])
