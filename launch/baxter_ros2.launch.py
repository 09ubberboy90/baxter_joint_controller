import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    controller = Node(package="baxter_joint_controller",
                         executable="controller",)

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("baxter_bridge"), 'launch', 'baxter_bridge_launch.py'),
        ),)
    ros_ip_env = SetEnvironmentVariable("ROS_IP", "130.209.242.9")
    ros_master_env = SetEnvironmentVariable("ROS_MASTER_URI", "http://cornwall:11311")

    ld.add_action(ros_ip_env)
    ld.add_action(ros_master_env)
    ld.add_action(bridge)
    ld.add_action(TimerAction(
            period=5.,
            actions=[
                controller,
            ]))

    return ld
