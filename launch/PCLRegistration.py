import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('template_matching3d'),
        'config',
        'PCLRegParams.yaml'
        )
        
    node=Node(
        package="template_matching3d",
        executable="PCLRegistration",
        name="PCLRegistration",
        output="screen",
        parameters = [config]
    )
    ld.add_action(node)


    node=Node(
        package="template_matching3d",
        executable="TabletopSegmentation",
        name="TabletopSegmentation",
        output="screen",
        parameters = [config]
    )
    ld.add_action(node)
    return ld