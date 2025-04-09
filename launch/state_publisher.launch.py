import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = 'barista_robot_model.urdf'
    # xacro_file = 'tb3_burger.urdf.xacro'
    package_description = "barista_robot_description"

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "urdf", urdf_file
        # get_package_share_directory(package_description), "xacro", xacro_file
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # Joint State Publisher GUI node
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )

    return LaunchDescription([
        # joint_state_publisher_gui_node,
        robot_state_publisher_node
    ])