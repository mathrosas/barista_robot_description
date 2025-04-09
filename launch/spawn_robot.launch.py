#!/usr/bin/python3
# -*- coding: utf-8 -*-
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.5, 0.01]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "barista_robot"

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   robot_base_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    spawn_controller_velocity = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            spawn_robot,
            spawn_controller_velocity
        ]
    )