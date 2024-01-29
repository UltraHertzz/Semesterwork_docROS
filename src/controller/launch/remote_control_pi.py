"""
In this launch file, we install controller driver on a remote pc instead of direct on developing board.
This is the launch file for developing board as well as the driver like mux, car driver.
"""

import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    
    # Set ROS domain ID to 30
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')

    game_pad_node = Node(
        package='controller',
        executable='game_pad_node',
        name = 'game_pad_node',
        output = 'screen'
    )
    key_board_node = Node(
        package='controller',
        executable='key_board_node',
        name = 'key_board_node',
        output = 'screen'
    )

    mux_node = Node(
        package='mux',
        executable='mux_node',
        name='mux_node',
        output='screen'
    )

    car_driver_node = Node(
        package='car_driver',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )

    return LaunchDescription([
        set_domain,
        # game_pad_node,
        # key_board_node,
        mux_node,
        car_driver_node
    ])