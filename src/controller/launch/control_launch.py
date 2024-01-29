import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import time

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
        package='car_driver_localpkg',
        executable='drive_node',
        name='drive_node',
        output='screen'
    )

    delay_launch_car_driver = TimerAction(
        period=1.0,
        actions=[
            car_driver_node
        ]
    )

    return LaunchDescription([
        set_domain,
        game_pad_node,
        # key_board_node, # TODO : add key_board function to realize mode change
        mux_node,
        # insert here for one sec waiting
        # car_driver_node # will cause problem, the driver node will not receive enough message and break out
        delay_launch_car_driver # delay 1.0s to start driver node
    ])