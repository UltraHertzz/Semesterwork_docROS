import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the package where the launch file is located
    package_name = 'realsense_camera'  # Replace with your package name
    package_dir = get_package_share_directory(package_name)

    # Path to the launch file
    other_launch_file = os.path.join(package_dir, 'launch', 'rs_launch.py')

    # Include the other launch file
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(other_launch_file),
        # launch_arguments={'arg_name': 'arg_value'}.items()  # Optional: Arguments to pass
    )
    yolo_node = Node(
        package='yolo',
        executable='yolo_node.py',
        name='yolo'
    )

    return LaunchDescription([
        included_launch,
        # Add other launch actions if needed
        yolo_node
    ])
