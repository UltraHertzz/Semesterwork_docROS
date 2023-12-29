from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='rs_cam'
    )
    yolo_node = Node(
        package='yolo',
        executable='yolo_node',
        name='yolo'
    )
    
    return LaunchDescription([
        cam_node,
        yolo_node  
    ])