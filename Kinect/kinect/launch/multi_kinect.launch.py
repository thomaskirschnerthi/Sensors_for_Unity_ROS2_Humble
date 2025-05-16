from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinect',
            executable='kinect_receiver',
            name='kinect1',
            parameters=[
                {'kinect_id': 'kinect1'},
                {'depth_port': 5007},
                {'rgb_port': 5008},
            ]
        ),
        Node(
            package='kinect',
            executable='kinect_receiver',
            name='kinect2',
            parameters=[
                {'kinect_id': 'kinect2'},
                {'depth_port': 5009},
                {'rgb_port': 5010},
            ]
        ),
        
        Node(
            package='kinect',
            executable='kinect_receiver',
            name='kinect3',
            parameters=[
                {'kinect_id': 'kinect3'},
                {'depth_port': 5011},
                {'rgb_port': 5012},
            ]
        ),
        Node(
            package='kinect',
            executable='kinect_receiver',
            name='kinect4',
            parameters=[
                {'kinect_id': 'kinect4'},
                {'depth_port': 5013},
                {'rgb_port': 5014},
            ]
        )
    ])
