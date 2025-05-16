from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Kamera-Info-Publisher (Dummy)
        Node(
            package='kinect',
            executable='camera_info_publisher',
            name='camera_info_publisher'
        ),
        # statische TF: base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        # RGBD-Sync Node (optional für RTAB)
        Node(
            package='rtabmap_ros',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[{'approx_sync': True}],
            remappings=[
                ('rgb/image', '/kinect1/rgb'),
                ('depth/image', '/kinect1/depth'),
                ('rgb/camera_info', '/kinect1/camera_info')
            ]
        ),
        # RTAB-Map-Kern
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgbd': True,
                'visual_odometry': True,
                'approx_sync': True
            }]
        ),
        # RTAB-Map GUI
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            parameters=[{'frame_id': 'base_link'}]
        )
    ])
