from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def load_tf_nodes(tf_file, prefix=""):
    tf_path = os.path.join(os.path.dirname(__file__), '..', 'config', tf_file)
    with open(tf_path, 'r') as f:
        tf_data = yaml.safe_load(f)

    tf_nodes = []
    for tf in tf_data['transforms']:
        translation = tf['translation']
        rotation = tf['rotation']
        parent = tf['parent']
        child = tf['child']
        args = [*map(str, translation), *map(str, rotation), parent, child]

        tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{prefix}static_tf_{parent}_to_{child}',
                arguments=args,
                output='screen'
            )
        )
    return tf_nodes

def generate_launch_description():
    tf_nodes_1 = load_tf_nodes('transforms.yaml', prefix='a_')
    tf_nodes_2 = load_tf_nodes('transforms2.yaml', prefix='b_')

    icp_odom_1 = Node(
        package='rtabmap_ros',
        executable='icp_odometry',
        name='icp_odometry_1',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'Reg/Strategy': 1,
            'use_sim_time': False
        }]
    )

    icp_odom_2 = Node(
        package='rtabmap_ros',
        executable='icp_odometry',
        name='icp_odometry_2',
        parameters=[{
            'frame_id': 'base_link2',
            'odom_frame_id': 'odom2',
            'scan_topic': '/scan2',
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'Reg/Strategy': 1,
            'use_sim_time': False
        }]
    )

    rtabmap_1 = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap_1',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            'rgb_topic': '/kinect2/qhd/image_color_rect',
            'depth_topic': '/kinect2/qhd/image_depth_rect',
            'camera_info_topic': '/kinect2/qhd/camera_info',
            'approx_sync': True,
            'rgbd_sync': True,
            'Reg/Strategy': 1,
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'database_path': os.path.expanduser('~/.ros/rtabmap1.db'),
            'subscribe_depth': True,
            'subscribe_scan': True,
            'Grid/FromDepth': True,
            'Grid/RangeMax': 10.0,
            'use_sim_time': False
        }]
    )

    rtabmap_2 = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap_2',
        output='screen',
        parameters=[{
            'frame_id': 'base_link2',
            'odom_frame_id': 'odom2',
            'scan_topic': '/scan2',
            'rgb_topic': '/kinect1/qhd/image_color_rect',
            'depth_topic': '/kinect1/qhd/image_depth_rect',
            'camera_info_topic': '/kinect1/qhd/camera_info',
            'approx_sync': True,
            'rgbd_sync': True,
            'Reg/Strategy': 1,
            'Odom/MaxTranslation': 0.25,
            'Odom/MaxRotation': 0.8,
            'database_path': os.path.expanduser('~/.ros/rtabmap2.db'),
            'subscribe_depth': True,
            'subscribe_scan': True,
            'Grid/FromDepth': True,
            'Grid/RangeMax': 10.0,
            'use_sim_time': False
        }],
        remappings=[
            ('/map', '/map2'),
            ('/map_graph', '/map_graph2'),
            ('/map_data', '/map_data2')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription(
        tf_nodes_1 + tf_nodes_2 +
        [icp_odom_1, icp_odom_2, rtabmap_1, rtabmap_2, rviz_node]
    )
