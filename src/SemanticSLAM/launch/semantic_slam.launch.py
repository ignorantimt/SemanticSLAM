from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'semanticSLAM'
    package_dir = get_package_share_directory(package_name)

    return LaunchDescription([
        Node(
            package=package_name,
            executable='semantic_ros_rgbd',
            arguments=[os.path.join(package_dir, 'Vocabulary/ORBvoc.bin'), os.path.join(package_dir, 'launch/rs_camera_d435i.yaml')],
            remappings=[
                ('/camera/color/image_raw', '/rgb'),
                ('/camera/aligned_depth_to_color/image_raw', '/depth')
            ]
        )
    ])