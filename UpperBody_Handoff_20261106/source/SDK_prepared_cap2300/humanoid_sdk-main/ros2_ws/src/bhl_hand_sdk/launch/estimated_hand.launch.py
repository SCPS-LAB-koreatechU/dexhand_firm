"""Isolated visualization of camera/calibration estimates; no robot controller."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('bhl_hand_sdk'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('input_file',default_value=str(Path.home()/'.local/state/bhl/camera_retarget_preview.json')),
        Node(package='bhl_hand_sdk',executable='hand_preview_state',output='screen',parameters=[{'input_file':LaunchConfiguration('input_file')}]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='hand_preview',parameters=[{'robot_description':(share/'preview_model/estimated_right_hand.urdf').read_text()}],remappings=[('joint_states','/hand_camera/estimated_joint_states')]),
        Node(package='rviz2',executable='rviz2',name='hand_preview_rviz',arguments=['-d',str(share/'preview_model/estimated.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
