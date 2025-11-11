from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the simulate directory (parent of raycaster_ros2)
    package_dir = get_package_share_directory('raycaster_ros2')
    # Navigate up from install/raycaster_ros2/share/raycaster_ros2 to simulate
    simulate_dir = os.path.abspath(os.path.join(package_dir, '../../../../..'))
    mujoco_lib_dir = os.path.join(simulate_dir, 'mujoco', 'lib')
    
    # Declare arguments
    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value='',
        description='Path to MuJoCo XML model file'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )
    
    # Set LD_LIBRARY_PATH to include MuJoCo library
    set_lib_path = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        mujoco_lib_dir + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )
    
    # RayCaster ROS2 node
    raycaster_node = Node(
        package='raycaster_ros2',
        executable='raycaster_ros2_node',
        name='raycaster_publisher',
        output='screen',
        parameters=[{
            'model_file': LaunchConfiguration('model_file'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }]
    )
    
    return LaunchDescription([
        model_file_arg,
        publish_rate_arg,
        publish_tf_arg,
        set_lib_path,
        raycaster_node,
    ])
