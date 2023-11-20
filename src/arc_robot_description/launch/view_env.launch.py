import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_path = get_package_share_directory('arc_robot_description') + '/urdf/table_common.urdf.xacro'

    return LaunchDescription([
        # Load robot_description parameter
        launch.actions.SetEnvironmentVariable('ROBOT_DESCRIPTION', urdf_file_path),
        
        # Launch joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
        # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': launch.substitutions.EnvironmentVariable('ROBOT_DESCRIPTION')}]
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', get_package_share_directory('arc_robot_description') + '/rviz/view_env.rviz']
        ),
    ])
