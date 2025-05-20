import os
from ament_index_python import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    
    #paths
    
    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 'urdf', 'my_robot.urdf.xacro')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
      
   

    rsp_node = Node(package="robot_state_publisher",
                    executable="robot_state_publisher",
                    parameters=[{"robot_description": robot_description}],
                    output="screen")
    
    jsp_node = Node(package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen'
    )
    
   

    arduino_bridge_node = Node(package='my_cpp_pkg',
                               executable='arduino_bridge_node',
                               name='arduino_bridge_node')
    
    
    odometry_node = Node(package='my_cpp_pkg',
                         executable='odometry_node',
                         name='odometry_node')
    
 
    
    
    
    return LaunchDescription([
        arduino_bridge_node,
        rsp_node,                     
        odometry_node,
        jsp_node,
    ])