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
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'), 'rviz', 'urdf_odom_config.rviz')
    


    lidar_tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    

    rsp_node = Node(package="robot_state_publisher",
                    executable="robot_state_publisher",
                    parameters=[{"robot_description": robot_description}],
                    output="screen")
    
    jsp_node = Node(package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen'
    )
    
    rviz_node = Node(package="rviz2",
                    executable="rviz2",
                    arguments=["-d", rviz_config_path]
            )
    

    mcu_bridge_node = Node(package='my_cpp_pkg',
                               executable='mcu_bridge_node',
                               name='mcu_bridge_node')
    
    
    odometry_node = Node(package='my_cpp_pkg',
                         executable='odometry_node',
                         name='odometry_node')
    
 
    
    
    
    return LaunchDescription([
        mcu_bridge_node,
        rsp_node,                     
        odometry_node,
        jsp_node,
        rviz_node,
    ])