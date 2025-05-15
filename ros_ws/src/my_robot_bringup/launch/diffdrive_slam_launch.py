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
    
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'), 'rviz', 'urdf_config_.rviz')
    
    lua_config_path = os.path.join(get_package_share_path('my_robot_bringup'), 'config')
    
    
    
    
    #Launch cofing
    
    lidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    lidar_param_file = LaunchConfiguration('params_file')
    
    lidar_params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(lidar_share_dir, 'params', 'TminiPro.yaml'))
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    
    
    
    
    #Nodes
    lidar_driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[lidar_param_file],
                                namespace='/',
                                )
    
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
    

    arduino_bridge_node = Node(package='my_cpp_pkg',
                               executable='arduino_bridge_node',
                               name='arduino_bridge_node')
    
    
    odometry_node = Node(package='my_cpp_pkg',
                         executable='odometry_node',
                         name='odometry_node')
    
    
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory", lua_config_path,
            "-configuration_basename", "my_robot.lua"
        ],
        output="screen"
    )
    
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
   
    
    
    
    return LaunchDescription([
        lidar_params_declare,
        lidar_driver_node,
        rsp_node,               
        lidar_tf2_node,        
        arduino_bridge_node,
        odometry_node,
        jsp_node,
        rviz_node,
        cartographer_node,
        cartographer_occupancy_grid_node   
    ])