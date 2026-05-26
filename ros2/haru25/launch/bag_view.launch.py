import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('haru25'), 'rviz', 'rviz.rviz')
    www_path    = PathJoinSubstitution([FindPackageShare("haru25_webgui"),"www"])
    plan_path   = PathJoinSubstitution([FindPackageShare("haru25"),"plan","csv", "plan.csv"])
    bt_path     = PathJoinSubstitution([FindPackageShare("haru25"),"config","haru25.xml"])
    bt_restart_path  = PathJoinSubstitution([FindPackageShare("haru25"),"config","haru25_restart.xml"])
    param_path  = PathJoinSubstitution([FindPackageShare("haru25"),"config","param_hardware.yaml"])
    
    node_haru25_webgui = Node(
        package="haru25_webgui",
        executable="haru25_webgui",
        parameters=[{'www_path': www_path}],
        output="both"
    )
    
    node_haru25_bt = Node(
        package="haru25_bt",
        executable="haru25_bt",
        parameters=[{'bt_path': bt_path},{'bt_restart_path': bt_restart_path}, param_path],
        output="both"
    )
    
    node_haru25_navigation = Node(
        package="haru25_navigation",
        executable="navigation",
        parameters=[{'plan_path': plan_path}, param_path, {'pub_lidar_conv': True}],
        output="both"
    )
    
    node_rviz2 = Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_dir],
                    #parameters=[{'use_sim_time': True}]
                    )
    
    node_tf_static_pub = Node(
                        package='tf2_ros',  # パッケージ名
                        executable='static_transform_publisher',  # 実行するノード名
                        name='static_transform_publisher',  # ノードの名前
                        output='screen',
                        arguments=['--x', '-0.206902', 
                                   '--y', '-0.207186',
                                   '--z', '0.02',
                                   '--roll', '0.0',
                                   '--pitch', '0.0',
                                   '--yaw', '0.785398163',
                                   '--frame-id', 'base_link',
                                   '--child-frame-id', 'base_scan']  # 引数の例 (位置(x, y, z) と 回転 (roll, pitch, yaw))
                    )
    
    node_scan_conv = Node(
                    package='haru25_tools',
                    executable='scan_conv',
                    output='screen',
                    )
    node_pos2tf = Node(
                    package='haru25_tools',
                    executable='pos2tf',
                    output='screen',
                    )

    return LaunchDescription([
        #node_haru25_webgui,
        #node_haru25_bt,
        #node_haru25_navigation,
        node_tf_static_pub,
        node_scan_conv,
        node_pos2tf,
        node_rviz2
    ])