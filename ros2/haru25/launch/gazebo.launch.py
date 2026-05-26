from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    www_path    = PathJoinSubstitution([FindPackageShare("haru25_webgui"),"www"])
    plan_path   = PathJoinSubstitution([FindPackageShare("haru25"),"plan","csv", "plan.csv"])
    bt_path     = PathJoinSubstitution([FindPackageShare("haru25"),"config","haru25.xml"])
    param_path  = PathJoinSubstitution([FindPackageShare("haru25"),"config","param_gazebo.yaml"])
    
    world_path = PathJoinSubstitution([FindPackageShare("haru25_gazebo"),"world","haru25.world"])
    gazebo_path = PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("haru25_gazebo"),"launch","gazebo.launch.py"])])

    node_haru25_webgui = Node(
        package="haru25_webgui",
        executable="haru25_webgui",
        parameters=[{'www_path': www_path}],
        output="both"
    )
    
    node_haru25_bt = Node(
        package="haru25_bt",
        executable="haru25_bt",
        parameters=[{'bt_path': bt_path}, param_path],
        output="both"
    )
    
    node_haru25_navigation = Node(
        package="haru25_navigation",
        executable="navigation",
        parameters=[{'plan_path': plan_path}, param_path, {'pub_lidar_conv': True}],
        output="both"
    )
    
    node_haru25_gazebo = Node(
        package="haru25_gazebo",
        executable="haru25_gazebo",
        output="both",
    )
    
    # Gazeboでの初位置設定
    gazebo_launch = IncludeLaunchDescription(gazebo_path, 
                    launch_arguments = {
                        'world_path': world_path,
                        'spawn_x': '3.112',
                        'spawn_y': '0.29',
                        #'spawn_x': '1.75',
                        #'spawn_y': '3.0',
                        'spawn_yaw': '0.0'
                    }.items())
    
    nodes = [
        node_haru25_webgui,
        node_haru25_bt,
        node_haru25_navigation,
        node_haru25_gazebo,
        gazebo_launch,
    ]
    
    return LaunchDescription(nodes)