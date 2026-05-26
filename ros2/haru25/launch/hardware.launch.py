from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

import glob
import re
import time

# 手動操作用のやつ

def get_rosbag_path():
    rosbag_path = '/home/avo/rosbag/'
    # show directory name
    try:
        dir_names = glob.glob(f'{rosbag_path}[0-9]_[0-9][0-9]/', recursive=False)
        dir_names.sort(reverse=True)

        # 3_33
        dir_name = dir_names[0]
        rosbag_names = glob.glob(f'{dir_name}**/', recursive=False)
        rosbag_names = [re.findall(r'(?<=auto)[0-9]*', s) for s in rosbag_names]
        max_index = -1

        for rosbag_name in rosbag_names:
            for index in rosbag_name:
                index = int(index)
                if max_index < index:
                    max_index = index
        return f'{dir_name}auto{max_index+1}'
    except:
        print("rosbag error")
        now = datetime.datetime.now()
        date = now.strftime('%Y%m%d_%H%M%S')
        return f'{rosbag_path}{date}'

def generate_launch_description():
    www_path    = PathJoinSubstitution([FindPackageShare("haru25_webgui"),"www"])
    plan_path   = PathJoinSubstitution([FindPackageShare("haru25"),"plan","csv", "plan.csv"])
    bt_path     = PathJoinSubstitution([FindPackageShare("haru25"),"config","haru25.xml"])
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
        parameters=[{'bt_path': bt_path}, param_path],
        output="both"
    )
    
    node_haru25_navigation = Node(
        package="haru25_navigation",
        executable="navigation",
        parameters=[{'plan_path': plan_path}, param_path],
        output="both"
    )
    
    node_haru25_hardware = Node(
        package="haru25_hardware",
        executable="haru25_hardware",
        parameters=[{'port': '/dev/teensy'}],
        output="both"
    )
    
    node_ld19 = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_scan'},
            {'port_name': '/dev/lidar'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 0.0},
            {'angle_crop_max': 0.0}
            ]
    )
    
    rosbag_path = get_rosbag_path()
    print(f'rosbag: {rosbag_path}')
    
    cmd_rosbag = LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', rosbag_path],
            output='screen', log_cmd=True
        )
    ])
    
    nodes = [
        node_haru25_webgui,
        node_haru25_bt,
        node_haru25_navigation,
        node_haru25_hardware,
        node_ld19,
        cmd_rosbag
    ]
    
    return LaunchDescription(nodes)