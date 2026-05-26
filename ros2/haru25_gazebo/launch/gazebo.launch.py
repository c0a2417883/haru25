from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    robot_name = "haru25"
    
    DeclareLaunchArgument('spawn_x', default_value="0.0")
    DeclareLaunchArgument('spawn_y', default_value="0.0")
    DeclareLaunchArgument('spawn_z', default_value="0.0")
    DeclareLaunchArgument('spawn_yaw', default_value="0.0")
    DeclareLaunchArgument('world_path', default_value="")

    spawn_x = LaunchConfiguration('spawn_x', default="0.0")
    spawn_y = LaunchConfiguration('spawn_y', default="0.0")
    spawn_z = LaunchConfiguration('spawn_z', default="0.0")
    spawn_yaw = LaunchConfiguration('spawn_yaw', default="0.0")
    world_path = LaunchConfiguration('world_path', default="")
    
    gazebo_path = PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),"launch","gazebo.launch.py"])])
    
    gazebo_launch = IncludeLaunchDescription(gazebo_path, launch_arguments = {'world': world_path, 'verbose': 'true'}.items())

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("haru25_gazebo"),
                        "urdf",
                        robot_name + ".xacro",
                    ]
                ),
            ]
        ), value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name,
                                   '-x', spawn_x,
                                   '-y', spawn_y,
                                   '-z', spawn_z,
                                   '-Y', spawn_yaw,
                                   ],
                        output='screen')
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'wheel_controller'],
        output='screen'
    )
    
    #load_suspension_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'suspension_controller'],
    #    output='screen'
    #)
    
    nodes = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_wheel_controller],
            )
        ),
        #RegisterEventHandler(
        #    event_handler=OnProcessExit(
        #        target_action=load_wheel_controller,
        #        on_exit=[load_suspension_controller],
        #    )
        #),
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity
    ]
    
    return LaunchDescription(nodes)
    
    