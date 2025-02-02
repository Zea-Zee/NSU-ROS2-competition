import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction

def resolve_start_position(context, sp):
    # Получаем значение аргумента 'sp' (номер позиции старта)
    #start_position_arg = LaunchConfiguration('sp').perform(context)
    start_position_arg = context.perform_substitution(sp)

    # Список стартовых координат
    start_position_list = {
        '0': [0.8, -1.747, 0.08, 0.0],
        '1': [1.64, -0.75, 0.08, 3.14],
        '2': [0.67, 0.25, 0.08, 0.0],
        '3': [1.2, 1.75, 0.08, 3.14],
        '4': [-1.60, 1.25, 0.08, 0.0],
        '5': [-1.75, 0.4, 0.08, 3.14*1.5],
        '6': [0.3, 1.75, 0.08, 3.14],
    }

    # Получаем координаты для заданного номера позиции
    start_position_cords = start_position_list[start_position_arg]

    # Создаём узел для спауна робота с выбранными координатами
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', str(start_position_cords[0]),
            '-y', str(start_position_cords[1]),
            '-z', str(start_position_cords[2]),
            '-Y', str(start_position_cords[3]),
        ],
        output='screen',
    )
    # global start_angle
    # start_angle = start_position_cords[3]

    context.launch_configurations['spawn_x'] = str(start_position_cords[0])
    context.launch_configurations['spawn_y'] = str(start_position_cords[1])
    context.launch_configurations['spawn_z'] = str(start_position_cords[2])
    context.launch_configurations['spawn_angle'] = str(start_position_cords[3])

    context.launch_configurations['spawn_x'] = str(start_position_cords[0])
    context.launch_configurations['spawn_y'] = str(start_position_cords[1])
    context.launch_configurations['spawn_z'] = str(start_position_cords[2])
    context.launch_configurations['spawn_angle'] = str(start_position_cords[3])

    return [create_node]

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robot_bringup')
    pkg_project_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path  =  os.path.join(pkg_project_description, 'urdf', 'robot.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)


    sp_arg = DeclareLaunchArgument(
        'sp',
        default_value='0',
        description='Start position:\n0 - default\n1 - razvilka\n2 - labirint\n3 - parking\n4 - peshehod\n5 - tunnel'
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': "-r course.sdf"}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"},
            {"use_sim_time": True}
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'autorace_2023.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[
            {"use_sim_time": True}
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            "use_sim_time": True,
        }],
        output='screen'
    )
    
    robot_controller = Node(
        package='my_robot_controller',
        executable='run',
        parameters=[{
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_angle': LaunchConfiguration('spawn_angle'),
        }]

    )

    referee = Node(
        package='referee_console',
        executable='mission_autorace_2023_referee'
    )


    return LaunchDescription([
        sp_arg,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('sp', default_value='0',
                              description='Start position:\n0 - default\n1 - razvilka\n2 - labirint\n3 - parking\n4 - peshehod\n5 - tunnel'),
        bridge,
        robot_state_publisher,
        rviz,
        TimerAction(
            period=3.0,
            actions=[
                OpaqueFunction(function=resolve_start_position, args=[LaunchConfiguration('sp')])
        ]),
        TimerAction(
            period=5.0,
            actions=[referee, robot_controller])
    ])
