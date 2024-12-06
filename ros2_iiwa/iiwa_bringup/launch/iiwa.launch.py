# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='iiwa_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='iiwa_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa.config.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='velocity_controller',
            description='Robot controller to start.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.170.10.2',
            description='Robot IP of FRI interface',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='30200',
            description='Robot port of FRI interface.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='velocity', #setted in order to have a velocity controller by defoult
            description='Robot command interface [position|velocity|effort].',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_vision',
            default_value='false', 
            description='Configuration of robot with camera',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_world',
            default_value='empty.world', 
            description='Configuration of robot with camera',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    use_vision = LaunchConfiguration('use_vision')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ),
            ' ',
            'prefix:=',
            prefix,
            ' ',
            'use_sim:=',
            use_sim,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'robot_port:=',
            robot_port,
            ' ',
            'initial_positions_file:=',
            initial_positions_file,
            ' ',
            'command_interface:=',
            command_interface,
            ' ',
            'base_frame_file:=',
            base_frame_file,
            ' ',
            'description_package:=',
            description_package,
            ' ',
            'runtime_config_package:=',
            runtime_config_package,
            ' ',
            'controllers_file:=',
            controllers_file,
            ' ',
            'namespace:=',
            namespace,
            ' ', 
            'use_vision:=',
            use_vision
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Running with Moveit2 planning
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'),
            '/launch',
            '/iiwa_planning.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'start_rviz': start_rviz,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
            'use_sim': use_sim,
        }.items(),
        condition=IfCondition(use_planning),
    )

    # Running with Moveit2 servoing
    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'),
            '/launch',
            '/iiwa_servoing.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'base_frame_file': base_frame_file,
            'namespace': namespace,
        }.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            controllers_file,
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'iiwa_with_camera.rviz'] ###our config file
    ) 

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
        ],
        #condition=UnlessCondition(OrSubstitution(use_planning, use_sim)), #Commented to start both Rviz and Gazebo
    )
    iiwa_simulation_world = PathJoinSubstitution(
        [FindPackageShare(description_package),
            'gazebo/worlds', LaunchConfiguration('gazebo_world')]
    )

    """declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)"""
        
    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value=iiwa_simulation_world,
                            description='Arguments for gz_sim'),)
    
    """
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:world_simulation_models
        Siccome il suo empty.world si richiama un ground plane e un sun, assicurati di avere tali modelli installati da qualche
        parte sul tuo pc, e metti quel percorso nella EV
    """
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
            condition=IfCondition(use_sim),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'iiwa',
                   '-allow_renaming', 'true',],
        condition=IfCondition(use_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=IfCondition(use_sim),
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

  ###Our Modification vvvv
    # Node for bridging the camera topics from Gazebo gz to ROS2
    bridge_camera_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )
    
    joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        )
    ###Our Modifcation ^^^^
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node,bridge_camera_node], #bridge is launched with rviz, add joint state publisher node here if u want to use gui
           
        ),
        condition=IfCondition(start_rviz),
        
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    
   

    nodes = [
        gazebo,
        control_node,
        iiwa_planning_launch,
        iiwa_servoing_launch,
        spawn_entity,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        external_torque_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        #joint_state_publisher_node, #delayed, Add by us to have gui on rviz
        #bridge_camera_node delayed
    ]

    return LaunchDescription(declared_arguments + nodes)
