import os
import launch
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取默认路径
    cloud_car_dir = get_package_share_directory('cloud_car')
    xacro_file = os.path.join(cloud_car_dir, 'urdf', 'cloud_car_model', 'cloud_car.urdf.xacro')
    world_file = os.path.join(cloud_car_dir, 'world', 'tianma.world')
    
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(xacro_file),
        description='XACRO的绝对路径'
    )
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue( # type: ignore
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )
  	
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='log'
    )

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
      	# 传递参数
        launch_arguments=[('world', world_file),('verbose','false')],
    )

    # 请求 Gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
                   '-entity','cloud_car'],
        output='log'
    )
    
    # 加载并激活 cloud_car_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'cloud_car_joint_state_broadcaster'],
        output='log'
    )

    # # 加载并激活 cloud_car_effort_controller 控制器
    # load_cloud_car_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','cloud_car_effort_controller'], 
    #     output='log'
    # )
    
    load_cloud_car_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','cloud_car_diff_drive_controller'], 
        output='log'
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit( # type: ignore
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit( # type: ignore
        #         target_action=spawn_entity_node,
        #         on_exit=[load_cloud_car_effort_controller],
        #     )
        # ),
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit( # type: ignore
            target_action=load_joint_state_controller,
            on_exit=[load_cloud_car_diff_drive_controller],
            )
        ),
    ])
