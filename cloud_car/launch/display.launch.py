import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 获取默认路径
    cloud_car_dir = get_package_share_directory('cloud_car')
    xacro_file = os.path.join(cloud_car_dir, 'urdf', 'first_robot.xacro')
    default_rviz_config_path = os.path.join(cloud_car_dir , 'config' , 'cloud_car_model.rviz')
    
    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=str(xacro_file),
        description='XACRO 的绝对路径'
    )
    
    # 获取文件内容生成新的参数
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])