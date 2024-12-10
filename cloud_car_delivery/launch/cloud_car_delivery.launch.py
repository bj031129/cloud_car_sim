from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的路径
    cloud_car_pkg = get_package_share_directory('cloud_car')
    cloud_car_nav2_pkg = get_package_share_directory('cloud_car_nav2')
    
    # 包含 gazebo 仿真启动文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cloud_car_pkg, 'launch', 'gazebo_sim.launch.py')
        )
    )
    
    # 包含 navigation2 启动文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cloud_car_nav2_pkg, 'launch', 'navigation2.launch.py')
        )
    )
    
    # 启动 delivery_node
    delivery_node = Node(
        package='cloud_car_delivery',
        executable='delivery_node',
        name='delivery_node',
        output='screen',
    )
    
    # 添加 HTTP 客户端节点
    http_client_node = Node(
        package='cloud_car_delivery',
        executable='http_client_node',
        name='http_client_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        delivery_node,
        http_client_node
    ]) 