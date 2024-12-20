import csv
import json
import time
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from . import BASE_GOALS_CSV_PATH, CSV_EXTENSION


class DeliveryNode(Node):
    def __init__(self):
        super().__init__('delivery_node')
        self.get_logger().info('delivery_node started')

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arrived_publisher = self.create_publisher(String, '/arrived', 10)
        self.back_to_station_publisher = self.create_publisher(String, '/back_to_station', 10)
        self.pickup_complete_publisher = self.create_publisher(String, '/pickup_complete', 10)
        self.position_publisher = self.create_publisher(String, '/position', 10)
        
        self.delivery_address_subscription = self.create_subscription(String, '/delivery_address', self.delivery_address_callback, 10)
        self.pickup_complete_subscription = self.create_subscription(String, '/pickup_complete', self.pickup_complete_callback, 10)
        self.status_subscription = self.create_subscription(String, '/status', self.status_callback, 10)
        
        # maintenance 维护中
        # idle 空闲
        # busy 忙碌
        self.status = 'idle'
        self.isArrived = False
        self.isBack = True
        self.goals = []
        self.last_nav_time = time.time()
        self.current_goal_index = 1
        self.goal_count = 0
        self.MIN_DISTANCE = 0.5
        self.goal_handle = None
        self.bridge = CvBridge()
        self._navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_set = False  # 添加标志变量
        self.repair_check_timer = None  # 定时器变量
        self.send_goal_timer = None  # 定时器变量
        self.is_navigating_to_goal = True  # True: 前往目标点, False: 返回站点


    def delivery_address_callback(self, msg):
        """
        处理派送地址订阅消息的回调函数
        :param msg: 包含派送地址的消息
        """
        if self.status == 'idle':
            self.get_logger().info(f'收到新的派送地址: {msg.data}')
            
            # 设置CSV文件路径
            csv_file_path = BASE_GOALS_CSV_PATH + msg.data + CSV_EXTENSION
            
            try:
                with open(csv_file_path, 'r') as f:
                    reader = csv.reader(f)
                    self.goals = np.array([list(map(float, cols)) for cols in reader])
                
                if self.goals.shape[0] > 0 and self.goals.shape[1] == 5:
                    self.get_logger().info("成功加载目标点")
                    self.goal_count = len(self.goals)
                    self.current_goal_index = 1
                    
                    if not self.initial_pose_set:  # 检查是否已经设置过初始位姿
                        self.init_robot_pose(0.0, 0.0, 0.0)
                        time.sleep(2)  # 等待初始位姿设置完成
                        self.initial_pose_set = True  # 设置标志为 True
                    
                    self.isBack = False
                    self.is_navigating_to_goal = True  # 设置为前往目标点
                    self.send_goal()
                    # 设置超时重发
                    self.send_goal_timer = self.create_timer(5.0, self.check_send_goal_timeout)
                else:
                    self.get_logger().error("目标点格式无效")
            except FileNotFoundError:
                self.get_logger().error(f"找不到配送地址对应的目标点文件: {csv_file_path}")
            except Exception as e:
                self.get_logger().error(f"加载目标点时发生错误: {str(e)}")
        else:
            self.get_logger().warn("机器人正在执行任务，无法接受新的派送地址")
    

    def init_robot_pose(self, x, y, yaw):
        """
        初始化机器人的位置和姿态
        :param x: 初始位置的x坐标
        :param y: 初始位置的y坐标
        :param yaw: 初始姿态的偏航角（弧度）
        """
        self.get_logger().info("初始化机器人位姿...")
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0

        # 将欧拉角转换为四元数
        quaternion = quaternion_from_euler(0, 0, yaw)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]

        # 设置协方差矩阵（可以根据实际情况调整）
        initial_pose.pose.covariance = [0.0] * 36

        # 发布初始位姿
        self.initial_pose_publisher.publish(initial_pose)
        self.get_logger().info(f'初始位姿已发布: 位置({x}, {y}), 姿态(yaw={yaw})')


    def send_goal(self):
        """
        发送导航目标点
        """
        self.last_nav_time = time.time()
        self._navigate_to_pose_client.wait_for_server()
        current_goal = self.goals[self.current_goal_index]
        goal_msg = self.get_goal_msg_by_xyyaw(current_goal[0], current_goal[1], current_goal[2])
        self.goal_pose_publisher.publish(goal_msg)
        self.get_logger().info(f'发送目标点: 位置({current_goal[0]}, {current_goal[1]}), 姿态(yaw={current_goal[2]})')

        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = goal_msg
        
        self._navigate_to_pose_client.send_goal_async(navigate_goal, feedback_callback=self.send_goal_feedback).add_done_callback(self.send_goal_callback)


    def send_goal_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        # if feedback.distance_remaining > self.MIN_DISTANCE:
        #     self.get_logger().info(f'当前距离目标点: {feedback.distance_remaining:.2f}米')
        # else:
        if feedback.distance_remaining <= self.MIN_DISTANCE:
            self.cancel_goal()  # 取消当前目标点
            # 到达目标点后发布经纬度信息
            current_goal = self.goals[self.current_goal_index]
            position_data = {
                "longitude": current_goal[3],
                "latitude": current_goal[4]
            }
            position_msg = String()
            position_msg.data = json.dumps(position_data)
            self.position_publisher.publish(position_msg)
            
            if self.current_goal_index == self.goal_count - 1:
                if self.isArrived == False:
                    self.isArrived = True
                    self.arrived_publisher.publish(String(data='true'))
                    self.get_logger().info(f'终点已到达，等待客户取货')
                    self.send_goal_timer.cancel()  # 取消定时器
                    self.send_goal_timer = None
            else:
                self.get_logger().info(f'目标点已到达')
                self.current_goal_index += 1
                self.send_goal()


    def send_goal_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            self.send_goal()
            return
        self.get_logger().info('目标已被接受')


    def pickup_complete_callback(self, msg):
        if msg.data == 'true':
            self.get_logger().info('收到取货完成消息，导航回起始位置')
            # 设置超时重发
            self.send_goal_timer = self.create_timer(5.0, self.check_send_goal_timeout)
            self.current_goal_index -= 1
            self.is_navigating_to_goal = False  # 设置为返回站点
            self.nav_to_start()


    def nav_to_start(self):
        self.last_nav_time = time.time()
        self._navigate_to_pose_client.wait_for_server()
        current_goal = self.goals[self.current_goal_index]
        goal_msg = self.get_goal_msg_by_xyyaw(current_goal[0], current_goal[1], current_goal[2])
        self.goal_pose_publisher.publish(goal_msg)
        self.get_logger().info(f'发送目标点: 位置({current_goal[0]}, {current_goal[1]}), 姿态(yaw={current_goal[2]})')

        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = goal_msg

        self._navigate_to_pose_client.send_goal_async(navigate_goal, feedback_callback=self.nav_to_start_feedback).add_done_callback(self.nav_to_start_callback)
        self.isArrived = False


    def nav_to_start_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # if feedback.distance_remaining > self.MIN_DISTANCE:
        #     self.get_logger().info(f'当前距离目标点: {feedback.distance_remaining:.2f}米')
        # else:
        if feedback.distance_remaining <= self.MIN_DISTANCE:
            self.cancel_goal()  # 取消当前目标点
            # 发布经纬度信息
            current_goal = self.goals[self.current_goal_index]
            position_data = {
                "longitude": current_goal[3],
                "latitude": current_goal[4]
            }
            position_msg = String()
            position_msg.data = json.dumps(position_data)
            self.position_publisher.publish(position_msg)
            
            if self.current_goal_index == 0:
                if self.isBack == False:
                    self.isBack = True
                    self.back_to_station_publisher.publish(String(data='true'))
                    self.get_logger().info('已回到站点')
                    self.send_goal_timer.cancel()  # 取消定时器
                    self.send_goal_timer = None
            else:
                self.get_logger().info(f'目标点已到达')
                self.current_goal_index -= 1
                self.nav_to_start()


    def nav_to_start_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            self.nav_to_start()
        else:
            self.get_logger().info('目标已被接受')


    def check_send_goal_timeout(self):
        """
        检查发送目标的超时状态
        """
        if time.time() - self.last_nav_time > 30:
            self.get_logger().info('目标发送超时，重新发送目标')
            if self.is_navigating_to_goal:
                self.send_goal()
            else:
                self.nav_to_start()


    def get_goal_msg_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation.x = rotation_quat[0]
        goal_msg.pose.orientation.y = rotation_quat[1]
        goal_msg.pose.orientation.z = rotation_quat[2]
        goal_msg.pose.orientation.w = rotation_quat[3]
        return goal_msg


    def status_callback(self, msg):
        """
        处理状态订阅消息的回调函数
        :param msg: 包含状态信息的消息
        """
        self.status = msg.data
        self.get_logger().info(f'状态更新为: {self.status}')
        
        if self.status == 'maintenance':
            self.get_logger().info('机器人故障，取消当前目标点')
            self.cancel_goal()  # 取消当前目标点
            
            # 启动定时器检查状态
            if self.repair_check_timer is None:
                self.repair_check_timer = self.create_timer(5.0, self.check_repair_status)


    def cancel_goal(self):
        """
        取消当前目标点的函数
        """
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('当前目标点已取消')
        return


    def check_repair_status(self):
        """
        检查修理状态
        """
        if self.status == 'busy':
            self.get_logger().info('机器人已修好，继续发送目标点')
            self.repair_check_timer.cancel()
            self.repair_check_timer = None
            if self.is_navigating_to_goal:
                self.send_goal()
            else:
                self.nav_to_start()


def main():
    rclpy.init()
    delivery = DeliveryNode()
    rclpy.spin(delivery)
    delivery.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()