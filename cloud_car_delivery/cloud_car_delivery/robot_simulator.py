"""
机器人模拟程序
不发布图像
真实接收目的地信息
速度随机
电量正常衰减
经纬度随目标点变化而变化，是从目标点文件中读取的
机器人状态正常模拟。与http_client_node.py中的状态一致
正常发送到达目标点信息
轮询是否取货
正常发送返回站点信息
不能故障
"""

import csv
import time
import json
import random
import requests
import logging
import numpy as np
import threading
from cloud_car_delivery import BASE_GOALS_CSV_PATH, CSV_EXTENSION


class RobotSimulator:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.base_url = "http://120.46.199.126:8080/robot"
        # self.base_url = "http://localhost:8000/robot"
        self.status = 'idle'  # idle, busy
        self.battery = 100
        self.position = {
            "longitude": 112.945295,
            "latitude": 28.164779
        }
        self.goals = None
        self.current_goal_index = 1
        self.goal_count = 0
        self.running = True
        self.is_navigating_to_goal = True  # True: 前往目标点, False: 返回站点
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # 配置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(message)s'
        )
        self.logger = logging.getLogger(robot_id)

    def post_velocity(self, isZero=False):
        """更新并发送随机速度"""
        if self.status == 'busy':
            if isZero:
                self.current_linear_velocity = 0.0
                self.current_angular_velocity = 0.0
            else:
                self.current_linear_velocity = random.uniform(0.5, 0.7)
            self.current_angular_velocity = random.uniform(-0.2, 0.2)
            try:
                url = f"{self.base_url}/velocity"
                requests.post(
                    url,
                    json={
                        "robotId": int(self.robot_id),
                        "linear_velocity": self.current_linear_velocity,
                        "angular_velocity": self.current_angular_velocity
                    }
                )
            except Exception as e:
                self.logger.error(f"机器人{self.robot_id}速度发送失败: {e}")


    def post_state(self):
        """发送状态并接收新任务"""
        url = f"{self.base_url}/state"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": int(self.robot_id),
                    "latitude": self.position["latitude"],
                    "longitude": self.position["longitude"],
                    "battery": self.battery,
                    "status": self.status
                }
            )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50021:
                    data = response_data['data']
                    if data and data['delivery_address'] is not None:
                        if self.status == 'idle':
                            self.load_goals(data['delivery_address'])
                            self.status = 'busy'
                            self.logger.info(f"机器人{self.robot_id}获取新任务: {data['delivery_address']}")
                        return True
            return False
        except Exception as e:
            self.logger.error(f"机器人{self.robot_id}状态发送失败")
            return False


    def load_goals(self, address):
        """从CSV文件加载目标点和经纬度"""
        csv_file_path = BASE_GOALS_CSV_PATH + address + CSV_EXTENSION
        try:
            with open(csv_file_path, 'r') as f:
                reader = csv.reader(f)
                self.goals = np.array([list(map(float, cols)) for cols in reader])
                
            if self.goals.shape[0] > 0 and self.goals.shape[1] == 5:
                self.logger.info(f"机器人{self.robot_id}成功加载目标点")
                self.goal_count = len(self.goals)
                self.current_goal_index = 1
                self.is_navigating_to_goal = True
                return True
            return False
        except Exception as e:
            self.logger.error(f"机器人{self.robot_id}加载目标点失败: {e}")
            return False


    def update_position_and_battery(self):
        """更新机器人的经纬度和电量"""
        current_goal = self.goals[self.current_goal_index]
        self.position["longitude"] = current_goal[3]
        self.position["latitude"] = current_goal[4]
        self.battery -= 1
        if self.battery < 0:
            self.battery = 0
        self.logger.info(f"机器人{self.robot_id}位置更新为: 经度 {current_goal[3]}, 纬度 {current_goal[4]}")


    def simulate_delivery(self):
        """模拟配送过程"""
        self.logger.info(f"机器人{self.robot_id}开始配送任务")
        
        while self.current_goal_index < self.goal_count - 1:
            # 正在前往current_goal_index目标点
            time.sleep(20)
            
            # 到达目标点后更新位置和状态
            self.update_position_and_battery()
            self.post_state()
            self.post_velocity()

            self.current_goal_index += 1

        # 正在前往终点
        time.sleep(20)
        # 到达终点后更新位置和状态
        self.update_position_and_battery()
        self.post_state()
        self.post_velocity(True)
        # 发送到达通知
        self.post_arrived()
        self.wait_for_pickup()


    def post_arrived(self):
        """通知到达目标点"""
        try:
            url = f"{self.base_url}/arrived"
            requests.post(
                url,
                json={
                    "robotId": int(self.robot_id)
                }
            )
            self.logger.info(f"机器人{self.robot_id}已到达目标点")
        except Exception as e:
            self.logger.error(f"机器人{self.robot_id}到达通知失败: {e}")


    def wait_for_pickup(self):
        """等待取货"""
        while self.post_pickup_status():
            time.sleep(5)
            self.update_position_and_battery()
            self.logger.info(f"机器人{self.robot_id}正在等待取货")


    def post_pickup_status(self):
        """检查是否取货完成"""
        url = f"{self.base_url}/pickup"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": int(self.robot_id)
                }
            )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50041 and response_data['data']['isPicked']:
                    self.is_navigating_to_goal = False
                    self.current_goal_index -= 1
                    self.logger.info(f"机器人{self.robot_id}收到取货完成消息")
                    return False
            return True
        except Exception as e:
            self.logger.error(f"机器人{self.robot_id}取货状态检查失败: {e}")
        return True


    def simulate_return(self):
        """模拟返回过程"""
        self.logger.info(f"机器人{self.robot_id}开始返回站点")

        while self.current_goal_index > 0:
            time.sleep(20)

            self.update_position_and_battery()
            self.post_state()
            self.post_velocity()
            
            self.current_goal_index -= 1

        # 正在返回站点
        time.sleep(20)
        self.update_position_and_battery()
        self.post_state()
        self.post_velocity(True)
        # 发送返回站点通知
        self.post_back_to_station()
        

    def post_back_to_station(self):
        """通知返回站点"""
        try:
            self.status = 'idle'
            self.battery = 100

            url = f"{self.base_url}/back" 
            requests.post(
                url,
                json={
                    "robotId": int(self.robot_id)
                }
            )
            self.logger.info(f"机器人{self.robot_id}已返回站点")
        except Exception as e:
            self.logger.error(f"机器人{self.robot_id}返回通知失败: {e}")


    def run(self):
        """运行机器人模拟器"""
        self.logger.info(f"机器人{self.robot_id}启动")
        # self.post_back_to_station()
        while self.running:
            try:
                # 发送状态, 获取新任务
                self.post_state()
                time.sleep(1)
                # 执行任务
                if self.status == 'busy':
                    if self.is_navigating_to_goal:
                        # 模拟配送
                        self.simulate_delivery()
                    else:
                        # 模拟返回
                        self.simulate_return()
                else:
                    # self.logger.info(f"机器人{self.robot_id}空闲, 等待新任务")
                    time.sleep(1)
            except Exception as e:
                self.logger.error(f"机器人{self.robot_id}运行错误: {e}")
                time.sleep(5)


class RobotFleetManager:
    def __init__(self):
        self.robots = []
        self.threads = []


    def add_robot(self, robot_id):
        """添加机器人到车队"""
        robot = RobotSimulator(str(robot_id))
        self.robots.append(robot)


    def start(self):
        """启动所有机器人"""
        for robot in self.robots:
            thread = threading.Thread(target=robot.run)
            thread.daemon = True
            self.threads.append(thread)
            thread.start()


    def stop(self):
        """停止所有机器人"""
        for robot in self.robots:
            robot.running = False
        for thread in self.threads:
            thread.join()


def main():
    manager = RobotFleetManager()
    for robot_id in range(2, 6):  # 创建4个机器人(ID: 2-5)
        manager.add_robot(robot_id)
    
    try:
        manager.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        manager.stop()

if __name__ == '__main__':
    main() 