import json
import time
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge


class HttpClientNode(Node):
    def __init__(self, base_url):
        super().__init__('http_client_node')
        
        self.delivery_publisher = self.create_publisher(String, '/delivery_address', 10)
        self.status_publisher = self.create_publisher(String, '/status', 10)
        self.pickup_publisher = self.create_publisher(String, '/pickup_complete', 10)      

        self.image_subscription = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.position_subscription = self.create_subscription(String, '/position', self.position_callback, 10)
        self.arrived_subscription = self.create_subscription(String, '/arrived', self.arrived_callback, 10)
        self.back_to_station_subscription = self.create_subscription(String, '/back_to_station', self.post_back_to_station, 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.maintenance_subscription = self.create_subscription(String, '/maintenance', self.maintenance_callback, 10)

        self.base_url = base_url
        self.bridge = CvBridge()
        self.current_image = None
        self.status = 'idle'
        self.battery = 100
        self.position = {
            "longitude": 112.945295,  # 经度
            "latitude": 28.164779  # 纬度
        }
        self.post_state_timer = self.create_timer(1.0, self.post_state)
        self.decrease_battery_timer = self.create_timer(20.0, self.change_battery)
        self.pickup_check_timer = None
        self.repair_check_timer = None
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.get_logger().info('HTTP客户端节点已启动')

        
    def image_callback(self, msg):
        """
        处理图像消息的回调函数
        :param msg: 包含图像消息
        """
        if self.status == 'busy':
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 将图像缩小为原始大小的 1/2
            height, width = cv_image.shape[:2]
            new_size = (width // 2, height // 2)
            resized_image = cv2.resize(cv_image, new_size, interpolation=cv2.INTER_AREA)
            
            # 压缩图像
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 设置JPEG压缩质量为50
            _, buffer = cv2.imencode('.jpg', resized_image, encode_param)
            
            self.current_image = buffer.tobytes()
            self.post_image()


    def post_image(self):
        """
        通过HTTP POST请求发送视频帧
        """
        if self.current_image is not None:
            url = f"{self.base_url}/video"
            try:
                response = requests.post(
                    url,
                    data=self.current_image,
                    headers={'Content-Type': 'image/jpeg'}
                )
                # if response.status_code == 200:
                #     response_data = response.json()
                #     if response_data['code'] == 50011:
                #         self.get_logger().info(response_data['msg'])
                #     else:
                #         self.get_logger().warning(f"图像发送异常: {response_data['msg']}")
                # else:
                #     response_data = response.json()
                #     self.get_logger().error(f"图像发送失败: {response_data['msg']}")
            except Exception as e:
                self.get_logger().error(f'发送图像时出错: {str(e)}')


    def cmd_vel_callback(self, msg):
        """
        处理速度信息的回调函数
        :param msg: Twist消息，包含线速度和角速度
        """
        if self.status == 'busy':
            self.current_linear_velocity = msg.linear.x
            self.current_angular_velocity = msg.angular.z
            self.post_vel()


    def post_vel(self):
        """
        发送速度信息到服务器
        """
        url = f"{self.base_url}/velocity"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": 1,
                    "linear_velocity": self.current_linear_velocity,
                    "angular_velocity": self.current_angular_velocity
                }
            )
            # if response.status_code == 200:
            #     response_data = response.json()
            #     if response_data['code'] == 50061:
            #         self.get_logger().info(response_data['msg'])
            #     else:
            #         self.get_logger().warning(f"速度信息发送异常: {response_data['msg']}")
            # else:
            #     response_data = response.json()
            #     self.get_logger().error(f"速度信息发送失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"发送速度信息请求失败: {e}")


    def change_battery(self):
        """
        每20秒电量下降1
        """
        if self.status == 'idle':
            self.battery = 100
        elif self.status == 'busy':
            self.battery -= 1
            if self.battery <= 0:
                self.battery = 0
                self.get_logger().warning('电量已耗尽')


    def position_callback(self, msg):
        """
        处理机器人位置信息的回调函数
        :param msg: 包含位置信息的消息
        """
        try:
            position_data = json.loads(msg.data)
            self.position["latitude"] = float(position_data["latitude"])
            self.position["longitude"] = float(position_data["longitude"])
        except Exception as e:
            self.get_logger().error(f'处理位置信息时出错: {str(e)}')


    def post_state(self):
        """
        发送机器人状态并接收配送地址
        """
        url = f"{self.base_url}/state"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": 1,
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
                    # 发布配送地址
                    if data and data['delivery_address'] is not None:
                        address_msg = String()
                        address_msg.data = data['delivery_address']
                        self.delivery_publisher.publish(address_msg)
                        self.status = 'busy'
                        self.status_publisher.publish(String(data=self.status))
                else:
                    self.get_logger().warning(f"状态发送异常: {response_data['msg']}")
            else:
                response_data = response.json()
                self.get_logger().error(f"状态发送失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"发送状态请求失败: {e}")


    def arrived_callback(self, msg):
        if msg.data == 'true':
            self.post_arrived()
            # 创建定时器检查取货状态
            self.pickup_check_timer = self.create_timer(5.0, self.post_pickup_status)


    def post_arrived(self):
        """
        发送到达信息到服务器
        """
        url = f"{self.base_url}/arrived"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": 1,
                }
            )
            # if response.status_code == 200:
            #     response_data = response.json()
            #     if response_data['code'] == 50031:
            #         self.get_logger().info(response_data['msg'])
            #     else:
            #         self.get_logger().warning(f"到达信息发送异常: {response_data['msg']}")
            # else:
            #     response_data = response.json()
            #     self.get_logger().error(f"到达信息发送失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"发送到达信息请求失败: {e}")


    def post_pickup_status(self):
        """
        定时检查取货状态
        """
        url = f"{self.base_url}/pickup"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": 1
                }
            )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50041 and response_data['data']['isPicked']:
                    # 用户已取货，发布消息
                    self.pickup_publisher.publish(String(data='true'))
                    # 取消定时器
                    if self.pickup_check_timer:
                        self.pickup_check_timer.cancel()
                        self.pickup_check_timer = None
            else:
                self.get_logger().warning(f"检查取货状态失败: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"检查取货状态出错: {e}")


    def post_back_to_station(self, msg):
        """
        处理回到站点的消息
        """
        if msg.data == 'true':
            self.status = 'idle'
            self.status_publisher.publish(String(data=self.status))
            self.battery = 100

            url = f"{self.base_url}/back"
            try:
                response = requests.post(
                    url,
                    json={
                        "robotId": 1,
                    }
                )
                if response.status_code == 200:
                    response_data = response.json()
                    if response_data['code'] == 50051:
                        self.get_logger().info(response_data['msg'])
                    else:
                        self.get_logger().warning(f"回站信息发送异常: {response_data['msg']}")
                else:
                    response_data = response.json()
                    self.get_logger().error(f"回站信息发送失败: {response_data['msg']}")
            except Exception as e:
                self.get_logger().error(f"发送回站信息请求失败: {e}")


    def maintenance_callback(self, msg):
        """
        处理维护消息的回调函数
        :param msg: 包含维护信息的消息
        """
        if self.status == 'busy':
            self.post_maintenance(msg.data)
            self.status = 'maintenance'
            self.status_publisher.publish(String(data=self.status))
            # 创建定时器，每5秒检查修理状态
            self.repair_check_timer = self.create_timer(5.0, self.post_repair_status)


    def post_maintenance(self, part):
        """
        发布故障信息
        :param msg: 包含维护信息的消息
        """
        url = f"{self.base_url}/maintenance"
        try:
            response = requests.post(
                url,
                json={
                    "robotId": 1,
                    "maintenanceType": part
                }
            )
        except Exception as e:
            self.get_logger().error(f"发送故障信息请求失败: {e}")


    def post_repair_status(self):
        """
        检查修理状态
        """
        url = f"{self.base_url}/getIsRepair"
        try:
            response = requests.post(
                url, json={
                    "robotId": 1
                    }
                )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50091 and response_data['data']:
                    # 机器人修好，恢复工作状态
                    self.status = 'busy'
                    self.status_publisher.publish(String(data=self.status))
                    # 取消定时器
                    if self.repair_check_timer:
                        self.repair_check_timer.cancel()
                        self.repair_check_timer = None
            else:
                self.get_logger().warning(f"检查修理状态失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"检查修理状态出错: {e}")

def main():
    rclpy.init()
    client = HttpClientNode('http://120.46.199.126:8080/robot')
    # client = HttpClientNode('http://localhost:8000/robot')
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()