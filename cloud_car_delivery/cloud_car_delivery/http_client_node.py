import json
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class HttpClientNode(Node):
    def __init__(self, base_url):
        super().__init__('http_client_node')
        
        self.delivery_publisher = self.create_publisher(String, '/delivery_address', 10)
        self.status_publisher = self.create_publisher(String, '/status', 10)
        self.image_subscription = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 10)
        self.position_subscription = self.create_subscription(String, '/robot_position', self.position_callback, 10)
        self.arrived_subscription = self.create_subscription(String, '/arrived', self.arrived_callback, 10)
        self.base_url = base_url
        self.bridge = CvBridge()
        self.current_image = None
        self.status = 'idle'
        self.battery = 100
        self.position = {
            "longitude": "112.945295",  # 经度
            "latitude": "28.164779"  # 纬度
        }

        # 创建定时器，每5秒发送一次请求
        self.timer = self.create_timer(5.0, self.post_state)
        # 创建定时器,每20秒电量下降1.0
        self.battery_timer = self.create_timer(20.0, self.decrease_battery)
        self.get_logger().info('HTTP客户端节点已启动')

    def arrived_callback(self, msg):
        if msg.data == 'true':
            self.post_arrived()

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
                    "isArrived": True
                }
            )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50031:
                    self.get_logger().info(response_data['msg'])
                else:
                    self.get_logger().warning(f"到达信息发送异常: {response_data['msg']}")
            else:
                response_data = response.json()
                self.get_logger().error(f"到达信息发送失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"发送到达信息请求失败: {e}")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        _, buffer = cv2.imencode('.jpg', cv_image)
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
                if response.status_code == 200:
                    response_data = response.json()
                    if response_data['code'] == 50011:
                        self.get_logger().info(response_data['msg'])
                    else:
                        self.get_logger().warning(f"图像发送异常: {response_data['msg']}")
                else:
                    response_data = response.json()
                    self.get_logger().error(f"图像发送失败: {response_data['msg']}")
            except Exception as e:
                self.get_logger().error(f'发送图像时出错: {str(e)}')

    def decrease_battery(self):
        """
        每20秒电量下降1
        """
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
            self.get_logger().info(f'更新机器人位置: 纬度={self.position["latitude"]}, 经度={self.position["longitude"]}')
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
                    "battery": self.battery
                }
            )
            if response.status_code == 200:
                response_data = response.json()
                if response_data['code'] == 50021:
                    data = response_data['data']
                    # 发布配送地址
                    if data['delivery_address'] is not None:
                        address_msg = String()
                        address_msg.data = data['delivery_address']
                        self.delivery_publisher.publish(address_msg)
                        self.get_logger().info(f'已发布配送地址: {address_msg.data}')

                    # 发布状态
                    status_msg = String()
                    status_msg.data = data['status']
                    if status_msg.data != self.status:
                        self.status = status_msg.data
                        self.status_publisher.publish(status_msg)
                        self.get_logger().info(f'已发布状态: {status_msg.data}')
                    # 如果状态为idle，则恢复电量
                    if status_msg.data == 'idle':
                        self.battery = 100

                else:
                    self.get_logger().warning(f"状态发送异常: {response_data['msg']}")
            else:
                response_data = response.json()
                self.get_logger().error(f"状态发送失败: {response_data['msg']}")
        except Exception as e:
            self.get_logger().error(f"发送状态请求失败: {e}")

def main():
    rclpy.init()
    client = HttpClientNode('http://localhost:8000/robot')
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 