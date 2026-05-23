import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualSafetyNode(Node):
    def __init__(self):
        super().__init__('visual_safety_node')
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        # 发布到 /cmd_vel，我们将在逻辑里“掐断”速度
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 1. 将 ROS 图像转为 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 2. 简单的“避障逻辑”：计算画面下半部分的平均亮度/颜色
        # 如果前方有大障碍物，画面下半部通常会变暗或颜色单一
        height, width, _ = cv_image.shape
        roi = cv_image[int(height*0.7):height, :] # 只看画面最底部 30%
        
        avg_intensity = np.mean(roi)
        
        # 3. 逻辑判定：如果太近了（假设亮度低于 50 或识别到特定色块）
        if avg_intensity < 50: 
            self.get_logger().warn("障碍物过近！紧急制动！")
            stop_msg = Twist() # 全 0 指令
            self.publisher.publish(stop_msg)

def main():
    rclpy.init()
    node = VisualSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
