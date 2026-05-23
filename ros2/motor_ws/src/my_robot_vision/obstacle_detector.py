import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class AntiCollisionNode(Node):
    def __init__(self):
        super().__init__('anti_collision_node')
        # 确认话题名称是否为 /image_raw，v4l2_camera 默认发布这个
        self.sub = self.create_subscription(Image, '/image_raw', self.process_image, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.get_logger().info(">>> 视觉防护盾已激活，正在监听 /image_raw ...")

    def process_image(self, msg):
        try:
            # ROS图像转OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = frame.shape
            
            # 划定底部危险区
            danger_zone = frame[int(h*0.8):h, :]
            gray = cv2.cvtColor(danger_zone, cv2.COLOR_BGR2GRAY)
            score = np.mean(gray)

            # 阈值判断：如果过暗（被挡住）
            if score < 50:
                self.get_logger().warn(f"！！！检测到障碍物 (特征值: {score:.1f})，强制停车！！！")
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.pub.publish(stop_cmd)
        except Exception as e:
            self.get_logger().error(f"处理出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = AntiCollisionNode()
    try:
        # 【关键点】这里必须 spin，程序才不会退出
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
