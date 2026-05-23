import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math
import transforms3d

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # 10寸轮毂电机理论半径：0.127 米
        # (如果后续发现地图建大了或建小了，回来微调这个数值)
        self.wheel_radius = 0.127  

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()
        
        # 存储最新的 IMU 偏航角速度
        self.latest_gyro_z_rad = 0.0

        # 订阅电机反馈 (拿到左右轮 RPM)
        self.create_subscription(Twist, '/motor_feedback', self.feedback_callback, 10)
        # 订阅电机板发上来的 IMU
        self.create_subscription(Imu, '/motor_imu', self.imu_callback, 10)
        
        # 发布里程计与 TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("🚀 高精度双源融合里程计已启动！")

    def imu_callback(self, msg):
        # 拿到的已经是 弧度/秒 了 (假设你在驱动里已经乘以了 pi/180)
        self.latest_gyro_z_rad = msg.angular_velocity.z

    def feedback_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        
        if dt <= 0 or dt > 1.0:
            self.last_time = current_time
            return

        # 1. 提取 RPM
        rpm_left = msg.linear.x
        rpm_right = msg.linear.y

        # 2. 运动学正解：RPM 转 m/s
        v_left = rpm_left * (2.0 * math.pi * self.wheel_radius / 60.0)
        v_right = rpm_right * (2.0 * math.pi * self.wheel_radius / 60.0)
        
        vx = (v_left + v_right) / 2.0
        vy = 0.0 

        # 3. 直接使用 IMU 的角速度作为小车真实的旋转速度
        vth = self.latest_gyro_z_rad

        # 4. 里程计位置积分
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 5. 航向角转四元数
        quat = transforms3d.euler.euler2quat(0, 0, self.th) 
        
        # --- 发布 TF (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        self.tf_broadcaster.sendTransform(t)

        # --- 发布 Odometry ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
