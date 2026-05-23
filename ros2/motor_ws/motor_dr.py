import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu 
import serial
import struct
import math


class SteadyMotorDriver(Node):
    def __init__(self):
        super().__init__('steady_motor_driver')

        # --- 核心安全参数调节 ---
        self.ratio = 0.835        
        self.max_rpm = 80.0      
        self.accel = 2.0          # 每10ms的变化量 (适用于底盘线速度/角速度)
        self.sync_diff_limit = 10 
        
        #IMU数据校准及滤波
        self.static_count = 0 
        self.gyro_offset = 0.0 
        self.gyro_alpha = 0.3       
        self.filtered_gyro = 0.0
        self.yaw_kp = 0.05          

        self.imu_pub = self.create_publisher(Imu, '/motor_imu', 10)
        self.feedback_pub = self.create_publisher(Twist, '/motor_feedback', 10)

        self.ports = {'L': '/dev/ttyS4', 'R': '/dev/ttyS3'}
        self.uarts = {}
        self.data = {
            'L': {'current': 0.0, 'real': 0, 'gyroZ': 0},
            'R': {'current': 0.0, 'real': 0, 'gyroZ': 0}
        }

        # 新增：保存底盘运动学维度的目标与当前速度
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0

        for side, path in self.ports.items():
            try:
                self.uarts[side] = serial.Serial(path, 115200, timeout=0)
                self.get_logger().info(f"{side}侧稳健模式初始化成功")
            except Exception as e:
                self.get_logger().error(f"无法开启 {side} 串口: {e}")

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_timer(0.01, self.control_loop) # 100Hz 
        self.create_timer(0.1, self.publish_feedback)
        self.create_timer(0.2, self.status_report) # 5Hz 

    def cmd_callback(self, msg):
        """ 解析指令到运动学坐标 """
        self.target_v = msg.linear.x * self.max_rpm
        self.target_w = msg.angular.z * 100.0

    def control_loop(self):
        """
        100Hz 核心控制循环
        """
        # ==========================================
        # 1. 运动学层：平滑底盘整体速度 (彻底解决停车时单轮蠕动扭动)
        # ==========================================
        if self.current_v < self.target_v:
            self.current_v = min(self.current_v + self.accel, self.target_v)
        elif self.current_v > self.target_v:
            self.current_v = max(self.current_v - self.accel, self.target_v)

        if self.current_w < self.target_w:
            self.current_w = min(self.current_w + self.accel, self.target_w)
        elif self.current_w > self.target_w:
            self.current_w = max(self.current_w - self.accel, self.target_w)

        # ==========================================
        # 2. 物理转换层：【修复左右转反了】
        # 这里将 w 的加减号颠倒，左轮加w，右轮减w
        # ==========================================
        base_L = self.current_v + self.current_w
        base_R = self.current_v - self.current_w

        # ==========================================
        # 3. 物理死区层：仅作用于发送前的最终命令
        # ==========================================
        def apply_physics(val):
            # 干脆利落的停车判定：如果摇杆松开(目标都为0)且当前已经减速到了死区边缘，直接斩断输出
            if self.target_v == 0 and self.target_w == 0 and abs(val) < 10.0:
                return 0.0
            
            # 起步阶跃补偿扭矩
            if val != 0 and abs(val) < 50.0:
                return 50.0 if val > 0 else -50.0
                
            return max(min(val, self.max_rpm), -self.max_rpm)

        self.data['L']['current'] = apply_physics(base_L)
        self.data['R']['current'] = apply_physics(base_R)

        # ==========================================
        # 4. 高阶直线纠偏逻辑
        # ==========================================
        if abs(self.current_v) > 10.0 and self.target_w == 0:
            correction = self.filtered_gyro * self.yaw_kp
            self.data['L']['current'] -= correction
            self.data['R']['current'] += correction
            
            # 纠偏后二次限幅保护
            self.data['L']['current'] = max(min(self.data['L']['current'], self.max_rpm), -self.max_rpm)
            self.data['R']['current'] = max(min(self.data['R']['current'], self.max_rpm), -self.max_rpm)

        # ==========================================
        # 5. 打包发送层
        # ==========================================
        packets = {}
        for side in ['L', 'R']:
            val = self.data[side]['current']
            send_val = -val if side == 'R' else val # 右轮物理镜像取反
            cal_cmd = int(send_val * self.ratio)
            header = 0xABCD
            cs = (header ^ 0 ^ cal_cmd) & 0xFFFF
            packets[side] = struct.pack("<HhhH", header, 0, cal_cmd, cs)

        if 'L' in self.uarts: self.uarts['L'].write(packets['L'])
        if 'R' in self.uarts: self.uarts['R'].write(packets['R'])

        # 反馈解析层保持你的原样不变
        for side, ser in self.uarts.items():
            if ser.in_waiting >= 36:
                raw = ser.read(ser.in_waiting)
                idx = raw.rfind(b'\xcd\xab')
                if idx != -1 and len(raw) - idx >= 36:
                    pkt = raw[idx:idx+36]
                    try:
                        res = struct.unpack("<HHHHHhhhhhhh hhhhh H", pkt)
                        raw_gyro_z = float(res[13]) 
                        
                        target_speed = abs(self.target_v) + abs(self.target_w)
                        if target_speed == 0 and abs(self.data['L']['real']) < 5:
                            self.static_count += 1
                            if self.static_count > 50:
                                self.gyro_offset = (self.gyro_offset * 0.99) + (raw_gyro_z * 0.01)
                        else:
                            self.static_count = 0
                        
                        corrected_gyro = raw_gyro_z - self.gyro_offset
                        self.filtered_gyro = (1 - self.gyro_alpha) * self.filtered_gyro + self.gyro_alpha * corrected_gyro
                        
                        self.data[side]['real'] = -res[5] if side == 'R' else res[5]
                        self.data[side]['gyroZ'] = self.filtered_gyro

                        if side == 'L':
                            #self.publish_imu(res)
                            # 传 raw_gyro_z 给加速度，传 filtered_gyro 给角速度
                            self.publish_imu(res, self.filtered_gyro)
                    except Exception:
                        pass
    
    def publish_imu(self, res):
        #imu_msg = Imu()
        #imu_msg.header.stamp = self.get_clock().now().to_msg()
        #imu_msg.header.frame_id = "imu_link"
        #imu_msg.linear_acceleration.x = float(res[7])
        #imu_msg.angular_velocity.z = float(res[13])
        #self.imu_pub.publish(imu_msg)
        """ 发布标准的 IMU 格式 """
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = float(res[7])
        # 💡 核心：发布经过零偏校准和低通滤波后的、最干净的角速度！
        # 同样，如果它原本是度/秒，你需要乘以 (3.14159 / 180.0) 转为 弧度/秒
        imu_msg.angular_velocity.z = filtered_gyro_z * (math.pi / 180.0)

        self.imu_pub.publish(imu_msg)

    def publish_feedback(self):
        try:
            feedback_msg = Twist()
            feedback_msg.linear.x = float(self.data['L']['real'])
            feedback_msg.linear.y = float(self.data['R']['real'])
            feedback_msg.angular.z = float(self.data['L']['real']-self.data['R']['real'])
            self.feedback_pub.publish(feedback_msg)
        except Exception as e:
            self.get_logger().error(f"Feedback Error: {e}")

    def status_report(self):
        l, r = self.data['L'], self.data['R']
        diff = l['real'] - r['real']
        print(f"\r[稳健运行] L:{l['real']:>4} | R:{r['real']:>4} | 物理差值:{diff:>3} | 目标V:{self.target_v:.0f}", end='', flush=True)

def main():
    rclpy.init()
    node = SteadyMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
