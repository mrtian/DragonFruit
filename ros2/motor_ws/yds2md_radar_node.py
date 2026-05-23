import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import numpy as np
import time
import threading

class YdsLidarPerfectSyncDriver(Node):
    def __init__(self):
        super().__init__('yds_lidar_final_driver')
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # 绑定串口
        self.ser = serial.Serial('/dev/ttyS6', 115200, timeout=0.01)
        self.lock = threading.Lock()

        # 360个标准几何槽位
        self.ranges = [float('inf')] * 360
        self.last_start_deg = 0.0
        self.valid_point_counter = 0
        self.circle_start_time = self.get_clock().now()

        self.running = True
        self.data_thread = threading.Thread(target=self.serial_recv_loop, daemon=True)
        self.data_thread.start()

        self.get_logger().info("🚀 终极真理版雷达驱动已就绪（含硬件零位补偿）！")

    def serial_recv_loop(self):
        cache = bytearray()
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    cache.extend(self.ser.read(self.ser.in_waiting))
                else:
                    time.sleep(0.001)
                    continue

                # 严格基于物理协议的动态断包逻辑（解决闪烁的核心）
                while len(cache) >= 10: 
                    if cache[0] == 0xAA and cache[1] == 0x55:
                        lsn = cache[3]   # 真实采样点数
                        
                        if lsn == 0 or lsn > 64:
                            del cache[0]
                            continue
                            
                        expected_len = 10 + lsn * 2  # 动态计算当前帧真实总长度
                        
                        if len(cache) < expected_len:
                            break  # 字节还没收全，等待
                        
                        packet = cache[:expected_len]
                        del cache[:expected_len]
                        self.parse_hardware_packet(packet, lsn)
                    else:
                        del cache[0]
            except Exception:
                time.sleep(0.01)    

    def parse_hardware_packet(self, pack, lsn):
        try:
            # 识别硬件零位同步包
            is_zero_packet = (pack[2] & 0x01) == 0x01

            # 💡 恢复唯一正确的官方角度解析公式
            raw_start_angle = (pack[5] << 8) | pack[4]
            raw_end_angle = (pack[7] << 8) | pack[6]

            start_deg = (raw_start_angle >> 1) / 64.0
            end_deg = (raw_end_angle >> 1) / 64.0

            # 触发投递机制
            if is_zero_packet:
                with self.lock:
                    if self.valid_point_counter > 100:
                        self.publish_synchronized_scan()
                self.circle_start_time = self.get_clock().now()
                self.ranges = [float('inf')] * 360
                self.valid_point_counter = 0

            if lsn <= 1:
                return

            deg_diff = end_deg - start_deg
            if deg_diff < 0: 
                deg_diff += 360.0
            
            STEP = deg_diff / float(lsn - 1)
            data_offset = 10

            # 💡 核心标定参数（出厂零位偏差补偿）
            # 根据你实测的结果，雷达内部光电码盘距离外壳正前方偏差约 24 度
            HARDWARE_ZERO_OFFSET = 22.5

            with self.lock:
                for i in range(lsn):
                    idx = data_offset + i * 2
                    if idx + 1 >= len(pack): break

                    raw_distance = (pack[idx+1] << 8) | pack[idx]
                    distance_mm = raw_distance / 4.0

                    if 150 < distance_mm < 6000:
                        exact_angle = (start_deg + (i * STEP)) % 360.0
                        
                        # 180度基础翻转 + 内部光电码盘偏差补偿
                        point_angle = int(exact_angle + 180.0 + HARDWARE_ZERO_OFFSET) % 360

                        self.ranges[point_angle] = distance_mm / 1000.0
                        self.valid_point_counter += 1

        except Exception:
            pass

    def publish_synchronized_scan(self):
        now_time = self.get_clock().now()
        scan_msg = LaserScan()

        scan_msg.header.stamp = self.circle_start_time.to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * np.pi * (359.0 / 360.0)
        scan_msg.angle_increment = (2 * np.pi) / 360.0

        duration = (now_time.nanoseconds - self.circle_start_time.nanoseconds) / 1e9
        if duration <= 0 or duration > 0.5: duration = 0.1

        scan_msg.scan_time = duration
        scan_msg.time_increment = duration / 360.0

        scan_msg.range_min = 0.15
        scan_msg.range_max = 6.0

        scan_msg.ranges = self.ranges[:]
        scan_msg.intensities = [100.0] * 360

        self.scan_pub.publish(scan_msg)

def main():
    rclpy.init()
    node = YdsLidarPerfectSyncDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
