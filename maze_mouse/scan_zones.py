import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def zone_min(ranges, center_deg, half_width_deg=20):
    """取以 center_deg 为中心、±half_width_deg 扇区内的最小有效距离。
    滤除 inf / nan / 过近的无效读数。"""
    n = len(ranges)
    vals = []
    for d in range(-half_width_deg, half_width_deg + 1):
        r = ranges[(center_deg + d) % n]
        if math.isfinite(r) and r > 0.05:
            vals.append(r)
    return min(vals) if vals else float('inf')


class ScanZones(Node):
    """雷达分区处理 + 反应式避障：
    360° /scan 划为 前(0°) 左(90°) 右(270°) 三扇区，取最近距离；
    前方 < 安全距离 -> 原地转向较空旷一侧；
    否则直行，并按左右距离差微调保持走廊居中。"""

    def __init__(self):
        super().__init__('scan_zones')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 雷达数据用 SensorData QoS（Best Effort），与 Gazebo 发布端匹配
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.control)  # 10 Hz
        self.front = self.left = self.right = float('inf')
        self.safe = 0.45  # 前方安全距离 (m)
        self.get_logger().info('雷达分区避障启动')

    def scan_cb(self, msg):
        self.front = zone_min(msg.ranges, 0)     # 正前方 ±20°
        self.left = zone_min(msg.ranges, 90)     # 正左方 ±20°
        self.right = zone_min(msg.ranges, 270)   # 正右方 ±20°

    def control(self):
        msg = Twist()
        if self.front < self.safe:
            # 前方有障碍：原地转向较空旷的一侧
            msg.angular.z = 0.8 if self.left > self.right else -0.8
        else:
            err = self.left - self.right
            if not math.isfinite(err):   # 左右都无墙时差值为 nan，置零避免乱转
                err = 0.0
            msg.linear.x = 0.15
            msg.angular.z = max(-0.6, min(0.6, 0.5 * err))

def main(args=None):
    rclpy.init(args=args)
    node = ScanZones()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
