import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from maze_mouse.scan_zones import zone_min
from maze_mouse.p_control import yaw_from_quaternion, normalize_angle


class WallFollowerAdaptive(Node):
    """左墙跟随（速度自适应改进版）：
    状态机与基础版一致（ENTER/FOLLOW/TURN/PIVOT/SEEK/DONE），
    区别在于 FOLLOW 状态的线速度按前方距离自适应：
      前方 > 1.2 m -> 0.22 m/s（直道全速）
      前方 > 0.7 m -> 0.16 m/s（中速）
      其余         -> 0.10 m/s（接近墙角减速，保证转向安全）
    与基础版（全程 0.16 m/s）做 A/B 对比实验，验证提速效果。"""

    ENTER, FOLLOW, TURN, PIVOT, SEEK, DONE = \
        'ENTER', 'FOLLOW', 'TURN', 'PIVOT', 'SEEK', 'DONE'

    def __init__(self):
        super().__init__('wall_follower_adaptive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.control)

        self.state = self.ENTER
        self.front = self.left = self.right = float('inf')
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.has_scan = False
        self.has_odom = False

        self.target = 0.35
        self.safe_front = 0.40
        self.clear_front = 0.55
        self.lost = 0.70
        self.found = 0.50
        self.kp = 1.6
        self.pivot_angle = 1.22
        self.pivot_sum = 0.0
        self.last_yaw = 0.0

        self.t0 = self.get_clock().now()
        self.get_logger().info('自适应速度左墙跟随启动')

    def adaptive_speed(self):
        """按前方空闲距离分档调速"""
        if self.front > 1.2:
            return 0.22
        elif self.front > 0.7:
            return 0.16
        else:
            return 0.10

    def scan_cb(self, msg):
        self.has_scan = True
        self.front = zone_min(msg.ranges, 0)
        self.left = zone_min(msg.ranges, 90)
        self.right = zone_min(msg.ranges, 270)

    def odom_cb(self, msg):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def control(self):
        if not self.has_scan or not self.has_odom:
            return
        if self.state == self.DONE:
            self.drive(0.0, 0.0)
            return

        if self.y > 6.1:
            t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            self.get_logger().info(f'===== 成功走出迷宫！总用时 {t:.1f} s（自适应版）=====')
            self.state = self.DONE
            self.drive(0.0, 0.0)
            return

        if self.state == self.ENTER:
            if self.y > 0.5:
                self.state = self.FOLLOW
            else:
                self.drive(0.16, 0.0)

        elif self.state == self.FOLLOW:
            if self.front < self.safe_front:
                self.state = self.TURN
            elif self.left > self.lost:
                self.state = self.PIVOT
                self.pivot_sum = 0.0
                self.last_yaw = self.yaw
            else:
                w = max(-1.0, min(1.0, self.kp * (self.left - self.target)))
                self.drive(self.adaptive_speed(), w)   # 唯一区别：速度分档

        elif self.state == self.TURN:
            if self.front > self.clear_front:
                self.state = self.FOLLOW
            else:
                self.drive(0.0, -0.7)

        elif self.state == self.PIVOT:
            self.pivot_sum += normalize_angle(self.yaw - self.last_yaw)
            self.last_yaw = self.yaw
            if self.pivot_sum >= self.pivot_angle:
                self.state = self.SEEK
            else:
                self.drive(0.0, 0.6)

        elif self.state == self.SEEK:
            if self.left < self.found:
                self.state = self.FOLLOW
            elif self.front < self.safe_front:
                self.state = self.TURN
            else:
                self.drive(0.16, 0.0)

        self.get_logger().info(
            f'[{self.state}] 前:{self.front:.2f} 左:{self.left:.2f} '
            f'位置:({self.x:.2f},{self.y:.2f})',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerAdaptive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
