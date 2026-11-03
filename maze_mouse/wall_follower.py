import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from maze_mouse.scan_zones import zone_min                          # 复用雷达分区
from maze_mouse.p_control import yaw_from_quaternion, normalize_angle  # 复用姿态换算


class WallFollower(Node):
    """左墙跟随走迷宫（左手定则）：
    ENTER  从入口外直线驶入迷宫（此刻左侧无墙，不能直接进入跟随逻辑）
    FOLLOW 沿左墙直行，P 控制保持目标距离
    TURN   前方有墙 -> 原地右转至通畅（处理凹墙角）
    PIVOT  左墙消失 -> 原地左转约 70°（处理凸墙角/出口缺口）
    SEEK   转角后斜切直行找墙（70° 斜角避免车头扫到缺口边缘）
    DONE   里程计越过出口线 y=6 -> 停车并打印用时
    迷宫所有内墙与外墙相连，左手定则保证从入口必达出口。"""

    ENTER, FOLLOW, TURN, PIVOT, SEEK, DONE = \
        'ENTER', 'FOLLOW', 'TURN', 'PIVOT', 'SEEK', 'DONE'

    def __init__(self):
        super().__init__('wall_follower')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.control)   # 10 Hz

        self.state = self.ENTER
        self.front = self.left = self.right = float('inf')
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.has_scan = False    # 传感器失效保护：无数据不动车
        self.has_odom = False

        self.target = 0.35       # 与左墙目标距离 (m)
        self.safe_front = 0.40   # 前方障碍判定 (m)
        self.clear_front = 0.55  # 前方恢复通畅判定 (m)，与 safe 形成迟滞防抖动
        self.lost = 0.70         # 左墙丢失判定 (m)
        self.found = 0.50        # 左墙找回判定 (m)
        self.v = 0.16            # 巡墙速度 (m/s)
        self.kp = 1.6            # 贴墙 P 增益
        self.pivot_angle = 1.22  # 凸角处原地左转约 70° (rad)
        self.pivot_sum = 0.0     # 已转角度累计
        self.last_yaw = 0.0

        self.t0 = self.get_clock().now()
        self.get_logger().info('左墙跟随启动：正在驶入迷宫入口...')

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

        # 出口判定：越过北墙线 y=6（迷宫唯一出口）
        if self.y > 6.1:
            t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            self.get_logger().info(f'===== 成功走出迷宫！总用时 {t:.1f} s =====')
            self.state = self.DONE
            self.drive(0.0, 0.0)
            return

        if self.state == self.ENTER:
            if self.y > 0.5:
                self.state = self.FOLLOW
                self.get_logger().info('已进入迷宫，开始左墙跟随')
            else:
                self.drive(self.v, 0.0)     # 直线驶入入口

        elif self.state == self.FOLLOW:
            if self.front < self.safe_front:
                self.state = self.TURN
                self.get_logger().info('前方有墙 -> 原地右转')
            elif self.left > self.lost:
                self.state = self.PIVOT
                self.pivot_sum = 0.0
                self.last_yaw = self.yaw
                self.get_logger().info('左墙丢失 -> 原地左转找缺口')
            else:
                # P 控制：与左墙偏差越大，修正角速度越大
                w = max(-1.0, min(1.0, self.kp * (self.left - self.target)))
                self.drive(self.v, w)

        elif self.state == self.TURN:
            if self.front > self.clear_front:
                self.state = self.FOLLOW
                self.get_logger().info('前方通畅 -> 恢复跟随')
            else:
                self.drive(0.0, -0.7)       # 原地右转

        elif self.state == self.PIVOT:
            # 原地左转，用里程计 yaw 累计转角，到位后进入 SEEK
            self.pivot_sum += normalize_angle(self.yaw - self.last_yaw)
            self.last_yaw = self.yaw
            if self.pivot_sum >= self.pivot_angle:
                self.state = self.SEEK
                self.get_logger().info('左转到位 -> 斜切直行找墙')
            else:
                self.drive(0.0, 0.6)

        elif self.state == self.SEEK:
            if self.left < self.found:
                self.state = self.FOLLOW
                self.get_logger().info('找回左墙 -> 恢复跟随')
            elif self.front < self.safe_front:
                self.state = self.TURN
                self.get_logger().info('找墙途中遇障 -> 原地右转')
            else:
                self.drive(self.v, 0.0)

        self.get_logger().info(
            f'[{self.state}] 前:{self.front:.2f} 左:{self.left:.2f} '
            f'位置:({self.x:.2f},{self.y:.2f})',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
