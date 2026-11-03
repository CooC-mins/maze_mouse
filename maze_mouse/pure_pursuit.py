import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from maze_mouse.p_control import yaw_from_quaternion, normalize_angle


class PurePursuit(Node):
    """Pure Pursuit 纯跟踪：沿预先规划的全局路径走出迷宫。
    与左墙跟随（反应式、无需地图）形成对比：
    PP 属于跟踪式，需要已知路径，但轨迹更平滑、速度更快。
    算法：在路径上取前视距离 Ld 处的目标点，按车头与目标点夹角 alpha
    计算曲率 kappa = 2*sin(alpha)/Ld，角速度 w = v*kappa。
    目标点索引单调递增，防止机器人回头找旧点。"""

    def __init__(self):
        super().__init__('pure_pursuit')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control)   # 20 Hz

        # 预先规划的全局路径：入口 -> 南墙 -> 西墙 -> 北墙 -> 出口
        self.path = [(1.4, 0.5), (0.9, 0.4), (0.4, 0.38), (0.38, 1.0),
                     (0.38, 5.5), (1.0, 5.6), (4.6, 5.6), (4.6, 6.6)]
        self.idx = 0
        self.v = 0.20        # 跟踪速度 (m/s)
        self.ld = 0.25       # 前视距离 (m)
        self.x = self.y = self.yaw = 0.0
        self.has_odom = False
        self.done = False
        self.t0 = None
        self.get_logger().info('Pure Pursuit 启动：沿规划路径跟踪至出口')

    def odom_cb(self, msg):
        self.has_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.t0 is None:
            self.t0 = self.get_clock().now()

    def find_target(self):
        """从上次目标索引起向后找第一个距离 >= Ld 的路径点（前视点）"""
        while self.idx < len(self.path) - 1 and math.hypot(
                self.path[self.idx][0] - self.x,
                self.path[self.idx][1] - self.y) < self.ld:
            self.idx += 1
        return self.path[self.idx]

    def control(self):
        if not self.has_odom:
            return
        if self.done:
            self.pub.publish(Twist())
            return

        # 终点/出口判定
        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) < 0.25 or self.y > 6.2:
            t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
            self.get_logger().info(f'===== Pure Pursuit 到达出口！总用时 {t:.1f} s =====')
            self.done = True
            self.pub.publish(Twist())
            return

        tx, ty = self.find_target()
        alpha = normalize_angle(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        w = 2.0 * self.v * math.sin(alpha) / self.ld
        w = max(-1.5, min(1.5, w))      # 角速度限幅

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
