import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """四元数 -> 偏航角 yaw（平面运动只关心 yaw）"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    """把角度规范到 (-pi, pi]，保证转向走最短方向"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class PControl(Node):
    """基于 /odom 里程计的 P 闭环控制：依次到达 waypoints 中的目标点。
    策略：角度偏差大 -> 原地转向对准；角度偏差小 -> 边修方向边前进，
    线速度/角速度均与误差成正比（P 控制），并用限幅保护电机。"""

    def __init__(self):
        super().__init__('p_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control)  # 20 Hz

        # 目标点序列（迷宫入口附近的空旷区域，走一个矩形）
        self.waypoints = [(1.4, 1.0), (2.5, 1.0), (2.5, 3.0), (1.0, 3.0)]
        self.idx = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_odom = False

        self.kp_l = 0.8        # 线速度比例系数
        self.kp_a = 1.8        # 角速度比例系数
        self.v_max = 0.18      # 线速度限幅 (m/s)
        self.w_max = 1.2       # 角速度限幅 (rad/s)
        self.goal_tol = 0.08   # 到点判定半径 (m)
        self.ang_tol = 0.35    # 超过该角度偏差就先原地转向 (rad)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.has_odom = True

    def control(self):
        if not self.has_odom:
            return
        if self.idx >= len(self.waypoints):
            self.pub.publish(Twist())    # 全部到达，停车
            return

        gx, gy = self.waypoints[self.idx]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        ang_err = normalize_angle(math.atan2(dy, dx) - self.yaw)

        msg = Twist()
        if dist < self.goal_tol:
            self.get_logger().info(
                f'到达目标点 {self.idx + 1}/{len(self.waypoints)}: ({gx:.1f}, {gy:.1f})')
            self.idx += 1
        elif abs(ang_err) > self.ang_tol:
            # 角度差太大：原地转向对准目标
            msg.angular.z = max(-self.w_max, min(self.w_max, self.kp_a * ang_err))
        else:
            # 前进 + 方向微调：速度正比于误差（P 控制），限幅保护
            msg.linear.x = max(0.0, min(self.v_max, self.kp_l * dist))
            msg.angular.z = max(-self.w_max, min(self.w_max, self.kp_a * ang_err))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
