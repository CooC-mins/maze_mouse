import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoop(Node):
    """开环控制：只按时间发速度，不看任何反馈。
    流程：直行 3 s -> 原地左转约 90° -> 直行 2 s -> 停止。
    用于演示开环的误差累积（实际轨迹与预期不符），为闭环 P 控制做铺垫。"""

    def __init__(self):
        super().__init__('open_loop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control)  # 10 Hz
        self.t0 = self.get_clock().now()
        self.t_forward1 = 3.0   # 第一段直行时长(s)
        self.t_turn = 1.8       # 转向时长(s)
        self.t_forward2 = 2.0   # 第二段直行时长(s)
        self.total = self.t_forward1 + self.t_turn + self.t_forward2
        self.get_logger().info('开环动作：直行3s -> 左转 -> 直行2s -> 停止')

    def control(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        msg = Twist()
        if t < self.t_forward1:
            msg.linear.x = 0.15          # 直行
        elif t < self.t_forward1 + self.t_turn:
            msg.angular.z = 0.87         # 原地左转约 90°
        elif t < self.total:
            msg.linear.x = 0.15          # 再直行
        else:
            self.get_logger().info('开环动作结束，停车')
            self.timer.cancel()          # 停发速度（Gazebo 保持最后指令）
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
