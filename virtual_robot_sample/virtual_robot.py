import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry  # 修正: Odometry は nav_msgs.msg にある
import numpy as np
import time

class VirtualRobotNode(Node):
    def __init__(self):
        super().__init__('virtual_robot_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_robot_state)

        # 仮想ロボットの状態
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vtheta = 0.0
        self.last_time = time.time()

        # クォータニオンの初期化（無回転状態）
        self.q_orientation = Quaternion()
        self.q_orientation.w = 1.0  # 初期姿勢（無回転）

        # Odometry メッセージの初期化（フレームIDは "odom"）
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"

    def cmd_vel_callback(self, msg):
        """ /cmd_vel を受信し、速度指令を更新 """
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z

    def update_robot_state(self):
        """ 一定時間ごとにロボットの状態を更新し、/odom に公開 """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # クォータニオンをオイラー角に変換して現在のヨー角を取得
        yaw = self.get_yaw_from_quaternion(self.q_orientation)

        # 状態更新 (差動二輪用)
        delta_x = self.vx * np.cos(yaw) * dt
        delta_y = self.vx * np.sin(yaw) * dt
        delta_theta = self.vtheta * dt  # 角速度 * 時間 = 角変位

        self.x += delta_x
        self.y += delta_y

        # 新しいクォータニオンを計算
        self.q_orientation = self.create_quaternion_from_yaw(yaw + delta_theta)

        # Odometry メッセージの更新
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = self.q_orientation
        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_msg.twist.twist.angular.z = self.vtheta

        # Odometry メッセージをパブリッシュ
        self.publisher.publish(self.odom_msg)
        self.get_logger().info(f"Odometry: x={self.x:.2f}, y={self.y:.2f}, yaw={yaw:.2f}")

    def get_yaw_from_quaternion(self, quaternion):
        """ クォータニオンをヨー角に変換 """
        siny_cosp = 2 * (quaternion.w * quaternion.z)
        cosy_cosp = 1 - 2 * (quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def create_quaternion_from_yaw(self, yaw):
        """ ヨー角からクォータニオンを生成 """
        q = Quaternion()
        q.w = np.cos(yaw / 2)
        q.z = np.sin(yaw / 2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
