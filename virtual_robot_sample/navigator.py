import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # パブリッシャー: /cmd_vel（速度指令）
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # サブスクライバー: /odom（現在位置）と /clicked_point（目標座標）
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_goal = self.create_subscription(PointStamped, '/clicked_point', self.goal_callback, 10)

        # ループ処理
        self.timer = self.create_timer(0.1, self.navigate)

        # 目標位置（デフォルトは未設定）
        self.target_x = None
        self.target_y = None

        # 現在位置
        self.x = 0.0
        self.y = 0.0
        self.q_orientation = Quaternion()  # クォータニオンの初期化
        self.q_orientation.w = 1.0  # 初期姿勢

    def odom_callback(self, msg):
        """ /odom を購読し、現在のロボットの状態を取得 """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # クォータニオンを取得
        self.q_orientation = msg.pose.pose.orientation

    def goal_callback(self, msg):
        """ /clicked_point を購読し、新しい目標座標を設定 """
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.get_logger().info(f"New goal received: x={self.target_x:.2f}, y={self.target_y:.2f}")

    def navigate(self):
        """ 目標座標に向かうような /cmd_vel を生成 """
        if self.target_x is None or self.target_y is None:
            return  # 目標が設定されていない場合は動作しない

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            # 目標に到達したら停止
            cmd_vel_msg = Twist()
            self.publisher.publish(cmd_vel_msg)
            self.target_x = None
            self.target_y = None
            self.get_logger().info("Target reached!")
            return

        # 目標方向の角度（2D平面上）
        target_theta = math.atan2(dy, dx)

        # クォータニオン → ヨー角変換
        current_yaw = self.get_yaw_from_quaternion(self.q_orientation)

        # 角度差を計算（yawと目標角度の差）
        angle_diff = target_theta - current_yaw

        # -π ～ π の範囲に正規化
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 制御パラメータ
        linear_speed = 0.5  # 前進速度
        angular_speed = 1.0  # 回転速度
        max_angular_speed = 1.5

        cmd_vel_msg = Twist()

        if abs(angle_diff) > 0.1:
            # 方向がずれている場合は回転
            cmd_vel_msg.angular.z = max(min(angle_diff * angular_speed, max_angular_speed), -max_angular_speed)
        else:
            # ほぼ目標方向なら前進
            cmd_vel_msg.linear.x = min(linear_speed, distance)

        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f"Navigating: x={self.x:.2f}, y={self.y:.2f}, yaw={current_yaw:.2f}")

    def get_yaw_from_quaternion(self, quaternion):
        """ クォータニオンをヨー角に変換 """
        siny_cosp = 2 * (quaternion.w * quaternion.z)
        cosy_cosp = 1 - 2 * (quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
