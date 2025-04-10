#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        # /initialpose 토픽에 발행 (큐 사이즈 10)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 타이머를 통해 1초 후에 한 번 발행하도록 설정 (한 번만 발행)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return  # 이미 발행한 경우 재발행하지 않음

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # 또는 사용중인 프레임 ID (예: "odom")

        # 초기 위치 설정 (예: (0, 0) 위치, 회전 없음)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # covariance 값 (6x6 행렬을 36 요소 리스트로 표현)
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.25, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.06853891945200942
        ]

        self.publisher_.publish(msg)
        self.get_logger().info("Initial pose published.")
        self.published = True  # 한 번 발행 후 재발행 방지

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
