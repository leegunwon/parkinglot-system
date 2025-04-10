import sys 
import os 
import sqlite3 
import pandas as pd 

# 로봇을 불러오기 위한 ros2 메시지 발행한다
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import rclpy.action

from PyQt5.QtWidgets import QApplication, QDialog, QPushButton
from PyQt5.QtCore import Qt
from PyQt5 import uic
import math

# 현재 스크립트의 폴더 위치를 기반으로 UI 파일 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# UI 파일 경로 설정하기
SelectBoxUI = os.path.join("/home/su/navigation2_ignition_gazebo_turtlebot3-main/src/parking_bot", "UI", "select_box.ui")
PhoneNumberUI = os.path.join("/home/su/navigation2_ignition_gazebo_turtlebot3-main/src/parking_bot", "UI", "phone_number.ui")
RentalMsgBoxUI = os.path.join("/home/su/navigation2_ignition_gazebo_turtlebot3-main/src/parking_bot", "UI", "rental_msg_box.ui")
ReturnMsgBoxUI = os.path.join("/home/su/navigation2_ignition_gazebo_turtlebot3-main/src/parking_bot", "UI", "return_msg_box.ui")

# ros2 노드 초기화해준다
if not rclpy.ok():
    rclpy.init()

class ROS2Publisher(Node):
    """ros2 메시지 발행을 위한 Publisher 노드"""
    def __init__(self,namespace):
        super().__init__("kiosk_rental_request_publisher")
        # 기존 버전 : rental/request를 전달 -> parking_system에서 nav2 목적지를 전송
        # 비효율적이어서 : 그냥 직접 nav2 목적지 전송
        self._action_client = rclpy.action.ActionClient(self, NavigateToPose, f'/{namespace}/navigate_to_pose')

        # # 네임스페이스를 변수로 받아서 설정
        # super().__init__("kiosk_rental_request_publisher", namespace=namespace)
        
        # # 액션 클라이언트 생성, 네임스페이스 반영된 상태에서 액션 서버 연결
        # self._action_client = ActionClient(self, NavigateToPose, f'/{namespace}/navigate_to_pose')


    def quaternion_from_euler(self,roll, pitch, yaw):
        """
        오일러 각도 (roll, pitch, yaw)를 쿼터니언 (x, y, z, w)으로 변환
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return x, y, z, w

    def send_navigation_goal(self,d_x,d_y,d_z,roll, pitch, yaw):
        # 액션 서버가 준비될 때까지 대기 (최대 10초)

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Navigation action server is not available!")
            return

        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = d_x  # 원하는 rent 위치 (수정 필요)
        goal_pose.pose.position.y = d_y
        goal_pose.pose.position.z = d_z

        # 오일러 각도 -> 쿼터니언 변환
        quat = self.quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        goal_msg.pose = goal_pose

        self.get_logger().info("Sending navigation goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            return

        self.get_logger().info("Navigation goal accepted.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

ros2_publisher = ROS2Publisher('tb1')

class SelectBox(QDialog):
    def __init__(self):
        super().__init__()
        uic.loadUi(SelectBoxUI, self)  # UI 로드

        # 버튼 연결
        self.yes_btn.clicked.connect(lambda: self.open_phone_number("rental"))
        self.no_btn.clicked.connect(lambda: self.open_phone_number("return"))

    def open_phone_number(self, mode):
        """대여 또는 반납을 선택하면 전화번호 입력창을 연다"""
        self.phone_number_window = PhoneNumber(mode)
        self.phone_number_window.exec_()  # 모달 창으로 실행


class PhoneNumber(QDialog):
    def __init__(self, mode):
        super().__init__()
        uic.loadUi(PhoneNumberUI, self)  # UI 로드

        self.mode = mode  # "rental" 또는 "return" 모드 저장

        # 화면 설정
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint
        )  # 프레임 제거 & 항상 최상위 유지
        self.setAttribute(Qt.WA_TranslucentBackground, True)  # 배경 투명

        # 버튼 연결
        self.point_confirm_btn.clicked.connect(self.point_check)  # 확인 버튼
        self.cancel_btn.clicked.connect(self.close)  # 취소 버튼

        # 숫자 입력 버튼 설정
        self.user_num = []
        buttons = self.point_buttons.findChildren(QPushButton)
        for button in buttons:
            button.clicked.connect(self.write_point_num)

    def write_point_num(self):
        """번호 버튼을 누르면 라벨에 업데이트됨"""
        num_list = [str(num) for num in range(0, 10)] + ['010']

        btn_name = self.sender().text()
        now_label_text = self.user_number_label.text()

        if btn_name in num_list:
            now_label_text += btn_name
            only_num = now_label_text.replace('-', '')

            if len(only_num) <= 11:
                self.user_number_label.setText(self.mask_numbers(now_label_text))
        else:  # 삭제 버튼 (지우기 기능)
            now_label_text = now_label_text[:-1]
            self.user_number_label.setText(self.mask_numbers(now_label_text))

    def mask_numbers(self, number):
        """전화번호 마스킹"""
        number = number.replace('-', '')
        if len(number) <= 3:
            return number
        elif 3 < len(number) < 8:
            return f'{number[:3]}-{(len(number) - 3) * "*"}'
        else:
            return number[:3] + '-****-' + number[7:]

    def point_check(self):
        """전화번호 확인 버튼 동작"""
        now_label_text = self.user_number_label.text().replace('-', '')

        if len(now_label_text) == 11:
            # 대여 모드면 "대기 장소"로 하고 반납 모드면 "반납 장소"로 설정
            destination = "move to waiting area" if self.mode == "rental" else "move to return area"
            
            self.send_ros2_message(destination)  # 목적지 전달
            self.close()

            # 메시지 창 표시
            self.next_window = RentalMsgBox() if self.mode == "rental" else ReturnMsgBox()
            self.next_window.exec_()
        else:
            print("올바른 전화번호를 입력하세요")
            
    def send_ros2_message(self, destination):
        """ros2 메시지를 발행하여 로봇을 지정된 장소로 호출"""
        if self.mode == "rental":
            # ros2_publisher.send_navigation_goal()
            ros2_publisher.send_navigation_goal(14.2, 10.9, -0.2, 0.0, 0.0, math.radians(0))
        else : 
            ros2_publisher.send_navigation_goal(43.0, -7.0, -0.2, 0.0, 0.0, math.radians(90))


class RentalMsgBox(QDialog):
    def __init__(self):
        super().__init__()
        uic.loadUi(RentalMsgBoxUI, self)  # UI 로드
        self.no_btn.clicked.connect(self.close)  # 확인 버튼 클릭 시 닫기

class ReturnMsgBox(QDialog):
    def __init__(self):
        super().__init__()
        uic.loadUi(ReturnMsgBoxUI, self)  # UI 로드
        self.no_btn.clicked.connect(self.close)  # 확인 버튼 클릭 시 닫기

def main():
    app = QApplication(sys.argv)
    select_box = SelectBox()
    select_box.exec_()  # 초기 창 실행
    rclpy.init()  # 반드시 노드를 초기화해야 함
    # 종료 시 ros2 노드도 정리
    ros2_publisher.destroy_node()
    rclpy.shutdown()

    sys.exit(app.exec_())
