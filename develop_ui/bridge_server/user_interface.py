#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260217_v02 (Service Client Implemented)
[Description] Firebase 명령을 받아 /start_process 서비스를 호출하는 브릿지
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time

# 메시지 및 서비스 타입 임포트
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    # [변경] UiInput 대신 RobotCommand 서비스 임포트
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    IMPORT_SUCCESS = True
except ImportError:
    print("❌ [Error] colab_interfaces 패키지를 찾을 수 없습니다. source install/setup.bash를 확인하세요.")
    IMPORT_SUCCESS = False

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface')
        
        # 1. Firebase 초기화
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info("🔥 Firebase Connected!")
            db.reference('commands').set({}) # 초기화
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        if IMPORT_SUCCESS:
            # 2. [핵심 변경] Service Client 생성 (/start_process)
            # 역할: 로봇에게 작업을 시작하라고 '요청'하는 클라이언트
            self.cli = self.create_client(RobotCommand, '/start_process')
            
            # 3. 긴급 정지 Publisher (/stop)
            # 역할: 다이어그램에 나온 대로 긴급 정지는 Topic으로 발행
            self.stop_pub = self.create_publisher(String, '/stop', 10)
            
            # 4. Subscribers (Robot -> UI)
            self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
            self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)
            self.create_subscription(SystemStatus, '/system_status', self.system_status_callback, 10)
        
        # 5. Timer & Variables
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = time.time() * 1000 
        
        self.latest_weight = 0.0
        self.latest_system_status = {}

    def loop_callback(self):
        self.check_firebase_commands()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        try:
            cmd_ref = db.reference('commands')
            cmd_data = cmd_ref.get()
            
            if cmd_data and 'timestamp' in cmd_data:
                if cmd_data['timestamp'] > self.last_command_timestamp:
                    self.last_command_timestamp = cmd_data['timestamp']
                    cmd_type = cmd_data.get('type', '')

                    # [핵심 로직] start_pouring 명령이 오면 -> 서비스 호출 (Call Service)
                    if cmd_type == 'start_pouring':
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 EMERGENCY STOP Signal Sent!")
                    
                    elif cmd_type == 'tare':
                         self.get_logger().info("⚖️ Tare Command Received")

        except Exception as e:
            self.get_logger().error(f"Command Check Error: {e}")

    def call_service_start_process(self, cmd_data):
        """ /start_process 서비스 호출 함수 """
        # 서비스 서버(로봇 제어 노드)가 켜져 있는지 1초만 기다려봄
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패. 로봇 제어 노드가 켜져 있나요?')
            return

        # 요청 데이터 채우기 (RobotCommand.srv 정의에 따름)
        req = RobotCommand.Request()
        req.mode = "FULL"  # 기본 모드
        req.target_weight = float(cmd_data.get('target_weight', 0.0))
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))

        self.get_logger().info(f"📤 서비스 요청 보냄: Target={req.target_weight}g, Mix={req.mixing_duration}s")
        
        # 비동기 호출 (결과를 기다리지 않고 바로 넘어감)
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """ 서비스 응답이 오면 실행되는 함수 """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 서비스 성공: {response.message}")
            else:
                self.get_logger().warn(f"❌ 서비스 실패: {response.message}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 에러 발생: {e}")

    def upload_to_firebase(self):
        try:
            updates = {
                'sensor_data/weight': round(self.latest_weight, 2),
                'sensor_data/timestamp': int(time.time() * 1000)
            }
            if self.latest_system_status:
                updates['system_stats'] = self.latest_system_status
                
                # [호환성] 기존 로봇 상태 경로에도 일부 데이터 업데이트
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
                updates['robot_status/velocity'] = self.latest_system_status.get('tcp_vel', 0)
                updates['robot_status/acceleration'] = self.latest_system_status.get('tcp_acc', 0)
            
            db.reference().update(updates)
        except Exception:
            pass

    def system_status_callback(self, msg):
        self.latest_system_status = {
            "phase": msg.phase,
            "tcp_vel": msg.tcp_vel,
            "tcp_acc": msg.tcp_acc,
            "pour_speed": msg.pour_speed,
            "total_count": msg.total_count,
            "success_count": msg.success_count,
            "error_rate": round(msg.error_rate, 2),
            "last_cycle_time": round(msg.last_cycle_time, 2)
        }
        
    def joint_callback(self, msg): pass
    def weight_callback(self, msg): self.latest_weight = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()