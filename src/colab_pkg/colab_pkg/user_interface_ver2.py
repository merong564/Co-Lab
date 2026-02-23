#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260223_v01 (Simplified End-to-End Testing)
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import datetime
import re  

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    IMPORT_SUCCESS = True
except ImportError:
    print("❌ [Error] colab_interfaces 패키지를 찾을 수 없습니다. source install/setup.bash를 확인하세요.")
    IMPORT_SUCCESS = False
    
ROBOT_ID = "dsr01"

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface', namespace=ROBOT_ID)
        
        try:
            cred = credentials.Certificate("/home/rokey/Co-Lab/serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info("🔥 Firebase Connected!")
            db.reference('commands').set({}) 
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        if IMPORT_SUCCESS:
            self.cli = self.create_client(RobotCommand, 'start_process')
            self.stop_pub = self.create_publisher(String, 'stop', 10)
            
            self.create_subscription(JointState, 'dsr01/joint_states', self.joint_callback, 10)
            self.create_subscription(Float32, 'load_cell/weight', self.weight_callback, 10)
            self.create_subscription(SystemStatus, 'system_status', self.system_status_callback, 10)
        
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = time.time() * 1000 
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        
        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        self.last_total_count = 0 

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

                    if cmd_type == 'start_pouring':
                        self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.current_material = cmd_data.get('material', 'Unknown')
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 EMERGENCY STOP Signal Sent!")
                    
                    elif cmd_type == 'tare':
                         self.get_logger().info("⚖️ Tare Command Received")

        except Exception as e:
            self.get_logger().error(f"Command Check Error: {e}")

    def call_service_start_process(self, cmd_data):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패. 로봇 제어 노드가 켜져 있나요?')
            return

        req = RobotCommand.Request()
        req.mode = "FULL"
        
        # [수정] 시작: 웹에서 넘겨준 재료 문자열에서 숫자를 추출해 리스트로 묶기
        match = re.search(r'무지개(\d+(\.\d+)?)/푸른색(\d+(\.\d+)?)/자갈(\d+(\.\d+)?)', self.current_material)
        if match:
            # [수정] 매칭 성공 시 무조건 LARGE가 먼저 오도록 (LARGE, SMALL1, SMALL2) 순서로 배열 생성
            # match.group(5) = 자갈, match.group(1) = 무지개, match.group(3) = 푸른색
            req.targets = ["LARGE", "SMALL1", "SMALL2"]
            req.target_weights = [float(match.group(5)), float(match.group(1)), float(match.group(3))]
        else:
            # 형식이 맞지 않을 경우의 방어 코드
            req.targets = ["LARGE"]
            req.target_weights = [float(cmd_data.get('target_weight', 0.0))]
        # [수정] 끝
        
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))

        # [수정] 로그 출력 문구를 리스트 형태에 맞게 변경
        self.get_logger().info(f"📤 서비스 요청 보냄: Targets={req.targets}, Weights={req.target_weights}g, Mix={req.mixing_duration}s")
        
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
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
                
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
                updates['robot_status/velocity'] = self.latest_system_status.get('tcp_vel', 0)
                updates['robot_status/acceleration'] = self.latest_system_status.get('tcp_acc', 0)
            
            db.reference().update(updates)
        except Exception:
            pass

    def system_status_callback(self, msg):
        if hasattr(self, 'last_total_count') and msg.total_count > self.last_total_count:
            if self.last_total_count != 0: 
                self.save_experiment_history(msg)
            self.last_total_count = msg.total_count

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
        
    def save_experiment_history(self, msg):
        try:
            target_w = self.current_target_weight
            final_w = round(self.latest_weight, 2)
            
            ss_error_g = round(abs(target_w - final_w), 2)
            error_rate = round(msg.error_rate, 2)

            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,
                'error_rate': error_rate,
                'success': True if error_rate <= 10.0 else False,
                'ss_error_g': ss_error_g,
                'cycle_time': round(msg.last_cycle_time, 2)
            }
            
            db_ref = db.reference('experiment_history')
            new_record = db_ref.push(history_data)
            
            self.get_logger().info(f"💾 [DB 저장 성공] ID: {new_record.key} | 오차: {error_rate}%")
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

    def joint_callback(self, msg): 
        pass
        
    def weight_callback(self, msg): 
        self.latest_weight = msg.data

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