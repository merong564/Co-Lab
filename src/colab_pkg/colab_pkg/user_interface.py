#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260226_v07 (Strict Core Preservation + Phase Force Shield)
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import datetime
import re  
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    from colab_interfaces.msg import ControlMetrics  
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
            self.stop_pub = self.create_publisher(String, 'stop/ui', 10)
            
            self.create_subscription(JointState, 'dsr01/joint_states', self.joint_callback, 10)
            self.create_subscription(Float32, 'load_cell/weight', self.weight_callback, 10)
            self.create_subscription(SystemStatus, 'system_status', self.system_status_callback, 10)
            
            self.create_subscription(ControlMetrics, 'log_control_metrics', self.control_metrics_callback, 10)
        
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        
        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        self.last_total_count = 0 
        self.is_first_msg = True 

        # 💡 [추가 기능 1] 무게 토픽을 완전히 잠가버리는 영구 차단 플래그
        self.is_weight_blocked = False

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
                        ui_time = cmd_data.get('timestamp', 0)
                        self.get_logger().info(f'▶ 작업 시작(Start) 신호 수신됨 (UI 클릭 시간: {ui_time})')
                        
                        # 💡 [추가] 새 작업 시작 시 차단 스위치 해제
                        self.is_weight_blocked = False

                        self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.current_material = cmd_data.get('material', 'Unknown')
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 EMERGENCY STOP Signal Sent!")
                    
                    elif cmd_type == 'tare':
                         # 💡 [추가] 영점 조절 시 차단 스위치 해제
                         self.is_weight_blocked = False
                         self.get_logger().info("⚖️ Tare Command Received (Weight Block Released)")

        except Exception as e:
            self.get_logger().error(f"Command Check Error: {e}")

    def call_service_start_process(self, cmd_data):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패. 로봇 제어 노드가 켜져 있나요?')
            return

        req = RobotCommand.Request()
        req.mode = "FULL"
        
        # 💡 [추가 기능 2] 에탄올, 아세톤, 물 포맷에 맞춰 정규표현식 변경 (기존 로직 유지)
        match = re.search(r'에탄올(\d+(\.\d+)?)/아세톤(\d+(\.\d+)?)/물(\d+(\.\d+)?)', self.current_material)
        if match:
            req.targets = ["LARGE", "SMALL1", "SMALL2"]
            req.target_weights = [float(match.group(5)), float(match.group(1)), float(match.group(3))]
        else:
            req.targets = ["LARGE"]
            req.target_weights = [float(cmd_data.get('target_weight', 0.0))]
        
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))

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
        except Exception as e:
            self.get_logger().error(f"❌ 파이어베이스 업로드 실패: {e}")

    def system_status_callback(self, msg):
        if getattr(self, 'is_first_msg', True):
            self.last_total_count = msg.total_count
            self.is_first_msg = False
        elif msg.total_count > self.last_total_count:
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
            local_error_rate = round(msg.error_rate, 2)

            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,
                'success': True if local_error_rate <= 10.0 else False,
                'ss_error_g': ss_error_g,
                'cycle_time': round(msg.last_cycle_time, 2)
            }
            
            db_ref = db.reference('experiment_history')
            new_record = db_ref.push(history_data)
            
            self.get_logger().info(f"💾 [DB 저장 성공 - 기본 히스토리] ID: {new_record.key}")
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

    def control_metrics_callback(self, msg):
        try:
            current_pour_speed = round(self.latest_system_status.get('pour_speed', 0.0), 2)
            current_error_rate = round(self.latest_system_status.get('error_rate', 0.0), 2)

            metrics_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'pour_speed': current_pour_speed,  
                'error_rate': current_error_rate,   
                'p_gain': round(msg.p_gain, 4),
                'd_gain': round(msg.d_gain, 4),
                'max_tilt_step': round(msg.max_tilt_step, 2),
                'stop_threshold': round(msg.stop_threshold, 2),
                'p_d_ratio': round(msg.p_d_ratio, 2) if hasattr(msg, 'p_d_ratio') else 0.0,
                'overshoot': round(msg.overshoot, 2),
                'rise_time': round(msg.rise_time, 2),
                'settling_time': round(msg.settling_time, 2),
                'ss_error': round(msg.ss_error, 2)
            }
            
            db_ref = db.reference('control_metrics_history')
            # 💡 [오류 수정] 이전 코드의 push() NameError를 방지하기 위해 db_ref.push()로 수정 유지
            new_record = db_ref.push(metrics_data) 
            
            self.get_logger().info(f"📊 [제어 지표 DB 저장 완료] Pour Speed & Error Rate 포함됨. ID: {new_record.key}")

            self.latest_system_status['max_tilt_step'] = round(msg.max_tilt_step, 2)
            self.latest_system_status['stop_threshold'] = round(msg.stop_threshold, 2)

        except Exception as e:
            self.get_logger().error(f"❌ DB 제어 지표 저장 실패: {e}")

    def joint_callback(self, msg): 
        # 💡 [오류 수정] 문법 에러(for msg.position) 방지를 위해 rad in msg.position 유지
        self.latest_joints = [math.degrees(rad) for rad in msg.position]
        self.last_joint_time = time.time()

    def weight_callback(self, msg): 
        # 💡 [추가 기능 연동] 대소문자 관계없이 완벽 인식
        current_phase = self.latest_system_status.get('phase', 'Ready').lower()
        
        # 믹싱 및 복귀 단계에 진입하면 '영구 차단 스위치'를 켭니다.
        if current_phase in ["mixing", "return"]:
            self.is_weight_blocked = True
            
        # 🛡️ [핵심 방어 로직] 차단 스위치가 켜져 있으면 데이터 무시하고 0.0 전달
        if self.is_weight_blocked:
            self.latest_weight = 0.0
            # 차단 모드에서는 로그도 띄우지 않고 가짜 신호를 완벽히 무시합니다.
        else:
            self.latest_weight = float(msg.data)
            self.get_logger().info(f"📡 [UI 브릿지 수신]: {self.latest_weight:.1f} g")

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