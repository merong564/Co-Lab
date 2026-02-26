#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260226_v12 (Cumulative Weight Fix + DB Analytics)
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import datetime
import re  
import os

# from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    from colab_interfaces.msg import ControlLive, ControlResult # [수정] 분리된 메시지 임포트
    IMPORT_SUCCESS = True
except ImportError:
    print("[Error] colab_interfaces 패키지를 찾을 수 없습니다. source install/setup.bash를 확인하세요.")
    IMPORT_SUCCESS = False
    
ROBOT_ID = "dsr01"

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface', namespace=ROBOT_ID)
        
        try:
            FIREBASE_CRED_PATH="/home/monn/Co-Lab/serviceAccountKey.json"  # 사용자 이름 각자 PC에 맞게 수정

            cred = credentials.Certificate(FIREBASE_CRED_PATH)
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info("Firebase Connected!")
            db.reference('commands').set({}) 
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        if IMPORT_SUCCESS:
            self.cli = self.create_client(RobotCommand, 'start_process')
            self.stop_pub = self.create_publisher(String, 'stop/ui', 10)
            
            # self.create_subscription(JointState, 'dsr01/joint_states', self.joint_callback, 10)
            self.create_subscription(Float32, 'load_cell/weight', self.weight_callback, 10)
            self.create_subscription(SystemStatus, 'system_status', self.system_status_callback, 10)
            
            # [수정] Live와 Result 메시지 구독 분리
            self.create_subscription(ControlLive, 'log_control_live', self.control_live_callback, 10)
            self.create_subscription(ControlResult, 'log_control_result', self.control_result_callback, 10)
        
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        
        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        self.is_weight_blocked = False

        # 지표 계산용 변수
        self.cycle_start_time = 0.0
        self.current_cycle_time = 0.0
        self.recipe_failed_flag = False
        self.current_recipe_max_error_rate = 0.0 

        # [추가] 연속 붓기 누적 무게 처리를 위한 변수
        self.base_offset = 0.0
        self.last_raw_weight = 0.0
        self.final_cumulative_weight = 0.0

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
                        self.get_logger().info('작업 시작(Start) 신호 수신됨')
                        self.is_weight_blocked = False

                        self.cycle_start_time = time.time()
                        self.current_cycle_time = 0.0
                        self.recipe_failed_flag = False
                        self.current_recipe_max_error_rate = 0.0 

                        # [추가] 새 작업 시작 시 누적 무게 초기화
                        self.base_offset = 0.0
                        self.last_raw_weight = 0.0
                        self.final_cumulative_weight = 0.0

                        self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.current_material = cmd_data.get('material', 'Unknown')
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("EMERGENCY STOP Signal Sent!")
                        self.recipe_failed_flag = True 
                    
                    elif cmd_type == 'tare':
                         self.is_weight_blocked = False
                         # [추가] 강제 영점 조절 시 누적 무게 초기화
                         self.base_offset = 0.0
                         self.last_raw_weight = 0.0
                         self.final_cumulative_weight = 0.0

        except Exception as e:
            self.get_logger().error(f"Command Check Error: {e}")

    def call_service_start_process(self, cmd_data):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('서비스(/start_process) 연결 실패.')
            return

        req = RobotCommand.Request()
        req.mode = "FULL"
        
        match = re.search(r'에탄올(\d+(\.\d+)?)/아세톤(\d+(\.\d+)?)/물(\d+(\.\d+)?)', self.current_material)
        if match:
            req.targets = ["LARGE", "SMALL1", "SMALL2"]
            req.target_weights = [float(match.group(5)), float(match.group(1)), float(match.group(3))]
        else:
            req.targets = ["LARGE"]
            req.target_weights = [float(cmd_data.get('target_weight', 0.0))]
        
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))
        
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            
            if self.cycle_start_time > 0:
                self.current_cycle_time = time.time() - self.cycle_start_time

            if response.success:
                self.get_logger().info(f"서비스 성공 (소요시간: {self.current_cycle_time:.2f}s)")
            else:
                self.get_logger().warn("서비스 로직 자체 실패")
                self.recipe_failed_flag = True
            
            self.save_experiment_history()
                
        except Exception as e:
            self.get_logger().error(f"서비스 에러: {e}")
            self.recipe_failed_flag = True
            self.save_experiment_history()

    def control_live_callback(self, msg):
        self.latest_system_status['tcp_vel'] = round(msg.tcp_vel, 2)
        self.latest_system_status['tcp_acc'] = round(msg.tcp_acc, 2)
        self.latest_system_status['pour_speed'] = round(msg.pour_speed, 2)

    def control_result_callback(self, msg):
        try:
            current_error_rate = round(msg.error_rate, 2)
            
            if current_error_rate > 5.0:
                self.recipe_failed_flag = True
                self.get_logger().warn(f"품질 기준 미달: 오차율 5% 초과 감지 ({current_error_rate}%). 레시피 실패 처리.")
            
            self.current_recipe_max_error_rate = max(self.current_recipe_max_error_rate, current_error_rate)

            metrics_data = {
                'timestamp': int(time.time() * 1000),
                'error_rate': current_error_rate,   
                'p_gain': round(msg.p_gain, 4),
                'd_gain': round(msg.d_gain, 4),
                'overshoot': round(msg.overshoot, 2),
                'ss_error': round(msg.ss_error, 2)
            }
            db.reference('control_metrics_history').push(metrics_data)

        except Exception as e:
            self.get_logger().error(f"Result 콜백 처리 실패: {e}")

    def upload_to_firebase(self):
        try:
            updates = {
                'sensor_data/weight': round(self.latest_weight, 2),
                'sensor_data/timestamp': int(time.time() * 1000)
            }
            if self.latest_system_status:
                updates['system_stats'] = self.latest_system_status
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
            db.reference().update(updates)
        except Exception as e:
            pass

    def system_status_callback(self, msg):
        self.latest_system_status["phase"] = msg.phase
        
    def save_experiment_history(self):
        try:
            target_w = self.current_target_weight
            final_w = round(self.final_cumulative_weight, 2) # [수정] latest_weight 대신 유효한 누적 무게 사용
            ss_error_g = round(abs(target_w - final_w), 2)
            
            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,
                'success': not self.recipe_failed_flag, 
                'error_rate': self.current_recipe_max_error_rate, 
                'ss_error_g': ss_error_g,
                'cycle_time': round(self.current_cycle_time, 2)
            }
            
            db_ref = db.reference('experiment_history')
            db_ref.push(history_data)
            
            self.get_logger().info(f"DB 저장 완료 최종판정: {'성공' if not self.recipe_failed_flag else '실패(품질미달 or 긴급정지)'}")
        except Exception as e:
            self.get_logger().error(f"DB 저장 실패: {e}")

    def weight_callback(self, msg): 
        current_phase = self.latest_system_status.get('phase', 'Ready').lower()
        if current_phase in ["mixing", "return"]:
            self.is_weight_blocked = True
            
        raw_weight = float(msg.data)

        if self.is_weight_blocked:
            self.latest_weight = 0.0
            self.last_raw_weight = 0.0 # [추가] 블록 해제 시 잘못된 오프셋 계산 방지
        else:
            # [추가] 무게가 3.0g 이상 급감하면 로봇이 Tare를 수행한 것으로 간주하여 오프셋 누적
            if self.last_raw_weight - raw_weight > 3.0:
                self.base_offset += self.last_raw_weight
                self.get_logger().info(f"영점 조절 감지. 누적 오프셋 갱신: {self.base_offset:.1f}g")

            self.last_raw_weight = raw_weight
            self.latest_weight = self.base_offset + raw_weight

            # [추가] DB 최종 저장을 위해 차단되기 전 유효한 누적 무게 지속 기록
            self.final_cumulative_weight = self.latest_weight

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