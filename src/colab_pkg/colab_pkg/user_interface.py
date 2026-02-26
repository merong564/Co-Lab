#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260226_v26 (Emergency Tag in History)
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
    from colab_interfaces.msg import ControlResult  
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
            self.create_subscription(ControlResult, 'log_control_metrics', self.control_result_callback, 10)
            
            self.create_subscription(String, 'stop/ui', self.stop_status_callback, 10)
            self.create_subscription(String, 'stop', self.stop_status_callback, 10)
            self.create_subscription(String, 'stop/impact', self.stop_status_callback, 10)
        
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        self.latest_pour_speed = 0.0
        
        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        
        self.last_total_count = 0 
        self.is_first_msg = True
        
        self.cycle_start_time = 0.0
        self.current_cycle_time = 0.0
        self.recipe_failed_flag = False
        self.is_cycle_running = False         
        
        self.base_accumulated_weight = 0.0 
        self.final_accumulated_weight = 0.0   
        
        # 💡 [추가] 긴급 중단 여부를 추적하는 변수
        self.is_emergency_stopped = False

    def stop_status_callback(self, msg):
        text = msg.data.strip().upper()
        if text in ["STOP", "RECOVERY", "EMERGENCY"]:
            self.recipe_failed_flag = True
            self.is_emergency_stopped = True  # 💡 [추가] 토픽 수신 시 긴급중단 플래그 ON
            self.latest_system_status['phase'] = 'Emergency' 
            try:
                db.reference('system_stats/phase').set('Emergency')
            except Exception:
                pass
            self.get_logger().warn(f"🚨 [비상/복구 감지] 시스템 중단. 원인: {text}")
            
            if self.is_cycle_running:
                self.current_cycle_time = time.time() - self.cycle_start_time
                self.save_experiment_history()
                self.is_cycle_running = False

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

                        self.cycle_start_time = time.time()
                        self.current_cycle_time = 0.0
                        self.recipe_failed_flag = False
                        self.is_emergency_stopped = False # 💡 [추가] 작업 시작 시 긴급중단 플래그 리셋
                        self.latest_pour_speed = 0.0
                        
                        self.base_accumulated_weight = 0.0
                        self.final_accumulated_weight = 0.0
                        self.is_cycle_running = True 

                        self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.current_material = cmd_data.get('material', 'Unknown')
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.recipe_failed_flag = True
                        self.is_emergency_stopped = True # 💡 [추가] UI 버튼 클릭 시 긴급중단 플래그 ON
                        self.latest_system_status['phase'] = 'Emergency'
                        db.reference('system_stats/phase').set('Emergency')
                        
                        if self.is_cycle_running:
                            self.current_cycle_time = time.time() - self.cycle_start_time
                            self.save_experiment_history()
                            self.is_cycle_running = False

        except Exception as e:
            self.get_logger().error(f"Command Check Error: {e}")

    def call_service_start_process(self, cmd_data):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패. 로봇 제어 노드가 켜져 있나요?')
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
            if not response.success:
                self.get_logger().warn(f"❌ 서비스 실패: {response.message}")
                self.recipe_failed_flag = True
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 에러 발생: {e}")
            self.recipe_failed_flag = True

    def upload_to_firebase(self):
        try:
            current_phase = self.latest_system_status.get('phase', 'Ready').lower()
            
            if current_phase in ['mixing', 'return']:
                display_w = 0.0 
            else:
                display_w = round(self.base_accumulated_weight + self.latest_weight, 2)

            updates = {
                'sensor_data/weight': display_w,
                'sensor_data/timestamp': int(time.time() * 1000)
            }
            if self.latest_system_status:
                updates['system_stats'] = self.latest_system_status
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
                updates['robot_status/velocity'] = self.latest_system_status.get('tcp_vel', 0)
                updates['robot_status/acceleration'] = self.latest_system_status.get('tcp_acc', 0)
            db.reference().update(updates)
        except Exception as e:
            pass

    def system_status_callback(self, msg):
        new_phase = getattr(msg, 'phase', 'Ready')
        old_phase = self.latest_system_status.get('phase', 'Ready')
        new_count = getattr(msg, 'total_count', 0)

        if new_phase == 'Mixing' and old_phase != 'Mixing':
            self.final_accumulated_weight = self.base_accumulated_weight + self.latest_weight
            self.get_logger().info(f"📌 [무게 확정] 믹싱 진입 전 최종 누적 무게 저장: {self.final_accumulated_weight:.2f}g")

        if getattr(self, 'is_first_msg', True):
            self.last_total_count = new_count
            self.is_first_msg = False
        elif new_count > self.last_total_count:
            if self.is_cycle_running:
                self.current_cycle_time = time.time() - self.cycle_start_time
                self.save_experiment_history()
                self.is_cycle_running = False 
            self.last_total_count = new_count

        self.latest_system_status = {
            "phase": new_phase,
            "tcp_vel": getattr(msg, 'tcp_vel', 0.0),       
            "tcp_acc": getattr(msg, 'tcp_acc', 0.0),       
            "pour_speed": getattr(self, 'latest_pour_speed', 0.0), 
            "total_count": new_count,
            "success_count": getattr(msg, 'success_count', 0),
            "error_rate": round(getattr(msg, 'error_rate', 0.0), 2),
            "last_cycle_time": round(getattr(msg, 'last_cycle_time', 0.0), 2)
        }
        
    def save_experiment_history(self):
        try:
            target_w = self.current_target_weight
            
            if self.recipe_failed_flag:
                final_w = round(self.base_accumulated_weight + self.latest_weight, 2)
            else:
                final_w = round(self.final_accumulated_weight, 2)
                
            ss_error_g = round(abs(target_w - final_w), 2)
            
            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,  
                'success': not self.recipe_failed_flag,
                'is_emergency': self.is_emergency_stopped, # 💡 [추가] DB에 긴급중단 여부 기록
                'ss_error_g': ss_error_g,
                'cycle_time': round(self.current_cycle_time, 2)
            }
            
            now = datetime.datetime.now()
            time_str = now.strftime('%Y%m%d_%H%M%S') 
            safe_material = str(self.current_material).replace('/', '-').replace(' ', '_').replace('(', '').replace(')', '')
            if not safe_material: safe_material = "Unknown"
            custom_key = f"{time_str}_{safe_material}" 
            
            db_ref = db.reference('experiment_history')
            db_ref.child(custom_key).set(history_data)
            
            self.get_logger().info(f"💾 [DB 저장 성공] ID: {custom_key} | 결과무게: {final_w}g | 소요시간: {self.current_cycle_time:.2f}s | 판정: {'성공' if not self.recipe_failed_flag else '실패'}")
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

    def control_result_callback(self, msg):
        try:
            current_pour_speed = round(getattr(msg, 'pour_speed', 0.0), 2)
            self.latest_pour_speed = current_pour_speed 
            current_error_rate = round(getattr(msg, 'error_rate', 0.0), 2)

            if current_error_rate > 10.0:
                self.recipe_failed_flag = True

            if hasattr(msg, 'final_settled_weight'):
                added_weight = float(msg.final_settled_weight)
                self.base_accumulated_weight += added_weight
                self.get_logger().info(f"➕ [하드웨어 영점 보정] 붓기 완료. 추가 무게: {added_weight:.1f}g, 총 누적 베이스: {self.base_accumulated_weight:.1f}g")

            metrics_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'pour_speed': current_pour_speed,  
                'error_rate': current_error_rate,   
                'p_gain': round(getattr(msg, 'p_gain', 0.0), 4),
                'd_gain': round(getattr(msg, 'd_gain', 0.0), 4),
                'max_tilt_step': round(getattr(msg, 'max_tilt_step', 0.0), 2),
                'stop_threshold': round(getattr(msg, 'stop_threshold', 0.0), 2),
                'p_d_ratio': round(getattr(msg, 'p_d_ratio', 0.0), 2),
                'overshoot': round(getattr(msg, 'overshoot', 0.0), 2),
                'rise_time': round(getattr(msg, 'rise_time', 0.0), 2),
                'settling_time': round(getattr(msg, 'settling_time', 0.0), 2),
                'ss_error': round(getattr(msg, 'ss_error', 0.0), 2),
                'final_settled_weight': round(getattr(msg, 'final_settled_weight', 0.0), 2)
            }
            
            now = datetime.datetime.now()
            time_str = now.strftime('%Y%m%d_%H%M%S')
            ms = int(time.time() * 100) % 100
            
            safe_material = str(self.current_material).replace('/', '-').replace(' ', '_').replace('(', '').replace(')', '')
            if not safe_material: 
                safe_material = "Unknown"
                
            custom_key = f"{time_str}_{ms}_Metrics_{safe_material}"

            db.reference('control_metrics_history').child(custom_key).set(metrics_data) 
            self.latest_system_status['max_tilt_step'] = round(getattr(msg, 'max_tilt_step', 0.0), 2)
            self.latest_system_status['stop_threshold'] = round(getattr(msg, 'stop_threshold', 0.0), 2)

        except Exception as e:
            self.get_logger().error(f"❌ Metrics 콜백 에러: {e}")

    def joint_callback(self, msg): pass
    
    def weight_callback(self, msg): 
        self.latest_weight = float(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()