#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260226_v17 (True Cycle Time & Final Weight Keeper)
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
            
            self.create_subscription(String, 'stop/ui', self.stop_status_callback, 10)
            self.create_subscription(String, 'stop', self.stop_status_callback, 10)
        
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        self.latest_pour_speed = 0.0
        
        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        
        # 💡 [핵심 추가] 진짜 사이클 타임과 최종 무게를 계산하기 위한 상태 변수들
        self.cycle_start_time = 0.0
        self.current_cycle_time = 0.0
        self.recipe_failed_flag = False
        self.is_cycle_running = False         # 현재 작업이 진행 중인지 여부
        self.final_accumulated_weight = 0.0   # 믹싱 직전 비커에 담긴 최종 누적 무게 저장용

    def stop_status_callback(self, msg):
        text = msg.data.strip().upper()
        if text in ["STOP", "RECOVERY", "EMERGENCY"]:
            self.recipe_failed_flag = True
            self.latest_system_status['phase'] = 'Emergency' 
            try:
                db.reference('system_stats/phase').set('Emergency')
            except Exception:
                pass
            self.get_logger().warn(f"🚨 [비상/복구 감지] 시스템 중단. UI 게이지 강제 초기화! 원인: {text}")

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

                        # 💡 [수정] 작업 시작 버튼을 누르는 순간 타이머 ON 및 변수 초기화
                        self.cycle_start_time = time.time()
                        self.current_cycle_time = 0.0
                        self.recipe_failed_flag = False
                        self.latest_pour_speed = 0.0
                        self.final_accumulated_weight = 0.0
                        self.is_cycle_running = True 

                        self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.current_material = cmd_data.get('material', 'Unknown')
                        self.call_service_start_process(cmd_data)
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 EMERGENCY STOP Signal Sent from UI!")
                        self.recipe_failed_flag = True
                        self.latest_system_status['phase'] = 'Emergency'
                        db.reference('system_stats/phase').set('Emergency')
                    
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
                self.get_logger().info(f"✅ 컨트롤러 서비스 완료 응답 수신")
            else:
                self.get_logger().warn(f"❌ 서비스 실패: {response.message}")
                self.recipe_failed_flag = True
        except Exception as e:
            self.get_logger().error(f"서비스 호출 중 에러 발생: {e}")
            self.recipe_failed_flag = True

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
        new_phase = getattr(msg, 'phase', 'Ready')
        old_phase = self.latest_system_status.get('phase', 'Ready')

        # 💡 [핵심 로직 1] 믹싱 단계 진입 시 (비커를 들어올리기 직전)
        # 로드셀 무게가 0이 되기 전에 지금까지 누적된 진짜 최종 무게를 킵해둡니다!
        if new_phase == 'Mixing' and old_phase != 'Mixing':
            self.final_accumulated_weight = self.latest_weight
            self.get_logger().info(f"📌 [무게 확정] 믹싱 진입 전 최종 누적 무게 저장: {self.final_accumulated_weight}g")

        # 💡 [핵심 로직 2] 정상 종료 시점 (Return이 끝나고 다시 Ready가 될 때)
        if self.is_cycle_running and new_phase == 'Ready' and old_phase == 'Return':
            self.current_cycle_time = time.time() - self.cycle_start_time
            self.save_experiment_history()
            self.is_cycle_running = False # 사이클 종료
            self.get_logger().info(f"⏱️ [사이클 종료] 진짜 소요 시간: {self.current_cycle_time:.2f}초")

        # 💡 [핵심 로직 3] 비상 종료 시점 (Emergency 발생 시)
        if self.is_cycle_running and new_phase == 'Emergency' and old_phase != 'Emergency':
            self.current_cycle_time = time.time() - self.cycle_start_time
            self.recipe_failed_flag = True
            self.save_experiment_history()
            self.is_cycle_running = False # 사이클 강제 종료

        self.latest_system_status = {
            "phase": new_phase,
            "tcp_vel": getattr(msg, 'tcp_vel', 0.0),       
            "tcp_acc": getattr(msg, 'tcp_acc', 0.0),       
            "pour_speed": getattr(self, 'latest_pour_speed', 0.0), 
            "total_count": getattr(msg, 'total_count', 0),
            "success_count": getattr(msg, 'success_count', 0),
            "error_rate": round(getattr(msg, 'error_rate', 0.0), 2),
            "last_cycle_time": round(getattr(msg, 'last_cycle_time', 0.0), 2)
        }
        
    def save_experiment_history(self):
        try:
            target_w = self.current_target_weight
            
            # 💡 [수정] 비상정지 시에는 그 순간의 무게를, 정상이면 믹싱 직전에 킵해둔 무게를 사용합니다.
            final_w = round(self.latest_weight if self.recipe_failed_flag else self.final_accumulated_weight, 2)
            ss_error_g = round(abs(target_w - final_w), 2)
            
            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,  # 0g이 아닌 진짜 누적 무게가 들어감!
                'success': not self.recipe_failed_flag,
                'ss_error_g': ss_error_g,
                'cycle_time': round(self.current_cycle_time, 2) # 진짜 사이클 타임!
            }
            
            now = datetime.datetime.now()
            time_str = now.strftime('%Y%m%d_%H%M%S') 
            safe_material = self.current_material.replace('/', '-').replace(' ', '_').replace('(', '').replace(')', '')
            custom_key = f"{time_str}_{safe_material}" 
            
            db_ref = db.reference('experiment_history')
            db_ref.child(custom_key).set(history_data)
            
            self.get_logger().info(f"💾 [DB 저장 성공] ID: {custom_key} | 결과무게: {final_w}g | 소요시간: {self.current_cycle_time:.2f}s | 판정: {'성공' if not self.recipe_failed_flag else '실패'}")
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

    def control_metrics_callback(self, msg):
        try:
            current_pour_speed = round(getattr(msg, 'pour_speed', 0.0), 2)
            self.latest_pour_speed = current_pour_speed 
            
            current_error_rate = round(getattr(msg, 'error_rate', 0.0), 2)

            if current_error_rate > 10.0:
                self.recipe_failed_flag = True
                self.get_logger().warn(f"🚨 오차 허용치 초과 감지 ({current_error_rate}%). 레시피 실패 처리 예약.")

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
                'ss_error': round(getattr(msg, 'ss_error', 0.0), 2)
            }
            
            now = datetime.datetime.now()
            time_str = now.strftime('%Y%m%d_%H%M%S')
            ms = int(time.time() * 100) % 100
            custom_key = f"{time_str}_{ms}_Metrics"

            db_ref = db.reference('control_metrics_history')
            db_ref.child(custom_key).set(metrics_data) 
            
            self.latest_system_status['max_tilt_step'] = round(getattr(msg, 'max_tilt_step', 0.0), 2)
            self.latest_system_status['stop_threshold'] = round(getattr(msg, 'stop_threshold', 0.0), 2)

        except Exception as e:
            self.get_logger().error(f"❌ DB 제어 지표 저장 실패: {e}")

    def joint_callback(self, msg): 
        self.latest_joints = [math.degrees(rad) for rad in msg.position]
        self.last_joint_time = time.time()

    def weight_callback(self, msg): 
        self.latest_weight = float(msg.data)

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