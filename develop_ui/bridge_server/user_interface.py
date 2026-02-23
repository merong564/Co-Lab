#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260220_v01 (Service Client Implemented + Experiment History Archiving)
[Description] Firebase 명령을 받아 /start_process 서비스를 호출하는 브릿지 및 결과 히스토리 DB 저장
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import datetime  # [추가] DB 히스토리에 날짜/시간을 기록하기 위한 모듈

# 메시지 및 서비스 타입 임포트
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
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
            # 2. Service Client 생성 (/start_process)
            self.cli = self.create_client(RobotCommand, 'start_process')
            
            # 3. 긴급 정지 Publisher (/stop)
            self.stop_pub = self.create_publisher(String, 'stop', 10)
            
            # 4. Subscribers (Robot -> UI)
            self.create_subscription(JointState, 'dsr01/joint_states', self.joint_callback, 10)
            self.create_subscription(Float32, 'load_cell/weight', self.weight_callback, 10)
            self.create_subscription(SystemStatus, 'system_status', self.system_status_callback, 10)
        
        # 5. Timer & Variables
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = time.time() * 1000 
        
        self.latest_weight = 0.0
        self.latest_system_status = {}
        
        # [추가] 실험 히스토리 저장을 위한 상태 변수
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

                    # [수정] start_pouring 명령이 오면 목표값과 재료를 기억해두고 서비스 호출
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
        """ /start_process 서비스 호출 함수 """
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패. 로봇 제어 노드가 켜져 있나요?')
            return

        req = RobotCommand.Request()
        req.mode = "FULL"
        req.target_weight = float(cmd_data.get('target_weight', 0.0))
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))

        self.get_logger().info(f"📤 서비스 요청 보냄: Target={req.target_weight}g, Mix={req.mixing_duration}s")
        
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
                
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
                updates['robot_status/velocity'] = self.latest_system_status.get('tcp_vel', 0)
                updates['robot_status/acceleration'] = self.latest_system_status.get('tcp_acc', 0)
            
            db.reference().update(updates)
        except Exception:
            pass

    def system_status_callback(self, msg):
        # [추가] 로봇 노드에서 작업 횟수(total_count)를 올렸다면 사이클이 끝났다는 의미!
        if hasattr(self, 'last_total_count') and msg.total_count > self.last_total_count:
            # 처음 노드가 켜졌을 때 이전 total_count를 불러오면서 바로 저장되는 것을 방지
            if self.last_total_count != 0: 
                self.save_experiment_history(msg)
            self.last_total_count = msg.total_count # 다음 체크를 위해 업데이트

        # [수정/추가] 기존 상태 데이터에 제어 지표(P, D, Overshoot 등) 추가
        self.latest_system_status = {
            "phase": msg.phase,
            "tcp_vel": msg.tcp_vel,
            "tcp_acc": msg.tcp_acc,
            "pour_speed": msg.pour_speed,
            "total_count": msg.total_count,
            "success_count": msg.success_count,
            "error_rate": round(msg.error_rate, 2),
            "last_cycle_time": round(msg.last_cycle_time, 2),
            # 👇 새로 추가된 제어 지표들 (메시지에 해당 필드가 있어야 함)
            "p_gain": getattr(msg, 'p_gain', 0.0),
            "d_gain": getattr(msg, 'd_gain', 0.0),
            "overshoot_g": round(getattr(msg, 'overshoot', 0.0), 2),
            "rise_time": round(getattr(msg, 'rise_time', 0.0), 2),
            "settling_time": round(getattr(msg, 'settling_time', 0.0), 2)
        }
        
    # [수정/추가] 1사이클 실험 종료 시 DB에 고급 분석 지표를 포함하여 저장
    def save_experiment_history(self, msg):
        try:
            # 변수 안전하게 추출
            p_gain = getattr(msg, 'p_gain', 0.0)
            d_gain = getattr(msg, 'd_gain', 0.0)
            overshoot = getattr(msg, 'overshoot', 0.0)
            rise_time = getattr(msg, 'rise_time', 0.0)
            settling_time = getattr(msg, 'settling_time', 0.0)
            
            # 지표 계산 로직
            target_w = self.current_target_weight
            final_w = round(self.latest_weight, 2)
            ss_error_g = round(abs(target_w - final_w), 2)
            error_rate = round(msg.error_rate, 2)
            
            # P/D 비율 및 붓기 속도 계산 (분모가 0인 경우 방지)
            pd_ratio = round(p_gain / d_gain, 2) if d_gain > 0 else 0.0
            avg_pouring_rate = round(final_w / rise_time, 2) if rise_time > 0 else 0.0
            
            # 사이클 타임 및 오버헤드 (안정화 시간 + 이동 대기 8초 가정)
            calc_cycle_time = round(settling_time + 8.0, 2)
            overhead_time = 8.0

            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': target_w,
                'final_weight': final_w,
                'error_rate': error_rate,
                # [중요] 성공 기준을 ±10%로 업데이트
                'success': True if error_rate <= 10.0 else False,
                
                # 👇 UI 연동을 위한 신규 분석 지표 추가 👇
                'ss_error_g': ss_error_g,
                'overshoot_g': round(overshoot, 2),
                'p_d_ratio': pd_ratio,
                'avg_pouring_rate': avg_pouring_rate,
                'cycle_time': calc_cycle_time,
                'overhead_time': overhead_time
            }
            
            # push()를 사용하여 experiment_history/ 경로에 데이터 누적
            db_ref = db.reference('experiment_history')
            new_record = db_ref.push(history_data)
            
            self.get_logger().info(f"💾 [DB 저장 성공] ID: {new_record.key} | 오차: {error_rate}%")
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

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