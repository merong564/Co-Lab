#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260220_v06 (MultiThreaded + 15% Tolerance 100/0 Success Rate + History Archiving)
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import os
from dotenv import load_dotenv
import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

ROBOT_ID = "dsr01"

# .env 파일 로드
env_path = os.path.expanduser('~/Co-Lab/.env')
load_dotenv(dotenv_path=env_path)

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    IMPORT_SUCCESS = True
except ImportError:
    print("❌ [Error] colab_interfaces 패키지를 찾을 수 없습니다.")
    IMPORT_SUCCESS = False

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface', namespace=ROBOT_ID)
        
        try:
            cred_path = os.getenv('FIREBASE_CRED_PATH')
            db_url = os.getenv('FIREBASE_DB_URL')

            if not cred_path or not db_url:
                raise ValueError(".env 파일에서 설정을 찾을 수 없습니다.")

            if not firebase_admin._apps:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred, {
                    'databaseURL': db_url
                })
            self.get_logger().info("🔥 Firebase Connected!")
            db.reference('commands').set({}) 
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        if IMPORT_SUCCESS:
            self.cli = self.create_client(RobotCommand, 'start_process')
            self.stop_pub = self.create_publisher(String, 'stop', 10)

            self.cb_group_sensor = ReentrantCallbackGroup()
            
            self.create_subscription(JointState, 'joint_states', self.joint_callback, 10, callback_group=self.cb_group_sensor)
            self.create_subscription(Float32, 'load_cell/weight', self.weight_callback, 10, callback_group=self.cb_group_sensor)
            self.create_subscription(SystemStatus, 'system_status', self.system_status_callback, 10, callback_group=self.cb_group_sensor)
        
        self.cb_group_timer = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.1, self.loop_callback, callback_group=self.cb_group_timer)
        self.last_command_timestamp = time.time() * 1000 
        
        self.latest_weight = 0.0
        self.latest_system_status = {}

        self.current_target_weight = 0.0
        self.current_material = "Unknown"
        self.experiment_start_time = 0.0

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
            self.get_logger().warn('⚠️ 서비스(/start_process) 연결 실패.')
            return

        req = RobotCommand.Request()
        req.mode = "FULL" 
        req.target_weight = float(cmd_data.get('target_weight', 0.0))
        req.mixing_duration = float(cmd_data.get('mixing_duration', 0.0))

        self.get_logger().info(f"📤 서비스 요청 보냄: Target={req.target_weight}g, Mix={req.mixing_duration}s")
        
        self.experiment_start_time = time.time()
        
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ 서비스 성공: {response.message}")
                self.save_experiment_history()
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

            if hasattr(self, 'latest_weight_rcv_time'):
                delay = time.time() - self.latest_weight_rcv_time

        except Exception as e:
            self.get_logger().error(f"Firebase Upload Error: {e}")

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

    def weight_callback(self, msg): 
        self.latest_weight = msg.data
        self.latest_weight_rcv_time = time.time()

    def save_experiment_history(self):
        try:
            error_rate = self.latest_system_status.get('error_rate', 0.0)
            if error_rate == 0.0 and self.current_target_weight > 0:
                error_g = abs(self.latest_weight - self.current_target_weight)
                error_rate = round((error_g / self.current_target_weight) * 100, 2)

            # [수정] 허용 오차율 15% 적용 및 성공률 100/0% 강제 변환
            TOLERANCE = 15.0 
            is_success = True if error_rate <= TOLERANCE else False
            success_rate = 100 if is_success else 0

            if self.experiment_start_time > 0:
                calculated_cycle_time = round(time.time() - self.experiment_start_time, 2)
            else:
                calculated_cycle_time = self.latest_system_status.get('last_cycle_time', 0.0)

            history_data = {
                'timestamp': int(time.time() * 1000),
                'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'material': self.current_material,
                'target_weight': self.current_target_weight,
                'final_weight': round(self.latest_weight, 2),
                'error_rate': error_rate,
                'success_rate': success_rate, # 100 아니면 0
                'cycle_time': calculated_cycle_time,
                'success': is_success # TOLERANCE 15% 기준 적용
            }
            db_ref = db.reference('experiment_history')
            new_record = db_ref.push(history_data)
            
            self.get_logger().info(f"💾 [DB 기록 완료] 오차율: {error_rate}%, 판정: {'성공' if is_success else '실패'}")
            self.experiment_start_time = 0.0
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 히스토리 저장 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()

    executor = MultiThreadedExecutor() 
    executor.add_node(node) 
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()