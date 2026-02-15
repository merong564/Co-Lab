import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import math
import time
import json

# 메시지 타입
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
try:
    from colab_interfaces.msg import UiInput
except ImportError:
    pass

class RosFirebaseBridge(Node):
    def __init__(self):
        super().__init__('ros_firebase_bridge')
        
        # 1. Firebase 초기화
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
        except Exception:
            pass

        # 2. Publisher & Subscriber
        self.ui_pub = self.create_publisher(UiInput, '/ui/input', 10)
        self.stop_pub = self.create_publisher(String, '/ui/stop', 10)
        
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)
        # [NEW] 로봇 상세 상태 수신 (JSON 형식)
        self.create_subscription(String, '/pour_system/status', self.robot_status_callback, 10)

        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        # 데이터 저장소
        self.latest_joints = None
        self.latest_weight = 0.0
        self.robot_extended_status = {} # 속도, 가속도, 단계 정보
        self.last_joint_time = 0.0      # 좀비 체크용 타이머

        self.get_logger().info('🚀 Bridge V7 Started (Zombie Check & Extended Info)')

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
                        msg = UiInput()
                        msg.is_confirmed = True
                        msg.target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.ui_pub.publish(msg)
                        self.get_logger().info(f"▶ START: {msg.target_weight}g")
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 STOP Signal Sent")
                        
                    elif cmd_type == 'tare':
                         self.get_logger().info("⚖️ Tare Command Received")
        except Exception:
            pass

    def joint_callback(self, msg):
        self.latest_joints = [math.degrees(rad) for rad in msg.position]
        self.last_joint_time = time.time() # 데이터 수신 시간 갱신

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def robot_status_callback(self, msg):
        """로봇에서 보낸 JSON 상태 정보 수신"""
        try:
            # 예: {"phase": "Pouring", "vel": 60, "acc": 60}
            self.robot_extended_status = json.loads(msg.data)
        except Exception:
            pass

    def upload_to_firebase(self):
        try:
            updates = {}
            current_time = time.time()

            # [핵심] 1.5초 이상 로봇 데이터가 없으면 'System Online' 갱신 안 함 -> UI가 Offline 처리
            if self.latest_joints and (current_time - self.last_joint_time < 1.5):
                # 관절 각도 (J1 ~ J6)
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2) # Tilt (J6)
                
                # [NEW] 확장 정보 (속도, 가속도, 단계)
                if self.robot_extended_status:
                    updates['robot_status/phase'] = self.robot_extended_status.get('phase', 'Ready')
                    updates['robot_status/velocity'] = self.robot_extended_status.get('vel', 0)
                    updates['robot_status/acceleration'] = self.robot_extended_status.get('acc', 0)
            
            # 무게 데이터는 센서가 살아있으면 계속 보냄
            updates['sensor_data/weight'] = round(self.latest_weight, 2)
            
            if updates:
                db.reference().update(updates)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RosFirebaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()