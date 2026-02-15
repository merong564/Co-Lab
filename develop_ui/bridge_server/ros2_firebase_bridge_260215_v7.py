import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import math
import time

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
        
        # Firebase 초기화 (기존과 동일)
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
        except Exception:
            pass

        # Publisher & Subscriber
        self.ui_pub = self.create_publisher(UiInput, '/ui/input', 10)
        self.stop_pub = self.create_publisher(String, '/ui/stop', 10)
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)

        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        self.latest_joints = None
        self.latest_weight = 0.0
        
        # [NEW] 마지막으로 데이터를 받은 시간 기록
        self.last_joint_time = 0.0

        self.get_logger().info('🚀 Bridge V7 Started (Zombie Check Added)')

    def loop_callback(self):
        self.check_firebase_commands()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        # (기존 코드와 동일: start_pouring, emergency_stop 처리)
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
        self.last_joint_time = time.time() # [NEW] 시간 갱신

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        try:
            updates = {}
            current_time = time.time()

            # [NEW] 로봇 데이터가 1초 이내에 갱신된 경우에만 업로드 (좀비 방지)
            if self.latest_joints and (current_time - self.last_joint_time < 1.0):
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
            else:
                # 데이터가 오래되면 Firebase에서 로봇 상태 삭제 (UI가 Offline 인식)
                # 또는 단순히 업데이트 안 함
                pass

            updates['sensor_data/weight'] = round(self.latest_weight, 2)
            
            if updates:
                db.reference().update(updates)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RosFirebaseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()