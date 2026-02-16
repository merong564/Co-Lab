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
            self.get_logger().info("🔥 Firebase Connected Successfully!")
            
            # [핵심 수정] 시작하자마자 기존 명령어 삭제 (초기화)
            db.reference('commands').set({}) 
            self.get_logger().info("🧹 [Init] Old commands cleared from Firebase.")
            
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        # 2. Publisher & Subscriber
        self.ui_pub = self.create_publisher(UiInput, '/ui/input', 10)
        self.stop_pub = self.create_publisher(String, '/ui/stop', 10)
        
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)
        self.create_subscription(String, '/pour_system/status', self.robot_status_callback, 10)

        self.timer = self.create_timer(0.1, self.loop_callback)
        
        # [수정] 현재 시간으로 초기화 (과거 데이터 무시용)
        self.last_command_timestamp = time.time() * 1000 
        
        self.latest_joints = None
        self.latest_weight = 0.0
        self.robot_extended_status = {} 
        self.last_joint_time = 0.0

        self.get_logger().info('🚀 Bridge V10 Started (Auto-Clear Added)')

    def loop_callback(self):
        self.check_firebase_commands()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        try:
            cmd_ref = db.reference('commands')
            cmd_data = cmd_ref.get()
            
            # 데이터가 있고, timestamp가 존재할 때만 실행
            if cmd_data and 'timestamp' in cmd_data:
                # [중요] 내가 켜진 이후(last_command_timestamp)에 들어온 명령만 처리
                if cmd_data['timestamp'] > self.last_command_timestamp:
                    self.last_command_timestamp = cmd_data['timestamp']
                    cmd_type = cmd_data.get('type', '')

                    if cmd_type == 'start_pouring':
                        msg = UiInput()
                        msg.is_confirmed = True
                        msg.target_weight = float(cmd_data.get('target_weight', 0.0))
                        self.ui_pub.publish(msg)
                        self.get_logger().info(f"▶ [Bridge] Start Command: {msg.target_weight}g")
                    
                    elif cmd_type == 'emergency_stop':
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 [Bridge] STOP Signal!")
                        
                    elif cmd_type == 'tare':
                         self.get_logger().info("⚖️ [Bridge] Tare Command")
        except Exception:
            pass

    def joint_callback(self, msg):
        self.latest_joints = [math.degrees(rad) for rad in msg.position]
        self.last_joint_time = time.time()

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def robot_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.robot_extended_status = data
        except Exception:
            pass

    def upload_to_firebase(self):
        try:
            updates = {}
            current_time = time.time()

            if self.latest_joints and (current_time - self.last_joint_time < 1.5):
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
                
                if self.robot_extended_status:
                    updates['robot_status/phase'] = self.robot_extended_status.get('phase', 'Ready')
                    updates['robot_status/velocity'] = self.robot_extended_status.get('vel', 0)
                    updates['robot_status/acceleration'] = self.robot_extended_status.get('acc', 0)
            
            updates['sensor_data/weight'] = round(self.latest_weight, 2)
            updates['sensor_data/timestamp'] = int(time.time() * 1000)
            
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