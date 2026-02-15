import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# ROS2 메시지 타입
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import math
import time

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
            self.get_logger().info('🔥 Firebase Connected Successfully!')
        except Exception as e:
            self.get_logger().error(f'Firebase Connection Failed: {e}')
            return

        # 2. ROS2 Publisher (Firebase -> 로봇 전달)
        self.cmd_pub = self.create_publisher(String, '/ui_command', 10)
        self.target_weight_pub = self.create_publisher(Float32, '/target_weight', 10)

        # 3. ROS2 Subscriber (로봇 -> Firebase 전달)
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)

        # 4. 루프 타이머 (0.1초마다 명령 체크 및 데이터 업로드)
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        self.latest_joints = None
        self.latest_weight = 0.0

    def loop_callback(self):
        self.check_firebase_commands()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        """Firebase의 commands 노드를 감시하여 ROS2로 발행"""
        try:
            cmd_ref = db.reference('commands')
            cmd_data = cmd_ref.get()

            if cmd_data and 'timestamp' in cmd_data:
                if cmd_data['timestamp'] > self.last_command_timestamp:
                    self.last_command_timestamp = cmd_data['timestamp']
                    cmd_type = cmd_data.get('type', '')

                    if cmd_type == 'start_pouring':
                        # 목표 무게 발행
                        target = float(cmd_data.get('target_weight', 0))
                        t_msg = Float32()
                        t_msg.data = target
                        self.target_weight_pub.publish(t_msg)
                        
                        # 시작 신호 발행
                        msg = String()
                        msg.data = "START"
                        self.cmd_pub.publish(msg)
                        self.get_logger().info(f"▶ [Bridge] 시작 명령 전송 (목표: {target}g)")

                    elif cmd_type == 'emergency_stop':
                        msg = String()
                        msg.data = "STOP"
                        self.cmd_pub.publish(msg)
                        self.get_logger().warn("🚨 [Bridge] 긴급 정지 명령 전송!")
        except Exception:
            pass

    def joint_callback(self, msg):
        self.latest_joints = [math.degrees(rad) for rad in msg.position]

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        """로봇 정보를 Firebase로 업로드"""
        try:
            updates = {}
            if self.latest_joints and len(self.latest_joints) >= 6:
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
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