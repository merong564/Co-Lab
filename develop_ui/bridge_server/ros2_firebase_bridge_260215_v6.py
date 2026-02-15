import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# === [변경] 커스텀 메시지 임포트 ===
# (주의: colab_interfaces 패키지가 빌드되어 있어야 합니다)
try:
    from colab_interfaces.msg import UiInput
except ImportError:
    print("⚠️ [Error] colab_interfaces 패키지를 찾을 수 없습니다. 'source install/setup.bash'를 했는지 확인하세요!")
    # 에러 방지를 위해 임시 클래스 정의 (실제 실행시엔 빌드된 패키지 사용 필수)
    class UiInput:
        pass

# 기본 메시지 타입
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
        # [변경] 통합된 UI 입력 메시지 발행 (/ui/input)
        self.ui_pub = self.create_publisher(UiInput, '/ui/input', 10)
        
        # [추가] 긴급 정지 전용 채널 (/ui/stop)
        self.stop_pub = self.create_publisher(String, '/ui/stop', 10)

        # 3. ROS2 Subscriber (로봇 -> Firebase 전달)
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)

        # 4. 루프 타이머 (0.1초마다 명령 체크 및 데이터 업로드)
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        self.latest_joints = None
        self.latest_weight = 0.0

        self.get_logger().info('🚀 Bridge V6 Started (Topic: /ui/input using UiInput.msg)')

    def loop_callback(self):
        self.check_firebase_commands()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        """Firebase의 commands 노드를 감시하여 ROS2(UiInput)로 발행"""
        try:
            cmd_ref = db.reference('commands')
            cmd_data = cmd_ref.get()

            if cmd_data and 'timestamp' in cmd_data:
                # 새로운 명령인지 확인 (타임스탬프 비교)
                if cmd_data['timestamp'] > self.last_command_timestamp:
                    self.last_command_timestamp = cmd_data['timestamp']
                    cmd_type = cmd_data.get('type', '')

                    # === [CASE 1] 실험 시작 ===
                    if cmd_type == 'start_pouring':
                        # UiInput 메시지 생성
                        msg = UiInput()
                        msg.is_confirmed = True
                        # float64 변환
                        msg.target_weight = float(cmd_data.get('target_weight', 0.0))
                        
                        # 발행
                        self.ui_pub.publish(msg)
                        self.get_logger().info(f"▶ [Bridge] UiInput 전송: is_confirmed=True, 목표={msg.target_weight}g")

                    # === [CASE 2] 긴급 정지 ===
                    elif cmd_type == 'emergency_stop':
                        # 긴급 정지는 String으로 명확하게 쏨
                        msg = String()
                        msg.data = "STOP"
                        self.stop_pub.publish(msg)
                        self.get_logger().warn("🚨 [Bridge] 긴급 정지 명령 전송! (/ui/stop)")

                    # === [CASE 3] 영점 조절 (필요시 구현) ===
                    elif cmd_type == 'tare':
                        # 로봇 쪽에서 영점 조절 기능이 있다면 토픽 추가 가능
                        self.get_logger().info("⚖️ [Bridge] 영점 조절 명령 수신 (로봇에 전달 기능 미구현)")

        except Exception as e:
            self.get_logger().error(f"Command Error: {e}")

    def joint_callback(self, msg):
        # 라디안 -> 도(Degree) 변환
        self.latest_joints = [math.degrees(rad) for rad in msg.position]

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        """로봇 상태(관절, 무게)를 Firebase로 업로드"""
        try:
            updates = {}
            # 로봇 관절 정보
            if self.latest_joints and len(self.latest_joints) >= 6:
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
                
            # 무게 센서 정보
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
        node.get_logger().info('Bridge Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()