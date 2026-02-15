import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# ROS2 메시지 타입
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math

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

        # 2. ROS2 Subscriber
        
        # (1) 로봇 관절 상태 (/dsr01/joint_states)
        self.create_subscription(
            JointState,
            '/dsr01/joint_states', 
            self.joint_callback,
            10
        )

        # (2) 로드셀 무게 (/loadcell_weight) - 요청하신 이름으로 변경
        self.create_subscription(
            Float32,
            '/loadcell_weight', 
            self.weight_callback,
            10
        )

        # 3. 데이터 전송 타이머 (0.1초마다 실행)
        self.latest_joints = None
        self.latest_weight = 0.0
        self.timer = self.create_timer(0.1, self.upload_to_firebase)

        self.get_logger().info('🚀 Bridge Node Started. Waiting for /dsr01/joint_states & /loadcell_weight...')

    def joint_callback(self, msg):
        # 데이터 수신 확인용 로그 (너무 자주 뜨면 주석 처리하세요)
        # self.get_logger().info(f'Joint Received: {len(msg.position)} axes')
        degrees = [math.degrees(rad) for rad in msg.position]
        self.latest_joints = degrees

    def weight_callback(self, msg):
        # 데이터 수신 확인용 로그
        # self.get_logger().info(f'Weight Received: {msg.data}')
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        try:
            updates = {}

            # 1. 관절 데이터 업데이트
            if self.latest_joints and len(self.latest_joints) >= 6:
                updates['robot_status/joint'] = {
                    'j1': round(self.latest_joints[0], 2),
                    'j2': round(self.latest_joints[1], 2),
                    'j3': round(self.latest_joints[2], 2),
                    'j4': round(self.latest_joints[3], 2),
                    'j5': round(self.latest_joints[4], 2),
                    'j6': round(self.latest_joints[5], 2)
                }
                # 붓기 각도 (J6 사용)
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)

            # 2. 무게 데이터 업데이트
            updates['sensor_data/weight'] = round(self.latest_weight, 2)

            # Firebase 전송
            if updates:
                db.reference().update(updates)
                # print("Update sent to Firebase") # 디버깅 필요시 주석 해제

        except Exception as e:
            self.get_logger().warn(f'Upload Error: {e}')

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