import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# ROS2 메시지 타입 (상황에 맞게 수정 가능)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import math
import time

class RosFirebaseBridge(Node):
    def __init__(self):
        super().__init__('ros_firebase_bridge')
        
        # ---------------------------------------------------------
        # 1. Firebase 초기화 (반드시 serviceAccountKey.json 필요)
        # ---------------------------------------------------------
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
            })
            self.get_logger().info('🔥 Firebase Connected Successfully!')
        except Exception as e:
            self.get_logger().error(f'Firebase Connection Failed: {e}')
            return

        # ---------------------------------------------------------
        # 2. ROS2 Subscriber (로봇 -> Firebase)
        # ---------------------------------------------------------
        
        # (1) 로봇 관절 상태 구독 (Isaac Sim 또는 실제 로봇)
        self.create_subscription(
            JointState,
            '/joint_states',  # Isaac Sim이나 실제 로봇이 발행하는 토픽명
            self.joint_callback,
            10
        )

        # (2) 무게 센서 구독 (PC1의 아두이노 노드가 발행한다고 가정)
        self.create_subscription(
            Float32,
            '/weight', 
            self.weight_callback,
            10
        )

        # ---------------------------------------------------------
        # 3. 데이터 전송 타이머 (Firebase 과부하 방지)
        # ---------------------------------------------------------
        # ROS는 1초에 100번 이상 데이터를 보내지만, 웹은 그럴 필요가 없음.
        # 0.1초(10Hz)마다 한 번씩만 업로드하도록 설정
        self.latest_joints = None
        self.latest_weight = 0.0
        self.timer = self.create_timer(0.1, self.upload_to_firebase)

        self.get_logger().info('🚀 Bridge Node Started. Waiting for ROS2 topics...')

    def joint_callback(self, msg):
        """ 로봇의 관절 데이터를 받아서 저장 (라디안 -> 도 변환) """
        # 보통 ROS는 라디안(rad), 두산 로봇 UI는 도(deg)를 씁니다.
        degrees = [math.degrees(rad) for rad in msg.position]
        self.latest_joints = degrees

    def weight_callback(self, msg):
        """ 로드셀 무게 데이터를 받아서 저장 """
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        """ 저장된 최신 데이터를 Firebase에 업로드 """
        try:
            updates = {}

            # 1. 관절 데이터 업데이트 (MoveJ 화면용)
            if self.latest_joints and len(self.latest_joints) >= 6:
                updates['robot_status/joint'] = {
                    'j1': round(self.latest_joints[0], 2),
                    'j2': round(self.latest_joints[1], 2),
                    'j3': round(self.latest_joints[2], 2),
                    'j4': round(self.latest_joints[3], 2),
                    'j5': round(self.latest_joints[4], 2),
                    'j6': round(self.latest_joints[5], 2)
                }
                
                # [중요] 붓기 시스템용 (Pouring System) 데이터
                # 붓기 각도(Tilt)는 보통 6번 관절(손목) 또는 5번 관절을 사용함.
                # 여기서는 6번 관절을 'current_angle'로 매핑합니다.
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
                updates['robot_status/velocity'] = 0  # 실제 속도 값을 안다면 여기에 연결
                updates['robot_status/acceleration'] = 0.0 

            # 2. 무게 데이터 업데이트 (그래프용)
            updates['sensor_data/weight'] = round(self.latest_weight, 2)

            # Firebase에 한 번에 전송 (네트워크 효율성)
            if updates:
                db.reference().update(updates)

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
