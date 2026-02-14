import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db

# ROS2 메시지 타입
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String # 에러 메시지용 String 추가
import math
import time # 가속도 계산용 시간 측정

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
        
        # (1) 로봇 관절 상태 (위치 + 속도)
        self.create_subscription(
            JointState,
            '/dsr01/joint_states', 
            self.joint_callback,
            10
        )

        # (2) 로드셀 무게
        self.create_subscription(
            Float32,
            '/loadcell_weight', 
            self.weight_callback,
            10
        )

        # (3) [추가] 로봇 에러 모니터링
        # 보통 에러는 String이나 Int로 오지만, 안전하게 로그용으로 구독
        # (만약 에러가 뜬다면 로그에 띄우기 위함)
        # self.create_subscription(String, '/dsr01/error', self.error_callback, 10)

        # 3. 데이터 전송 타이머 (0.1초)
        self.latest_joints = None
        self.latest_velocities = None # 속도 저장용
        self.latest_weight = 0.0
        
        # 가속도 계산을 위한 변수들
        self.prev_velocity_sum = 0.0
        self.prev_time = time.time()
        self.current_accel = 0.0

        self.timer = self.create_timer(0.1, self.upload_to_firebase)

        self.get_logger().info('🚀 Bridge Node Started. Monitoring Speed & Accel...')

    def joint_callback(self, msg):
        # 1. 위치(각도) 변환
        degrees = [math.degrees(rad) for rad in msg.position]
        self.latest_joints = degrees

        # 2. 속도 정보 추출 (UI의 Speed 게이지용)
        # msg.velocity가 있으면 사용, 없으면 0
        if msg.velocity:
            self.latest_velocities = [abs(v) for v in msg.velocity] # 절대값 사용 (방향 무관)
            
            # 3. 가속도 계산 (간이 계산: 속도 평균의 변화량 / 시간)
            current_time = time.time()
            dt = current_time - self.prev_time
            
            if dt > 0:
                # 6개 관절 속도의 평균을 구해서 전체적인 로봇의 "움직임 강도"를 측정
                current_vel_sum = sum([abs(v) for v in msg.velocity]) / len(msg.velocity)
                
                # 가속도 = (현재속도 - 이전속도) / 시간
                accel = (current_vel_sum - self.prev_velocity_sum) / dt
                self.current_accel = abs(accel) # 절대값 (감속도 가속 취급)
                
                # 다음 계산을 위해 저장
                self.prev_velocity_sum = current_vel_sum
                self.prev_time = current_time

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    def upload_to_firebase(self):
        try:
            updates = {}

            # 1. 관절 데이터 & 상태 업데이트
            if self.latest_joints and len(self.latest_joints) >= 6:
                updates['robot_status/joint'] = {
                    'j1': round(self.latest_joints[0], 2),
                    'j2': round(self.latest_joints[1], 2),
                    'j3': round(self.latest_joints[2], 2),
                    'j4': round(self.latest_joints[3], 2),
                    'j5': round(self.latest_joints[4], 2),
                    'j6': round(self.latest_joints[5], 2)
                }
                
                # 기울기 (J6)
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)

                # [NEW] 속도 (Velocity) - 가장 빠르게 움직이는 관절 기준 or 평균
                # 여기서는 0~100% 표현을 위해 대략적인 스케일링을 합니다.
                if self.latest_velocities:
                    max_vel_rad = max(self.latest_velocities) # 가장 빠른 관절
                    # 대략 3.0 rad/s를 100%로 잡고 백분율 환산
                    vel_percent = min(int((max_vel_rad / 3.0) * 100), 100)
                    updates['robot_status/velocity'] = vel_percent
                else:
                    updates['robot_status/velocity'] = 0

                # [NEW] 가속도 (Accel)
                updates['robot_status/acceleration'] = round(self.current_accel, 2)

            # 2. 무게 데이터 업데이트
            updates['sensor_data/weight'] = round(self.latest_weight, 2)

            # Firebase 전송
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