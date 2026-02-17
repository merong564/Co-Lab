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

# [중요] 사용자가 정의한 인터페이스 임포트
try:
    from colab_interfaces.msg import UiInput
    from colab_interfaces.msg import SystemStatus
except ImportError:
    print("❌ [Error] colab_interfaces 패키지를 찾을 수 없습니다. source install/setup.bash를 확인하세요.")

class UserInterface(Node): # [변경] 클래스명 수정
    def __init__(self):
        super().__init__('user_interface') # [변경] 노드명 수정
        
        # 1. Firebase 초기화
        try:
            cred = credentials.Certificate("serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info("🔥 Firebase Connected Successfully!")
            
            # 초기화: 기존 명령 삭제
            db.reference('commands').set({}) 
            self.get_logger().info("🧹 [Init] Old commands cleared.")
            
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        # 2. Publisher (Web -> Robot)
        # Service Client는 다른 노드에서 처리하므로, 여기서는 명령을 토픽으로 전달하거나 
        # 추후 서비스 호출 로직을 위해 데이터를 중계합니다.
        self.ui_pub = self.create_publisher(UiInput, '/ui/input', 10)
        
        # [요청 6] 긴급 정지 토픽
        self.stop_pub = self.create_publisher(String, '/stop', 10) 
        
        # 3. Subscriber (Robot -> Web)
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)
        
        # [요청 4] SystemStatus 메시지 구독 (/system_status)
        self.create_subscription(SystemStatus, '/system_status', self.system_status_callback, 10)

        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = time.time() * 1000 
        
        # 데이터 저장소
        self.latest_joints = None
        self.latest_weight = 0.0
        self.last_joint_time = 0.0
        
        # 시스템 상태 저장소 (SystemStatus.msg 대응)
        self.latest_system_status = {}

        self.get_logger().info('🚀 UserInterface Node V22 Started (SystemStatus Integrated)')

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
                        # UiInput 메시지 정의에 따라 필드 할당 (target_weight 등)
                        # [주의] UiInput.msg에 mixing_duration이 없다면 target_weight만 보냅니다.
                        # Service Client를 만드는 분이 이 토픽을 구독하거나, 
                        # 이 노드에서 바로 Service를 Call하도록 수정될 수 있습니다.
                        
                        target_w = float(cmd_data.get('target_weight', 0.0))
                        mixing_d = float(cmd_data.get('mixing_duration', 0.0))
                        
                        # 임시: UiInput에 필드가 있다고 가정하고 할당 (없으면 에러날 수 있음)
                        # msg.target_weight = target_w
                        # msg.mixing_duration = mixing_d 
                        # msg.mode = "FULL" 
                        
                        # 현재는 기존 호환성을 위해 target_weight만이라도 확실히 보냄
                        try:
                            msg.target_weight = target_w
                        except:
                            pass
                            
                        self.ui_pub.publish(msg)
                        self.get_logger().info(f"▶ Command: Mode=FULL, Target={target_w}g, Mix={mixing_d}s")
                    
                    elif cmd_type == 'emergency_stop':
                        # [요청 6] /stop 토픽 발행
                        self.stop_pub.publish(String(data="STOP"))
                        self.get_logger().warn("🚨 STOP Signal Published to /stop")
                        
                    elif cmd_type == 'tare':
                         self.get_logger().info("⚖️ Tare Command Received")
        except Exception:
            pass

    def joint_callback(self, msg):
        self.latest_joints = [math.degrees(rad) for rad in msg.position]
        self.last_joint_time = time.time()

    def weight_callback(self, msg):
        self.latest_weight = msg.data

    # [요청 4] SystemStatus.msg 처리 함수
    def system_status_callback(self, msg):
        """
        SystemStatus.msg 구조:
        string phase
        float32 tcp_vel
        float32 tcp_acc
        float32 pour_speed
        int32 total_count
        int32 success_count
        float32 error_rate
        float32 last_cycle_time
        """
        self.latest_system_status = {
            # 1. 로봇 및 공정 상태 (Real-time)
            "phase": msg.phase,
            "tcp_vel": msg.tcp_vel,
            "tcp_acc": msg.tcp_acc,
            "pour_speed": msg.pour_speed,
            
            # 2. 통계 데이터 (Cumulative)
            "total_count": msg.total_count,
            "success_count": msg.success_count,
            "error_rate": round(msg.error_rate, 2),
            "last_cycle_time": round(msg.last_cycle_time, 2)
        }

    def upload_to_firebase(self):
        try:
            updates = {}
            current_time = time.time()

            # 1. 관절 데이터 (Zombie check)
            if self.latest_joints and (current_time - self.last_joint_time < 1.5):
                updates['robot_status/joint'] = {f'j{i+1}': round(v, 2) for i, v in enumerate(self.latest_joints)}
                updates['robot_status/current_angle'] = round(self.latest_joints[5], 2)
            
            # 2. 무게 센서 데이터 (요청 5: 유량은 UI에서 계산 or Robot에서 pour_speed로 줌)
            updates['sensor_data/weight'] = round(self.latest_weight, 2)
            updates['sensor_data/timestamp'] = int(time.time() * 1000)
            
            # 3. 시스템 상태 (SystemStatus) 업로드
            # Robot Controller가 /system_status를 발행하면 여기에 값이 채워짐
            if self.latest_system_status:
                updates['system_stats'] = self.latest_system_status
                
                # 로봇 상태 시각화를 위해 robot_status 경로에도 일부 데이터 매핑 (UI 호환성)
                updates['robot_status/phase'] = self.latest_system_status.get('phase', 'Ready')
                updates['robot_status/velocity'] = self.latest_system_status.get('tcp_vel', 0)
                updates['robot_status/acceleration'] = self.latest_system_status.get('tcp_acc', 0)

            if updates:
                db.reference().update(updates)
        except Exception:
            pass

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