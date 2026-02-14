import rclpy
import DR_init
import time
import sys

# ROS2 메시지
from std_msgs.msg import Float32, String

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class PouringController:
    def __init__(self, node):
        self.node = node
        self.current_weight = 0.0
        self.target_weight = 0.0 # 초기값 0 (UI에서 받아옴)
        
        # 상태 플래그
        self.cmd_received = False   # 명령 받았나?
        self.is_emergency = False   # 긴급정지인가?
        self.task_done = False      # 목표달성 했나?

        # 구독 설정
        self.node.create_subscription(Float32, '/loadcell_weight', self.weight_cb, 10)
        self.node.create_subscription(String, '/ui_command', self.command_cb, 10)
        self.node.create_subscription(Float32, '/target_weight', self.target_cb, 10)

    def weight_cb(self, msg):
        self.current_weight = msg.data

    def target_cb(self, msg):
        self.target_weight = msg.data

    def command_cb(self, msg):
        cmd = msg.data
        if cmd == "START":
            self.cmd_received = True
            self.is_emergency = False
            self.task_done = False
            print(f"\n▶ [Signal] 시작 신호 수신! (목표: {self.target_weight}g)")
        elif cmd == "STOP":
            self.is_emergency = True
            self.cmd_received = False
            print("\n🚨 [Signal] 긴급 정지 신호 수신!")

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def should_stop(self):
        """움직임을 계속할지 검사 (True면 멈춰야 함)"""
        self.spin_once()
        
        # 1. 긴급 정지 체크
        if self.is_emergency:
            return True
        
        # 2. 목표 달성 체크 (목표값이 설정되어 있고, 현재 무게가 목표 이상일 때)
        if self.target_weight > 0 and self.current_weight >= self.target_weight:
            if not self.task_done:
                print(f"✅ 목표 달성! ({self.current_weight:.1f}g / {self.target_weight}g)")
                self.task_done = True
            return True
            
        return False

def main(args=None):
    # 1. ROS2 초기화 및 노드 생성 (가장 먼저 해야 함!)
    rclpy.init(args=args)
    node = rclpy.create_node("pouring_simulation", namespace=ROBOT_ID)
    
    # 2. DR_init에 노드 등록
    DR_init.__dsr__node = node

    # 3. 라이브러리 임포트 (노드 등록 후)
    try:
        from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    # 4. 컨트롤러 생성
    controller = PouringController(node)

    # 5. 로봇 자율 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    
    # 자세 정의 (J1, J2, J3, J4, J5, J6)
    p_ready = posj(0, 0, 90, 0, 90, 0)   # 수평 (대기)
    p_pour = posj(0, 0, 90, 0, 90, 80)   # 붓기 (80도)

    print("📡 [Robot] 대기 모드 진입... UI 시작 명령을 기다립니다.")

    # 여기가 에러가 났던 부분입니다. try와 except 줄 맞춤이 중요합니다.
    try:
        while rclpy.ok():
            controller.spin_once()

            # [상태 1] 명령 대기 중
            if not controller.cmd_received:
                time.sleep(0.1)
                continue 

            # [상태 2] 시작 명령 수신 -> 실험 시작
            print("🚀 실험 사이클 진입!")
            
            # (1) 준비 자세로 이동
            if controller.should_stop(): break 
            movej(p_ready, vel=60, acc=60)
            
            # (2) 붓기 동작 루프
            while not controller.should_stop() and not controller.is_emergency:
                # 붓기 전 체크
                if controller.should_stop(): break

                print(f"🚰 붓기 시도 (현재: {controller.current_weight:.1f}g)")
                movej(p_pour, vel=60, acc=60)
                
                # 붓는 동안(3초) 대기하며 계속 감시
                for _ in range(30):
                    wait(0.1)
                    if controller.should_stop(): break
                
                # 다시 세우기 (잠그기)
                movej(p_ready, vel=60, acc=60)
                wait(1.0) # 안정화
                
                # 목표 달성했으면 루프 탈출
                if controller.task_done: break

            # 실험 종료 후 처리
            print("🏁 사이클 종료. 다시 대기 모드로 전환합니다.")
            movej(p_ready, vel=30, acc=30)
            
            # 변수 초기화 (다음 실험을 위해)
            controller.cmd_received = False 
            controller.task_done = False

    except KeyboardInterrupt:
        print("종료 요청됨.")
    finally:
        # 종료 처리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()