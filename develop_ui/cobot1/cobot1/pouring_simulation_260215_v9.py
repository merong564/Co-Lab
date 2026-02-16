import rclpy
import DR_init
import time
import sys
import json

from std_msgs.msg import Float32, String

try:
    from colab_interfaces.msg import UiInput
except ImportError:
    print("❌ [CRITICAL ERROR] 'colab_interfaces' 패키지를 찾을 수 없습니다!")
    sys.exit(1)

# ==========================================
# 1. 설정 및 상수 (팀원 코드값 유지)
# ==========================================
# 🛠️ 해결책: V9 업데이트 (데이터 콸콸 보내기)
# 로봇이 움직이는 동안 속도 정보를 계속해서 보내도록 수정하면 UI에 40%, 60%가 선명하게 뜰 겁니다.
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 40       
ACC = 60            
P_GAIN = 0.03        
MAX_TILT_STEP = 3.0  
STOP_THRESHOLD = 40.0 
LOOP_DELAY = 0.1     

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# 2. 로봇 컨트롤러 클래스
# ==========================================
class VirtualTiltingController:
    def __init__(self, node):
        self.node = node
        self.current_weight = 0.0
        self.target_weight = 0.0
        self.cmd_received = False
        self.is_emergency = False
        
        self.node.create_subscription(Float32, '/loadcell_weight', self.weight_callback, 10)
        self.node.create_subscription(UiInput, '/ui/input', self.ui_callback, 10)
        self.node.create_subscription(String, '/ui/stop', self.stop_callback, 10)
        self.status_pub = self.node.create_publisher(String, '/pour_system/status', 10)

    def weight_callback(self, msg):
        self.current_weight = msg.data

    def ui_callback(self, msg):
        if msg.is_confirmed:
            self.target_weight = msg.target_weight
            self.cmd_received = True
            self.is_emergency = False
            print(f"📩 [UI Command] 목표 무게 수신: {self.target_weight}g")

    def stop_callback(self, msg):
        if msg.data == "STOP":
            self.is_emergency = True
            print("🚨 [Emergency] 긴급 정지!")

    def publish_status(self, phase):
        msg = String()
        # [핵심] 현재 설정된 속도/가속도 값을 JSON으로 포장해서 보냄
        msg.data = json.dumps({"phase": phase, "vel": VELOCITY, "acc": ACC})
        self.status_pub.publish(msg)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

# ==========================================
# 3. 메인 실행 로직
# ==========================================
def calculate_tilt_angle(current_w, target_w):
    error = target_w - current_w
    delta_angle = error * P_GAIN
    if delta_angle > MAX_TILT_STEP: delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP: delta_angle = -MAX_TILT_STEP
    return delta_angle, error

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pouring_simulation_real_sync_v9", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    try:
        from DSR_ROBOT2 import movej, get_current_posj, movel, posj, wait, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    except ImportError as e:
        print(f"❌ DSR Library Import Error: {e}")
        return

    controller = VirtualTiltingController(node)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)
    
    pour_ready_pos = posj(0, 0, 90, 0, 90, 0)
    
    print("🤖 [System V9] 속도 정보 실시간 전송 패치 완료.")
    controller.publish_status("Ready")

    try:
        while rclpy.ok():
            controller.spin_once()

            if not controller.cmd_received:
                # 대기 중에도 상태 전송 (UI가 새로고침되어도 0 안 뜨게)
                controller.publish_status("Ready")
                time.sleep(0.5)
                continue
            
            target_weight = controller.target_weight
            print(f"🚀 [Start] 목표: {target_weight}g")
            
            controller.publish_status("Approach")
            movej(pour_ready_pos, vel=100, acc=100)
            wait(1.0)
            
            step_count = 0
            
            # [수정 포인트] 루프 진입 전 한 번 보냄
            controller.publish_status("Pouring")

            while rclpy.ok() and not controller.is_emergency:
                controller.spin_once() 
                current_weight = controller.current_weight
                
                # [수정 포인트] 루프 돌 때마다 계속 상태(속도/가속도) 전송!
                # 이제 UI가 데이터를 놓쳐도 다음 턴에 다시 받습니다.
                controller.publish_status("Pouring")

                # (A) 미리 멈춤 체크
                stop_target = target_weight - STOP_THRESHOLD
                if current_weight >= stop_target:
                    print(f"🛑 [Stop Trigger] {current_weight:.1f}g 도달!")
                    controller.publish_status("Return")
                    
                    movej(pour_ready_pos, vel=150, acc=150)
                    
                    wait(2.0)
                    controller.spin_once()
                    
                    print(f"✅ [Done] 최종 무게: {controller.current_weight:.1f}g")
                    controller.publish_status("Done")
                    break

                # (B) P제어
                delta, error = calculate_tilt_angle(current_weight, target_weight)
                current_joints = get_current_posj()
                
                if current_joints is not None:
                    target_joints = [x for x in current_joints]
                    target_joints[5] += delta
                    if target_joints[5] > 90: target_joints[5] = 90
                    if target_joints[5] < 0: target_joints[5] = 0
                    
                    movej(target_joints, vel=VELOCITY, acc=ACC)
                    
                    print(f"[Step {step_count}] 무게: {current_weight:.1f}g | 델타: {delta:.2f}°")
                    step_count += 1
                    
                    wait(LOOP_DELAY) 
                else:
                    time.sleep(0.1)

            controller.cmd_received = False
            controller.is_emergency = False
            print("🔄 [System] 대기 모드 전환")
            controller.publish_status("Ready")

    except KeyboardInterrupt:
        print("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()