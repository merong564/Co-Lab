#!/usr/bin/env python3
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# [중요] DR_init은 main에서 설정 후 사용
import DR_init

from colab_interfaces.srv import RobotCommand

# ===============================
# 1. 설정 및 상수
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 속도/가속도
L_VEL, L_ACC = 150, 150

# 그리퍼 DO 채널
DO_OPEN = 1
DO_CLOSE = 2

# ===============================
# 2. 통신 전담 노드 (서비스 서버)
# ===============================
class TaskTransfer(Node):
    def __init__(self):
        super().__init__("task_transfer", namespace=ROBOT_ID)
        
        self.callback_group = ReentrantCallbackGroup()
        self._busy = False  # [안전 장치] 중복 명령 방지

        # 서비스 서버 생성
        self.srv = self.create_service(
            RobotCommand, 
            "execute_transfer", 
            self._on_command,
            callback_group=self.callback_group
        )
        self.get_logger().info("TaskTransfer Ready. Service: /dsr01/execute_transfer")

    def _on_command(self, request, response):
        # [안전 장치] 로봇이 움직이는 중이면 명령 거절
        if self._busy:
            response.success = False
            response.message = "BUSY: Robot is currently moving."
            return response

        self._busy = True
        try:
            mode = (getattr(request, "mode", "") or "").strip().upper()
            
            # [수정 1] tube_type을 사용자 입력으로 받음 (RobotCommand.srv에 string tube_type 추가 필요)
            tube_type = (getattr(request, "tube_type", "") or "LARGE").strip().upper()
            
            target_weight = getattr(request, "target_weight", 0.0)
            mixing_duration = getattr(request, "mixing_duration", 0.0)

            self.get_logger().info(f"[Service] {mode} / Tube: {tube_type} / W: {target_weight}")

            # 로봇 제어 함수 호출
            success, msg = perform_task(mode, tube_type, target_weight, mixing_duration)

            response.success = success
            response.message = msg

        except Exception as e:
            self.get_logger().error(f"Service Error: {e}")
            response.success = False
            response.message = f"ERROR: {e}"
        finally:
            self._busy = False # 작업 완료 후 깃발 내림

        return response

# ===============================
# 3. 로봇 제어 로직
# ===============================
def get_poses(posx_func):
    return {
        "SMALL": {
            "PICK_DOWN": posx_func(555.786, -78.524, 126.047, 90.674, 92.519, 93.656),
            "PICK_UP":   posx_func(555.784, -78.523, 259.725, 90.674, 92.518, 93.657),
            "POUR_READY": posx_func(604.441, 157.760, 242.631, 91.920, 97.360, 88.550),
        },
        "LARGE": {
            "PICK_DOWN": posx_func(306.636, -66.725,  89.141, 91.356, 91.786, 90.102),
            "PICK_UP":   posx_func(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
            "POUR_READY": posx_func(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
        }
    }

def initialize_robot():
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
    time.sleep(2.0)
    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print("✅ [Robot] Initialized: Autonomous Mode")
    except Exception as e:
        print(f"❌ [Robot] Init Failed: {e}")

def perform_task(mode, tube_type, target_weight, mixing_duration):
    from DSR_ROBOT2 import movel, posx, wait, set_digital_output, DR_BASE

    try:
        POSES = get_poses(posx)
    except Exception:
        return False, "DSR_NOT_READY"

    # [수정 1] 입력받은 tube_type 검증
    if tube_type not in POSES:
        return False, f"INVALID_TUBE_TYPE: {tube_type}"

    def gripper(action):
        if action == "OPEN":
            set_digital_output(DO_CLOSE, 0)
            set_digital_output(DO_OPEN, 1)
            time.sleep(0.3)
        elif action == "CLOSE":
            set_digital_output(DO_OPEN, 0)
            set_digital_output(DO_CLOSE, 1)
            time.sleep(0.4)

    # 동작 시퀀스
    try:
        P = POSES[tube_type]

        if mode == "PICKUP":
            print(f"[Action] PICKUP Start ({tube_type})")
            
            # [수정 2] goL 제거하고 movel 직접 사용
            gripper("OPEN")
            movel(P["PICK_UP"],   vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Approach
            movel(P["PICK_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Insert
            gripper("CLOSE")
            movel(P["PICK_UP"],   vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Lift
            movel(P["POUR_READY"], vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Beaker
            return True, "PICKUP_DONE"

        elif mode == "RETURN":
            print(f"[Action] RETURN Start ({tube_type})")
            
            # [수정 2] goL 제거하고 movel 직접 사용
            movel(P["PICK_UP"],   vel=L_VEL, acc=L_ACC, ref=DR_BASE) # RackTop
            movel(P["PICK_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Place
            gripper("OPEN")
            wait(0.5)
            movel(P["PICK_UP"],   vel=L_VEL, acc=L_ACC, ref=DR_BASE) # Retract
            return True, "RETURN_DONE"
        
        else:
            return False, f"INVALID_MODE: {mode}"

    except Exception as e:
        print(f"❌ [Action] Failed: {e}")
        return False, f"EXECUTION_ERROR: {e}"

# ===============================
# 4. 메인
# ===============================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskTransfer()

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("==========================================")
    print(" [TaskTransfer] Service Server Started ")
    print("==========================================")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()