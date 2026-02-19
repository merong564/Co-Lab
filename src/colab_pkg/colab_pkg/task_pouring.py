#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

from colab_interfaces.srv import RobotCommand
from std_msgs.msg import Float32, String

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60
P_GAIN = 0.03
MAX_TILT_STEP = 3.0
STOP_THRESHOLD = 40.0

# ✅ STOP 신호 플래그 (STOP 토픽 받으면 True)
STOP_REQUESTED = False

# ==========================================
# 2. 통신 전담 노드 (서비스 & 토픽)
# ==========================================
class TaskPouring(Node):
    def __init__(self):
        super().__init__("task_pouring", namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()
        self.current_weight = 0.0

        # 서비스 서버
        self.srv_pouring = self.create_service(
            RobotCommand,
            "execute_pouring",
            self.execute_pouring_callback,
            callback_group=self.callback_group,
        )

        # 무게 구독
        self.sub_weight = self.create_subscription(
            Float32,
            "load_cell/weight",
            self.weight_callback,
            10,
            callback_group=self.callback_group,
        )

        # STOP 토픽 구독 (/dsr01/stop)
        self.sub_stop = self.create_subscription(
            String,
            "stop",
            self.stop_callback,
            10,
            callback_group=self.callback_group,
        )

    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        if (msg.data or "").strip().upper() != "STOP":
            return

        STOP_REQUESTED = True
        self.get_logger().warn("🚨 STOP received -> will return to ready pose then finish")

    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Target: {request.target_weight}g")

        success = perform_task(self, float(request.target_weight))

        response.success = bool(success)
        response.message = "Pouring Completed" if success else "Pouring Failed"
        return response

    def weight_callback(self, msg: Float32):
        self.current_weight = float(msg.data)

# ==========================================
# 3. 로봇 제어 로직 (DSR 라이브러리 사용)
# ==========================================
def initialize_robot():
    from DSR_ROBOT2 import (
        set_tool,
        set_tcp,
        set_robot_mode,
        ROBOT_MODE_MANUAL,
        ROBOT_MODE_AUTONOMOUS,
    )

    time.sleep(3.0)  # 노드 연결 대기(안전)
    try:
        print("[Thread] Initializing Robot settings...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f"✅ [Thread] Robot Initialized: {ROBOT_ID}")
    except Exception as e:
        print(f"❌ [Thread] Init Failed: {e}")

def calculate_tilt_angle(current_w: float, target_w: float):
    error = target_w - current_w
    delta_angle = error * P_GAIN

    if delta_angle > MAX_TILT_STEP:
        delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP:
        delta_angle = -MAX_TILT_STEP

    return float(delta_angle), float(error)

def perform_task(node: TaskPouring, target_weight: float) -> bool:
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait

    global STOP_REQUESTED

    print(f"[SYSTEM] Task Start! Target: {target_weight}g")

    pour_ready_pos = posx(585.44, 157.76, 242.63, 91.92, 97.36, 88.55)

    # 시작 자세로 이동
    try:
        movel(pour_ready_pos, vel=100, acc=100)
        wait(1.0)
    except Exception as e:
        print(f"[ERROR] Move Failed: {e}")
        return False

    stop_target = target_weight - STOP_THRESHOLD

    while rclpy.ok():
        # ✅ STOP 들어오면: 자세 복귀 후 종료(노드 유지)
        if STOP_REQUESTED:
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed after STOP: {e}")
                STOP_REQUESTED = False
                return False

            print("🟡 [STOP] Returned to ready pose. Finishing task.")
            STOP_REQUESTED = False
            return True  # STOP을 '정상 종료'로 볼지 여부(원하면 False로)

        current_weight = float(node.current_weight)

        # 목표 근처 도달하면 복귀 자세로 이동 후 종료
        if current_weight >= stop_target:
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed: {e}")
                return False

            print(f"✅ [Done] Final: {current_weight:.1f}g (stop_target={stop_target:.1f}g)")
            return True

        delta, error = calculate_tilt_angle(current_weight, target_weight)

        try:
            current_joints = get_current_posj()
            if not current_joints:
                print("[ERROR] get_current_posj() returned empty.")
                return False

            target_joints = list(current_joints)
            target_joints[5] += delta  # J6

            movej(target_joints, vel=VELOCITY, acc=ACC)
            print(f"Cur: {current_weight:.1f} | Delta: {delta:.2f} | Err: {error:.1f}")
            time.sleep(0.1)

        except Exception as e:
            print(f"[ERROR] Tilt Move Failed: {e}")
            return False

    return False

# ==========================================
# 4. 메인 (노드 분리 전략 적용)
# ==========================================
def main(args=None):
    rclpy.init(args=args)

    # 1) 로봇 제어용 노드 (DSR이 독점)
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)

    # 2) 통신용 노드 (서비스/무게/STOP)
    task_node = TaskPouring()

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("==========================================")
    print(" [Ready] Service Server Started (Multi-Node) ")
    print(f" - service: /{ROBOT_ID}/execute_pouring")
    print(f" - weight : /{ROBOT_ID}/load_cell/weight (Float32)")
    print(f" - stop   : /{ROBOT_ID}/stop (String, data='STOP')")
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
