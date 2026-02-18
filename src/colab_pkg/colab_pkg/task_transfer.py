#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

# ===============================
# 1. 설정 및 상수
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도/가속도 (안전하게 설정)
VEL, ACC = 60, 60
DO_OPEN = 1
DO_CLOSE = 2

# DSR_init 설정 (전역)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===============================
# 2. 로봇 제어 로직
# ===============================
def get_poses(posx_func):
    return {
        "LARGE": {
            "PICK_DOWN": posx_func(306.636, -66.725,  89.141, 91.356, 91.786, 90.102),
            "PICK_UP":   posx_func(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
            "POUR_READY": posx_func(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
        }
    }

def initialize_robot():
    """로봇 초기화 (grip_simple 예제와 동일한 방식)"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    # 1. 매뉴얼 모드 전환
    set_robot_mode(ROBOT_MODE_MANUAL)
    
    # 2. 툴/TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    # 3. 자율 모드 전환
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0) # 모드 변경 안정화 대기

    # 상태 출력
    print("#" * 50)
    print(f"✅ Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)

def perform_task(mode, tube_type):
    """작업 수행 로직"""
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_BASE

    # 좌표 데이터 로드
    try:
        POSES = get_poses(posx)
    except Exception:
        print("❌ Error: DSR Library not ready")
        return

    # 안전한 초기 자세 (Joint)
    J_READY = [0, 0, 90, 0, 90, 0]

    def gripper(action):
        if action == "OPEN":
            set_digital_output(DO_CLOSE, 0)
            set_digital_output(DO_OPEN, 1)
            wait(0.5)
        elif action == "CLOSE":
            set_digital_output(DO_OPEN, 0)
            set_digital_output(DO_CLOSE, 1)
            wait(0.5)

    try:
        P = POSES[tube_type]

        # [안전 장치] 작업 시작 전 movej로 관절을 풀어줌 (특이점 회피)
        print(f"🚀 Moving to Ready Pose (J)")
        movej(J_READY, vel=VEL, acc=ACC)
        wait(0.5)

        # 작업 수행
        if mode == "PICKUP":
            print(f"[Action] PICKUP Start ({tube_type})")
            gripper("OPEN")
            
            # sync=True는 기본값이지만 명시적으로 확인
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
            gripper("CLOSE")
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)
            print("[Action] PICKUP Done")

        elif mode == "RETURN":
            print(f"[Action] RETURN Start ({tube_type})")
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
            gripper("OPEN")
            wait(0.5)
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            
            # 복귀
            movej(J_READY, vel=VEL, acc=ACC)
            print("[Action] RETURN Done")

    except Exception as e:
        print(f"❌ [Action] Failed: {e}")

# ===============================
# 3. 메인 (grip_simple 구조 따름)
# ===============================
def main(args=None):
    rclpy.init(args=args)

    # [수정] grip_simple 처럼 namespace를 명시합니다.
    node = rclpy.create_node("task_test_node", namespace=ROBOT_ID)

    # DSR_init에 노드 주입
    DR_init.__dsr__node = node

    try:
        # 초기화
        initialize_robot()

        print("\n=== [Test] Start Sequence ===")
        
        # 동작 실행
        perform_task("PICKUP", "LARGE")
        time.sleep(1.0)
        perform_task("RETURN", "LARGE")

        print("=== [Test] End Sequence ===\n")

    except Exception as e:
        print(f"Main Error: {e}")
    
    finally:
        # 종료 처리
        rclpy.shutdown()

if __name__ == "__main__":
    main()