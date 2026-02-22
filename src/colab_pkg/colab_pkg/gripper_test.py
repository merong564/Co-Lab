#!/usr/bin/env python3
import time
import rclpy
import DR_init

# ===============================
# 1. 설정 및 상수
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ===============================
# 2. 로봇 초기화
# ===============================
def initialize_robot():
    """로봇 초기화 (툴/TCP/모드 세팅)"""
    from DSR_ROBOT2 import (
        set_tool, set_tcp,
        set_robot_mode, get_robot_mode,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)


# ===============================
# 3. 작업 수행 로직 (단독 실행)
# ===============================
def perform_task(logger=None):
    from DSR_ROBOT2 import (
        movej, movel,
        posj, posx,
        set_digital_output, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        get_digital_input,

        # compliance/force
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_FC_MOD_REL,

        # periodic / force read
        move_periodic, get_tool_force,

        DR_BASE, DR_TOOL
    )

    def log(msg: str):
        if logger is not None:
            logger.info(msg)
        else:
            print(msg)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 속도/가속
    J_VEL, J_ACC = 40, 40
    L_VEL, L_ACC = 100, 100

    # 그리퍼 DO
    ON, OFF = 1, 0

    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)

    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(2.0)
        wait_digital_input(2)

    # [수정] 85mm 열기 동작 함수 정의 (단일 신호 제어 원복 및 실행 순서 변경)
    def release_85mm():
        print("Releasing to 85mm...")
        # [추가] 신호 공백 방지를 위해 새로운 85mm 신호를 먼저 인가 후 기존 신호 해제
        set_digital_output(3, ON)
        set_digital_output(4, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(2.0)
        wait_digital_input(3)

    # Grip 동작
    def grip():
        print("Gripping...")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)


    # =========================
    # 전체 흐름
    # =========================
    
    log("###### 그리퍼 닫기 ######")
    grip()
    log("###### 그리퍼 열기 ######")
    release()
    log("###### 그리퍼 닫기 ######")
    grip()
    release_85mm()
    

# ===============================
# 4. main
# ===============================
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("mix_master", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(logger=node.get_logger())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()