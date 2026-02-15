#!/usr/bin/env python3
# 시험관을 들고 비커 앞 위치로 이동하는 코드
import rclpy
import DR_init
import time


def main(args=None):
    # ===============================
    # 0) 로봇 기본 설정
    # ===============================
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    rclpy.init(args=args)
    node = rclpy.create_node("test1_pour_flow_joint", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej, posj,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_digital_output,
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ===============================
    # 1) 속도 / 가속
    # ===============================
    J_VEL, J_ACC = 60, 60

    # ===============================
    # 2) 그리퍼 제어 (DO)
    # ===============================
    DO_OPEN = 1
    DO_CLOSE = 2

    # 요구 스펙(참고용): DO 그리퍼면 실제 force/거리 제어는 보통 안 됨
    GRIP_FORCE_N = 40
    GRIP_DEPTH_MM = 15

    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)
        time.sleep(0.3)

    def gripper_close():
        # (force=40N, stroke=15mm 같은 제어가 되는 전동그리퍼면
        #  여기서 vendor API로 명령 내려야 함. 지금은 DO라 ON/OFF만.)
        set_digital_output(DO_OPEN, 0)
        set_digital_output(DO_CLOSE, 1)
        time.sleep(0.4)

    # ===============================
    # 3) 조인트 자세 정의 (deg)
    # ===============================
    # Ready 자세: "기존 코드 동일"이라고 했으니 그대로 유지
    ready_j = posj(0, 0, 90.0, 0, 90.0, 0)

    # 시험관 잡기 전 주변 위치(approach)
    approach_j = posj(-37.328, 56.203, 47.703, 76.172, 125.663, -15.473)

    # 시험관 잡는 위치(pick)
    pick_j = posj(-37.343, 64.778, 52.493, 67.632, 121.528, -30.756)

    # 시험관 잡은 상태에서 위로 올린 위치(lift) - 너가 준 값 그대로
    lift_j = posj(-37.328, 56.203, 47.703, 76.172, 125.663, -15.473)

    # 비커 앞 대기 위치
    pour_wait_j = posj(-16.851, 46.494, 64.413, 75.866, 104.317, -25.299)

    # ===============================
    # 4) 동작 시퀀스 (단 1회)
    # ===============================
    try:
        # 1) 초기자세(ready)
        movej(ready_j, vel=J_VEL, acc=J_ACC)

        # 2) 그리퍼 열기
        gripper_open()

        # 3) 시험관 잡기 전 주변 위치로 이동
        movej(approach_j, vel=J_VEL, acc=J_ACC)

        # 4) 시험관 잡는 위치로 이동
        movej(pick_j, vel=J_VEL, acc=J_ACC)

        # 5) 잡기 (40N/15mm 요구 스펙)
        gripper_close()

        # 6) 잡은 상태로 위로 올리기
        movej(lift_j, vel=J_VEL, acc=J_ACC)

        # 7) 비커 앞 대기 위치로 이동
        movej(pour_wait_j, vel=J_VEL, acc=J_ACC)

        print("✔ Joint sequence finished (single run)")

    except Exception as e:
        print(f"[ERROR] {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
