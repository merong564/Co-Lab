#!/usr/bin/env python3
# 시험관을 붓고나서 시험관을 제자리로 원위치 하는 코드
import rclpy
import DR_init


def main(args=None):
    # ===============================
    # 0) 로봇 기본 설정
    # ===============================
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    rclpy.init(args=args)
    node = rclpy.create_node("tube_release_and_home", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej, posj, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_digital_output
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ===============================
    # 1) 속도 / 가속
    # ===============================
    J_VEL, J_ACC = 30, 30

    # ===============================
    # 2) 그리퍼 제어 (DO)
    # ===============================
    DO_OPEN = 1
    DO_CLOSE = 2

    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)

    # ===============================
    # 3) 조인트 좌표 (deg)
    # ===============================
    j1 = posj(-37.328, 56.203, 47.703, 76.172, 125.663, -15.473)
    j2 = posj(-37.343, 64.778, 52.493, 67.632, 121.528, -30.756)

    ready_j = posj(0, 0, 90.0, 0, 90.0, 0)

    # ===============================
    # 4) 시퀀스
    # ===============================
    try:
        movej(j1, vel=J_VEL, acc=J_ACC)
        movej(j2, vel=J_VEL, acc=J_ACC)

        # 놓기
        gripper_open()

        wait(2.0)   # 안정 대기

        # 복귀
        movej(j1, vel=J_VEL, acc=J_ACC)
        movej(ready_j, vel=J_VEL, acc=J_ACC)

        print("✔ Release → back to j1 → home finished")

    except Exception as e:
        print(f"[ERROR] {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
