#!/usr/bin/env python3
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
    node = rclpy.create_node("test1_pour_flow", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej, movel, posj, posx,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        set_digital_output
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ===============================
    # 1) 속도 / 가속
    # ===============================
    J_VEL, J_ACC = 60, 60
    L_VEL, L_ACC = 150, 150

    # ===============================
    # 2) 그리퍼 제어
    # ===============================
    DO_OPEN = 1
    DO_CLOSE = 2

    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)
        time.sleep(0.3)

    def gripper_close():
        set_digital_output(DO_OPEN, 0)
        set_digital_output(DO_CLOSE, 1)
        time.sleep(0.4)

    # ===============================
    # 3) 자세 / 좌표 정의
    # ===============================

    # Ready 자세
    ready_j = posj(0, 0, 90.0, 0, 90.0, 0)

    # IK 해 고정용 조인트 자세 (손목 180도 방지)
    pre_pick_j = posj(10.0, -30.0, 100.0, 0.0, 90.0, 0.0)

    # 시험관 위치 (임의값)
    tube_x, tube_y, tube_z = 420.0, 250.0, 160.0

    # ✅ 옆면 집기 자세 (TCP 기준, 절대 변경 안 함)
    RX, RY, RZ = 0.0, 90.0, 0.0

    # 시험관 접근 / 집기 / 리프트
    tube_approach_pos = posx(tube_x, tube_y - 50.0, tube_z, RX, RY, RZ)
    tube_pick_pos     = posx(tube_x, tube_y,        tube_z, RX, RY, RZ)
    tube_lift_pos     = posx(tube_x, tube_y, tube_z + 100.0, RX, RY, RZ)

    # 비커 앞 대기 위치 (❗ 자세는 시험관 자세 그대로 유지)
    pour_ready_pos = posx(
        604.44, 157.76, 242.63,
        RX, RY, RZ
    )

    # ===============================
    # 4) 동작 시퀀스 (단 1회)
    # ===============================
    try:
        # Ready
        movej(ready_j, vel=J_VEL, acc=J_ACC)

        # 그리퍼 열기
        gripper_open()

        # 해 고정 (손목 뒤집힘 방지)
        movej(pre_pick_j, vel=J_VEL, acc=J_ACC)

        # 시험관 접근 → 집기
        movel(tube_approach_pos, vel=L_VEL, acc=L_ACC)
        movel(tube_pick_pos,     vel=L_VEL, acc=L_ACC)

        # 그리퍼 닫기 (힘 21 가정)
        gripper_close()

        # 들어올리기 (자세 유지)
        movel(tube_lift_pos, vel=L_VEL, acc=L_ACC)

        # 비커 위치로 이동 (자세 절대 유지)
        movel(pour_ready_pos, vel=L_VEL, acc=L_ACC)

        print("✔ Sequence finished (single run)")

    except Exception as e:
        print(f"[ERROR] {e}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
