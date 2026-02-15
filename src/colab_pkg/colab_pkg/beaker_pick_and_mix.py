#!/usr/bin/env python3
import rclpy
import DR_init

# ===============================
# 로봇 기본 설정 (필수)
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def perform_task(VELOCITY=60, ACC=60):
    from DSR_ROBOT2 import (
        movej, posj,
        set_digital_output,
        move_periodic, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
    )

    # 로봇 모드
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ===============================
    # 1) 그리퍼 설정
    # ===============================
    DO_OPEN = 1
    DO_CLOSE = 2

    def gripper_close():
        set_digital_output(DO_OPEN, 0)
        set_digital_output(DO_CLOSE, 1)

    # (필요하면 오픈도 같이 둬)
    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)

    # 시작 시 안전하게 오픈(원하면 주석처리)
    gripper_open()
    wait(0.2)

    # ===============================
    # 2) 조인트 좌표
    # ===============================
    beaker_approach_j = posj(-17.561, 71.881, 35.950, 76.814, 105.046, -21.195)
    beaker_pick_j     = posj(-15.252, 70.070, 39.779, 77.202, 102.451, -22.726)
    beaker_lift_j     = posj( 15.228, 63.002, 32.623, 80.680, 105.138,  -8.335)

    # ===============================
    # 3) 비커 잡기
    # ===============================
    movej(beaker_approach_j, vel=VELOCITY, acc=ACC)
    movej(beaker_pick_j,     vel=VELOCITY, acc=ACC)

    gripper_close()
    wait(0.3)

    movej(beaker_lift_j, vel=VELOCITY, acc=ACC)
    wait(0.3)

    # ===============================
    # 4) 천천히 휘젓기
    # ===============================

    # 1단계: 좌우 흔들기 (안전 테스트용)
    R_MM = 10.0         # 원 반지름 느낌(진폭). 4~8mm부터 추천
    PERIOD_SEC = 3.5    # 한 바퀴 시간. 클수록 더 천천히
    REPEAT = 10         # 바퀴 수
    ATIME = 0.6         # 가감속 시간(부드럽게)

    move_periodic(
        amp=[R_MM, R_MM, 0, 0, 0, 0],   # X,Y만 움직이고 Z는 0으로 고정
        period=PERIOD_SEC,
        atime=ATIME,
        repeat=REPEAT
    )

    wait(0.5)

def main(args=None):
    rclpy.init(args=args)

    # 노드 생성 (필수)
    node = rclpy.create_node("m0609_beaker_stir", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        perform_task(VELOCITY=60, ACC=60)
    except Exception as e:
        node.get_logger().error(f"Task failed: {e}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
