#!/usr/bin/env python3
import rclpy
import DR_init
import math

# ===============================
# 0) 로봇 기본 설정
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def perform_task():
    from DSR_ROBOT2 import (
        # motion
        movej, movel, movec,
        posj, posx,
        get_current_posx,
        # io / etc
        set_digital_output, wait,
        # mode / ref
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        DR_BASE
    )

    # ===============================
    # 1) 로봇 모드 + 속도
    # ===============================
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    J_VEL, J_ACC = 60, 60          # 조인트 이동(접근/픽)
    L_VEL, L_ACC = 60, 60          # 선형/원호(리프트/휘젓기)
    BLEND = 2.0

    # ===============================
    # 2) 그리퍼 설정 (DO 번호는 현장에 맞게)
    # ===============================
    DO_OPEN = 1
    DO_CLOSE = 2

    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)

    def gripper_close():
        set_digital_output(DO_OPEN, 0)
        set_digital_output(DO_CLOSE, 1)

    # ===============================
    # 3) 조인트 좌표 (네가 준 값)
    # ===============================
    beaker_approach_j = posj(-17.561, 71.881, 35.950, 76.814, 105.046, -21.195)
    beaker_pick_j     = posj(-15.252, 70.070, 39.779, 77.202, 102.451, -22.726)

    # ===============================
    # 4) 비커 잡기
    # ===============================
    gripper_open()
    wait(0.2)

    movej(beaker_approach_j, vel=J_VEL, acc=J_ACC)
    wait(0.5)
    movej(beaker_pick_j,     vel=J_VEL, acc=J_ACC)

    gripper_close()
    wait(0.5)

    # ===============================
    # 5) 조인트 목표로 리프트 (네가 준 값으로 이동)
    # ===============================
    lift_j = posj(-15.228, 63.002, 32.623, 80.680, 105.138, -8.335)
    movej(lift_j, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    # ===============================
    # 6) 자세 고정 + XY 원 궤적만 (rz 포함 orientation 변화 없음)
    # ===============================
    REF = DR_BASE
    cur, _ = get_current_posx(ref=REF)
    cx, cy, cz, rx0, ry0, rz0 = cur   # 현재 자세(orientation) 고정

    R = 50.0        # 원 반지름(mm)  -> (x-cx)^2 + (y-cy)^2 = R^2
    TURNS = 3      # 몇 바퀴
    STEPS = 60    # 1바퀴를 몇 점으로 쪼갤지 (클수록 더 부드러움)
    BLEND = 2.0    # movel radius(블렌딩)

    # 시작점(원 오른쪽)으로 자연스럽게 진입
    movel(posx(cx + R, cy, cz, rx0, ry0, rz0),
          vel=L_VEL, acc=L_ACC, ref=REF, radius=0.0)

    total_steps = TURNS * STEPS
    for i in range(1, total_steps + 1):
        th = 2.0 * math.pi * (i / STEPS)   # 0~2pi 가 1바퀴

        x = cx + R * math.cos(th)
        y = cy + R * math.sin(th)

        rad = 0.0 if i == total_steps else BLEND  # 마지막은 정확히 멈추게
        movel(posx(x, y, cz, rx0, ry0, rz0),
              vel=L_VEL, acc=L_ACC, ref=REF, radius=rad)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("m0609_stir_task_final", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        perform_task()
    except Exception as e:
        node.get_logger().error(f"Task failed: {e}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
