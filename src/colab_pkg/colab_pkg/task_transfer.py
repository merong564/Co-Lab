#!/usr/bin/env python3
"""
task_transfer.py

- 클래스명: TaskTransfer
- 노드명: task_transfer
- MODE로 PICKUP / RETURN 분기 (string)
- 이동:
  - 시험관/비커 관련 이동: movel(posx)  (직선)
  - HOME(ready) 복귀만: movej(posj)      (요구대로 예외)
- mix 코드는 별도 파일에서 실행
"""

import time
import rclpy
import DR_init
from rclpy.node import Node

# ===============================
# 개발용 설정 (추후 srv Request로 교체)
# ===============================
MODE = "PICKUP"        # "PICKUP" or "RETURN"
TUBE_TYPE = "SMALL"    # "SMALL" or "LARGE"

# movel 속도/가속
L_VEL, L_ACC = 150, 150

# movej 속도/가속 (HOME 복귀용)
J_VEL, J_ACC = 30, 30

# 그리퍼 DO
DO_OPEN = 1
DO_CLOSE = 2


class TaskTransfer(Node):
    def __init__(self):
        # ===============================
        # 0) 로봇 기본 설정
        # ===============================
        self.ROBOT_ID = "dsr01"
        self.ROBOT_MODEL = "m0609"
        DR_init.__dsr__id = self.ROBOT_ID
        DR_init.__dsr__model = self.ROBOT_MODEL

        super().__init__("task_transfer", namespace=self.ROBOT_ID)
        DR_init.__dsr__node = self

        from DSR_ROBOT2 import (
            movel, posx,
            movej, posj,
            wait,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            set_digital_output,
            DR_BASE
        )

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        self.movel = movel
        self.posx = posx
        self.movej = movej
        self.posj = posj
        self.wait = wait
        self.set_digital_output = set_digital_output
        self.REF = DR_BASE

        self.L_VEL, self.L_ACC = L_VEL, L_ACC
        self.J_VEL, self.J_ACC = J_VEL, J_ACC

        # ===============================
        # 1) 모드/대상
        # ===============================
        self.mode = (MODE or "").strip().upper()
        self.tube_type = (TUBE_TYPE or "").strip().upper()

        if self.mode not in ("PICKUP", "RETURN"):
            raise ValueError(f"MODE must be PICKUP or RETURN, got: {MODE}")
        if self.tube_type not in ("SMALL", "LARGE"):
            raise ValueError(f"TUBE_TYPE must be SMALL or LARGE, got: {TUBE_TYPE}")

        # ===============================
        # 2) HOME(ready) - 조인트 좌표계
        # ===============================
        self.ready_j = self.posj(0, 0, 90.0, 0, 90.0, 0)

        # ===============================
        # 3) task 좌표(posx) - 너가 준 값 그대로
        # ===============================
        self.POSES = {
            "SMALL": {
                "PICK_DOWN": self.posx(555.786, -78.524, 126.047, 90.674, 92.519, 93.656),
                "PICK_UP":   self.posx(555.784, -78.523, 259.725, 90.674, 92.518, 93.657),
                "POUR_READY": self.posx(604.441, 157.760, 242.631, 91.920, 97.360, 88.550),
            },
            "LARGE": {
                "PICK_DOWN": self.posx(306.636, -66.725,  89.141, 91.356, 91.786, 90.102),
                "PICK_UP":   self.posx(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
                "POUR_READY": self.posx(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
            }
        }

    # -------------------------------
    # gripper
    # -------------------------------
    def gripper_open(self):
        self.set_digital_output(DO_CLOSE, 0)
        self.set_digital_output(DO_OPEN, 1)
        time.sleep(0.3)

    def gripper_close(self):
        self.set_digital_output(DO_OPEN, 0)
        self.set_digital_output(DO_CLOSE, 1)
        time.sleep(0.4)

    # -------------------------------
    # motion
    # -------------------------------
    def goL(self, p):  # movel only
        self.movel(p, vel=self.L_VEL, acc=self.L_ACC, ref=self.REF)

    def goJ_home(self):  # home만 movej
        self.movej(self.ready_j, vel=self.J_VEL, acc=self.J_ACC)

    # -------------------------------
    # PICKUP: 집고 비커 앞 대기(여기서 끝, mix는 별도)
    # -------------------------------
    def pickup_flow(self):
        P = self.POSES[self.tube_type]
        pick_up = P["PICK_UP"]
        pick_down = P["PICK_DOWN"]
        pour_ready = P["POUR_READY"]

        # (선택) 시작을 home에서 하고 싶으면 아래 줄 주석 해제
        self.goJ_home()

        self.gripper_open()

        self.goL(pick_up)
        self.goL(pick_down)

        self.gripper_close()

        self.goL(pick_up)
        self.goL(pour_ready)

        print(f"✔ PICKUP done (tube_type={self.tube_type}). 이제 mix 코드는 별도로 실행.")

    # -------------------------------
    # RETURN: 원위치 내려놓기 -> 위로 빠짐 -> home은 movej로 복귀
    # -------------------------------
    def return_flow(self):
        P = self.POSES[self.tube_type]
        pick_up = P["PICK_UP"]
        pick_down = P["PICK_DOWN"]

        # (현재 pour_ready 근처에서 시작한다고 가정)
        self.goL(pick_up)
        self.goL(pick_down)

        self.gripper_open()
        self.wait(2.0)

        self.goL(pick_up)

        # home 복귀는 조인트로
        self.goJ_home()

        print(f"✔ RETURN done (tube_type={self.tube_type}). home(movej) 복귀 완료.")

    def execute(self):
        if self.mode == "PICKUP":
            self.pickup_flow()
        else:
            self.return_flow()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TaskTransfer()
        node.execute()
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
