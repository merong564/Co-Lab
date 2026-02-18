#!/usr/bin/env python3
import time
import rclpy
import DR_init

# ===============================
# 개발용 분기 (추후 RobotCommand.srv Request로 교체)
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


class TaskTransfer:
    def __init__(self, node):
        # ===============================
        # 0) 로봇 기본 설정
        # ===============================
        self.ROBOT_ID = "dsr01"
        self.ROBOT_MODEL = "m0609"
        DR_init.__dsr__id = self.ROBOT_ID
        DR_init.__dsr__model = self.ROBOT_MODEL
        DR_init.__dsr__node = node  # ✅ create_node로 만든 node

        # ✅ node 세팅 이후 import (중요)
        from DSR_ROBOT2 import (
            movel, posx,
            movej, posj,
            wait,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            set_digital_output,
            DR_BASE
        )

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        # ===============================
        # 1) 바인딩
        # ===============================
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
        # 2) 모드/대상
        # ===============================
        self.mode = (MODE or "").strip().upper()
        self.tube_type = (TUBE_TYPE or "").strip().upper()

        if self.mode not in ("PICKUP", "RETURN"):
            raise ValueError(f"MODE must be PICKUP or RETURN, got: {MODE}")
        if self.tube_type not in ("SMALL", "LARGE"):
            raise ValueError(f"TUBE_TYPE must be SMALL or LARGE, got: {TUBE_TYPE}")

        # ===============================
        # 3) HOME(ready) - 조인트 좌표계 (HOME만 movej)
        # ===============================
        self.ready_j = self.posj(0, 0, 90.0, 0, 90.0, 0)

        # ===============================
        # 4) 작업 좌표(posx) - SMALL/LARGE
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
    def goL(self, p):  # movel only (작업 이동 전부 직선)
        self.movel(p, vel=self.L_VEL, acc=self.L_ACC, ref=self.REF)

    def goJ_home(self):  # HOME만 movej
        self.movej(self.ready_j, vel=self.J_VEL, acc=self.J_ACC)

    # -------------------------------
    # PICKUP: 집고 비커 앞 위치로 이동
    # -------------------------------
    def pickup_flow(self):
        P = self.POSES[self.tube_type]

        # 시작을 HOME에서 하고 싶으면 사용 (HOME만 movej 허용)
        self.goJ_home()

        self.gripper_open()

        self.goL(P["PICK_UP"])
        self.goL(P["PICK_DOWN"])

        self.gripper_close()

        self.goL(P["PICK_UP"])
        self.goL(P["POUR_READY"])

        print(f"✔ PICKUP done (tube_type={self.tube_type})")

    # -------------------------------
    # RETURN: 원위치 내려놓기 -> 위로 빠짐 -> HOME 복귀(movej)
    # -------------------------------
    def return_flow(self):
        P = self.POSES[self.tube_type]

        # (현재 pour_ready 근처에서 시작한다고 가정)
        self.goL(P["PICK_UP"])
        self.goL(P["PICK_DOWN"])

        self.gripper_open()
        self.wait(2.0)

        self.goL(P["PICK_UP"])

        # HOME만 movej
        self.goJ_home()

        print(f"✔ RETURN done (tube_type={self.tube_type})")

    def execute(self):
        if self.mode == "PICKUP":
            self.pickup_flow()
        else:
            self.return_flow()


def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = "dsr01"
    node = rclpy.create_node("task_transfer", namespace=ROBOT_ID)

    try:
        task = TaskTransfer(node)
        task.execute()
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
