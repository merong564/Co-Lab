#!/usr/bin/env python3
import time
import rclpy
import DR_init
from rclpy.node import Node

from colab_interfaces.srv import RobotCommand  # 너희 srv 사용


# ===============================
# movel 속도/가속
# ===============================
L_VEL, L_ACC = 150, 150

# 그리퍼 DO
DO_OPEN = 1
DO_CLOSE = 2


class TaskTransfer(Node):
    """
    Flowchart 기반 TaskTransfer (Service Server)

    - 로봇 연결(DSR_ROBOT2 바인딩)
    - 서비스 요청 대기: /dsr01/robot_command
    - request.mode에 따라 PICKUP / RETURN 수행

    좌표 매핑(네가 준 값):
      PICKUP:  Approach(PICK_UP) -> Insert(PICK_DOWN) -> Grip -> Lift(PICK_UP) -> Beaker(POUR_READY)
      RETURN:  RackTop(PICK_UP)  -> Place(PICK_DOWN)  -> Open -> Retract(PICK_UP)
    """

    def __init__(self):
        # ===============================
        # 0) ROS/DSR 기본 설정
        # ===============================
        self.ROBOT_ID = "dsr01"
        self.ROBOT_MODEL = "m0609"
        super().__init__("task_transfer", namespace=self.ROBOT_ID)

        DR_init.__dsr__id = self.ROBOT_ID
        DR_init.__dsr__model = self.ROBOT_MODEL
        DR_init.__dsr__node = self

        # ===============================
        # 1) DSR 바인딩
        # ===============================
        from DSR_ROBOT2 import (
            movel, posx, wait,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            set_digital_output,
            DR_BASE
        )
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        self.movel = movel
        self.posx = posx
        self.wait = wait
        self.set_digital_output = set_digital_output
        self.REF = DR_BASE

        self.L_VEL, self.L_ACC = L_VEL, L_ACC
        self._busy = False

        # ===============================
        # 2) 네가 준 좌표 그대로 (posx)
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

        # ===============================
        # 3) 서비스 서버 (서비스 요청 받는가?)
        # ===============================
        self.srv = self.create_service(RobotCommand, "robot_command", self._on_command)
        self.get_logger().info("TaskTransfer ready. Service: /dsr01/robot_command")

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
    # motion (movel only)
    # -------------------------------
    def goL(self, p):
        self.movel(p, vel=self.L_VEL, acc=self.L_ACC, ref=self.REF)

    # -------------------------------
    # Flowchart: PICKUP
    # -------------------------------
    def pickup_flow(self, tube_type: str):
        P = self.POSES[tube_type]
        approach = P["PICK_UP"]     # 시험관 위치 접근(Approach)
        insert   = P["PICK_DOWN"]   # 파지 위치 이동(Insert)
        lift     = P["PICK_UP"]     # 시험관 뽑기(Lift) = 다시 위로
        beaker   = P["POUR_READY"]  # 비커 위치로 이동

        self.gripper_open()

        self.goL(approach)
        self.goL(insert)

        self.gripper_close()

        self.goL(lift)
        self.goL(beaker)

    # -------------------------------
    # Flowchart: RETURN
    # -------------------------------
    def return_flow(self, tube_type: str):
        P = self.POSES[tube_type]
        rack_top = P["PICK_UP"]     # 랙 상단 위치로 이동 (좌표 없어서 PICK_UP으로 대체)
        place    = P["PICK_DOWN"]   # 시험관 꽂기(Place)
        retract  = P["PICK_UP"]     # 랙 상단 위치로 이동(Retract)

        self.goL(rack_top)
        self.goL(place)

        self.gripper_open()
        self.wait(0.5)

        self.goL(retract)

    # -------------------------------
    # service callback
    # -------------------------------
    def _on_command(self, request, response):
        """
        RobotCommand.srv 가정:
          string mode      # "PICKUP" or "RETURN"
          string tube_type # "SMALL" or "LARGE"
          ---
          bool success
          string message
        """
        if self._busy:
            response.success = False
            response.message = "BUSY"
            return response

        self._busy = True
        try:
            mode = (getattr(request, "mode", "") or "").strip().upper()
            tube_type = (getattr(request, "tube_type", "") or "").strip().upper()

            if tube_type not in ("SMALL", "LARGE"):
                response.success = False
                response.message = f"INVALID_TUBE_TYPE: {tube_type}"
                return response

            if mode == "PICKUP":
                self.get_logger().info(f"Command PICKUP ({tube_type})")
                self.pickup_flow(tube_type)
                response.success = True
                response.message = "PICKUP_DONE"

            elif mode == "RETURN":
                self.get_logger().info(f"Command RETURN ({tube_type})")
                self.return_flow(tube_type)
                response.success = True
                response.message = "RETURN_DONE"

            else:
                response.success = False
                response.message = f"INVALID_MODE: {mode}"

        except Exception as e:
            response.success = False
            response.message = f"ERROR: {e}"
        finally:
            self._busy = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskTransfer()
    try:
        rclpy.spin(node)  # 서비스 요청 대기
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
