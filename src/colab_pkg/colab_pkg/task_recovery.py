#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import DR_init

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from colab_interfaces.srv import RobotCommand

# ===============================
# 1. 설정 및 상수 (너희 스타일)
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class TaskRecovery(Node):
    def __init__(self):
        super().__init__('task_recovery', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # STOP 처리
        self.stop_requested = False
        self.sub_stop = self.create_subscription(String, 'stop', self._on_stop, 10)

        # 속도
        self.J_VEL, self.J_ACC = 30, 30
        self.L_VEL, self.L_ACC = 80, 80

        # 4번 테스트: Z 올리고 홈 복귀
        self.Z_UP_MM = 120.0

        # 홈(너가 준 값)
        from DSR_ROBOT2 import posj
        self.home_j = posj(0, 0, 90.0, 0, 90.0, 0)

        self.srv = self.create_service(
            RobotCommand,
            'execute_recovery',
            self.execute_recovery_callback,
            callback_group=self.callback_group
        )

        # 로봇 기본 세팅
        try:
            from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
        except Exception as e:
            self.get_logger().warn(f"Robot init setting failed (continue): {e}")

        self.get_logger().info("TaskRecovery Ready. Service: /dsr01/execute_recovery")

    def _on_stop(self, msg: String):
        self.stop_requested = True
        self.get_logger().warn(f"[STOP] received: {msg.data}")

    def _check_stop(self):
        if self.stop_requested:
            raise RuntimeError("STOP topic received")

    # ===============================
    # 3) perform_task 안에 로봇 동작 모으기
    # ===============================
    def perform_task(self, request):
        from DSR_ROBOT2 import (
            release_force, release_compliance_ctrl, wait,
            get_current_posx, movel, movej, posx, posj, DR_BASE
        )

        # ---- (요구사항 5) 물체별 return 함수들: perform_task 내부에 두기 ----
        def return_large():
            # TODO: 아래 좌표는 너희 LARGE 원위치 좌표로 교체
            # 예: 랙 접근/원위치/그리퍼 오픈/후퇴 순서
            self.get_logger().warn("[RETURN] LARGE -> TODO positions")
            # movej(posj(...), vel=self.J_VEL, acc=self.J_ACC)
            # wait(0.2)

        def return_small1():
            self.get_logger().warn("[RETURN] SMALL1 -> TODO positions")
            # movej(posj(...), vel=self.J_VEL, acc=self.J_ACC)
            # wait(0.2)

        def return_beaker():
            self.get_logger().warn("[RETURN] BEAKER -> TODO positions")
            # movej(posj(...), vel=self.J_VEL, acc=self.J_ACC)
            # wait(0.2)

        # ---- 1) 힘/컴플라이언스 해제 (요구사항 1) ----
        # “두산 제공 함수만 사용” 조건은 지키면서, 실패해도 복구가 진행되도록 예외는 무시(현장 안전)
        try:
            release_force()
        except Exception:
            pass
        try:
            release_compliance_ctrl()
        except Exception:
            pass
        wait(0.2)

        self._check_stop()

        # ---- 2) 현재 자세에서 Z-up (요구사항 4) ----
        cur, _ = get_current_posx(ref=DR_BASE)  # [x,y,z,rx,ry,rz]
        x, y, z, rx, ry, rz = cur
        z_up = z + self.Z_UP_MM

        self.get_logger().warn(f"[RECOVERY] z={z:.2f} -> z_up={z_up:.2f}")

        movel(posx(x, y, z_up, rx, ry, rz), vel=self.L_VEL, acc=self.L_ACC, ref=DR_BASE)
        wait(0.2)

        self._check_stop()

        # ---- 3) 홈 이동 ----
        movej(self.home_j, vel=self.J_VEL, acc=self.J_ACC)
        wait(0.2)

        self._check_stop()

        # ---- 5) 들고 있는 물체 확인 후 원위치 (요구사항 5) ----
        # 판별 기준: request.targets[0] (컨트롤러가 넣어줘야 함)
        target = None
        if getattr(request, "targets", None) and len(request.targets) > 0:
            target = str(request.targets[0]).strip().upper()

        if not target:
            self.get_logger().warn("[RECOVERY] No target in request.targets -> skip return_xxx()")
            return

        if target == "LARGE":
            return_large()
        elif target == "SMALL1":
            return_small1()
        elif target == "BEAKER":
            return_beaker()
        else:
            self.get_logger().warn(f"[RECOVERY] Unknown target='{target}' -> skip return_xxx()")

    def execute_recovery_callback(self, request, response):
        # recovery 시작 시 stop 플래그 초기화(복구 목적)
        self.stop_requested = False

        try:
            self.get_logger().warn(
                f"[RECOVERY] mode={request.mode} targets={list(request.targets)}"
            )

            self.perform_task(request)

            response.success = True
            response.message = "RECOVERY_DONE"
            return response

        except Exception as e:
            self.get_logger().error(f"[RECOVERY] failed: {e}")
            response.success = False
            response.message = str(e)
            return response


def main(args=None):
    rclpy.init(args=args)

    # 너희 패턴 그대로: DSR 호출 전용 hidden node
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node

    task_node = TaskRecovery()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    print("\n=== Service Server Started (Multi-Node) ===")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()