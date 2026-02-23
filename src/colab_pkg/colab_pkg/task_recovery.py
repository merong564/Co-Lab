#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
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

        from DSR_ROBOT2 import (
            set_tool, set_tcp,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            get_current_posx, movel, movej, posx, posj, DR_BASE
        )

        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.set_robot_mode = set_robot_mode
        self.ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS

        self.get_current_posx = get_current_posx
        self.movel = movel
        self.movej = movej
        self.posx = posx
        self.posj = posj
        self.DR_BASE = DR_BASE

        # 속도
        self.J_VEL, self.J_ACC = 30, 30
        self.L_VEL, self.L_ACC = 80, 80

        # 홈(너가 준 값)
        self.home_j = self.posj(0, 0, 90.0, 0, 90.0, 0)

        self.srv = self.create_service(
            RobotCommand,
            'execute_recovery',
            self.execute_recovery_callback,
            callback_group=self.callback_group
        )

        # 로봇 기본 세팅 (너희 Task들처럼)
        try:
            self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)
            self.set_tool(ROBOT_TOOL)
            self.set_tcp(ROBOT_TCP)
        except Exception as e:
            self.get_logger().warn(f"Robot init setting failed (continue): {e}")

        self.get_logger().info("TaskRecovery Ready. Service: /dsr01/execute_recovery")

    def _try_release_force_compliance(self):
        """환경마다 함수명이 달라서 '있으면 호출' 방식으로 안전하게 처리"""
        try:
            import DSR_ROBOT2 as dsr
            for fn_name in ["release_force", "release_compliance_ctrl", "release_task_compliance_ctrl"]:
                fn = getattr(dsr, fn_name, None)
                if callable(fn):
                    try:
                        fn()
                        self.get_logger().warn(f"[RECOVERY] called {fn_name}()")
                    except Exception:
                        pass
        except Exception:
            pass

    def execute_recovery_callback(self, request, response):
        try:
            self.get_logger().warn(f"[RECOVERY] mode={request.mode} targets={list(request.targets)}")

            # 1) force/compliance 해제 시도
            self._try_release_force_compliance()

            # 2) 현재 자세에서 Z-up
            cur, _ = self.get_current_posx(ref=self.DR_BASE)  # [x,y,z,rx,ry,rz]
            x, y, z, rx, ry, rz = cur
            z_up = z + 120.0

            self.movel(
                self.posx(x, y, z_up, rx, ry, rz),
                vel=self.L_VEL, acc=self.L_ACC, ref=self.DR_BASE
            )
            time.sleep(0.1)

            # 3) 홈 이동
            self.movej(self.home_j, vel=self.J_VEL, acc=self.J_ACC)

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

##################################################################
#########shock 토픽 강제 발행 명령어#########################3#################################
######ros2 topic pub /dsr01/safety/shock std_msgs/msg/String "{data: 'IMPACT_TEST'}" -1
#################################################################################################
########recovery 서비스 강제 호출 명령어################################################################################333
#ros2 service call /dsr01/execute_recovery colab_interfaces/srv/RobotCommand \
#"{mode: 'RECOVERY', targets: ['reason=TEST'], target_weights: [], mixing_duration: 0.0}"
####################################################################################3
