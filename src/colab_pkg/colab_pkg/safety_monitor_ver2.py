#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from colab_interfaces.msg import SystemStatus # [추가]

import DR_init

# ===============================
# 1. 설정 및 상수 (너희 스타일)
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class SafetyMonitor(Node):
    """
    - /dsr01/stop/impact (std_msgs/String) 발행
    """
    def __init__(self):
        super().__init__('safety_monitor', namespace=ROBOT_ID)

        from DSR_ROBOT2 import get_tool_force, DR_TOOL
        self.get_tool_force = get_tool_force
        self.DR_TOOL = DR_TOOL

        # ✅ 토픽명 변경
        self.pub_impact = self.create_publisher(String, 'stop/impact', 10)

        # ===== 튜닝(꼭 필요한 것만 + 주석) =====
        self.HZ = 20.0
        
        # [수정] 단일 고정 임계값 삭제 후 평시/접촉시 임계값 분리 및 동적 변수 선언
        self.DF_THRESH_NORMAL = 10.0 
        self.DF_THRESH_HIGH = 30.0 
        self.current_thresh = self.DF_THRESH_NORMAL # [추가]

        # 연속 발행 방지(초)
        self.COOLDOWN_SEC = 2.0
        # 시작 직후 흔들림 무시(프레임)
        self.WARMUP_FRAMES = 8

        self.prev_mag = None
        self.last_fire_ts = 0.0
        self.warmup_cnt = 0

        # [추가] 공정 상태 구독자 설정
        self.sub_status = self.create_subscription(
            SystemStatus, 
            'system_status', 
            self.status_callback, 
            10
        )

        # ✅ loop -> perform_task
        self.timer = self.create_timer(1.0 / self.HZ, self.perform_task)
        self.get_logger().info("SafetyMonitor Ready. Topic: /dsr01/stop/impact")

    # [추가] 공정 상태에 따른 임계값 동적 변경 콜백
    def status_callback(self, msg: SystemStatus):
        if msg.phase in ["Mixing"]:
        # if msg.phase in ["Transfer", "Mixing", "Return"]:

            self.current_thresh = self.DF_THRESH_HIGH
        else:
            self.current_thresh = self.DF_THRESH_NORMAL

    def perform_task(self):
        try:
            f = self.get_tool_force(self.DR_TOOL)  # [Fx,Fy,Fz,Mx,My,Mz]
            fx, fy, fz = float(f[0]), float(f[1]), float(f[2])
        except Exception as e:
            self.get_logger().warn(f"get_tool_force failed: {e}")
            return

        mag = (fx*fx + fy*fy + fz*fz) ** 0.5

        if self.prev_mag is None:
            self.prev_mag = mag
            return

        if self.warmup_cnt < self.WARMUP_FRAMES:
            self.warmup_cnt += 1
            self.prev_mag = mag
            return

        df = abs(mag - self.prev_mag)
        self.prev_mag = mag

        # [수정] 현재 적용 중인 임계값을 출력하도록 로그 변경
        self.get_logger().info(f"실시간 감지 힘 - Mag: {mag:.2f}, DF: {df:.2f} (Thresh: {self.current_thresh})")

        now = time.time()
        
        # [수정] self.DF_THRESHOLD 대신 self.current_thresh 사용
        if df >= self.current_thresh and (now - self.last_fire_ts) >= self.COOLDOWN_SEC:
            self.last_fire_ts = now
            msg = String()
            msg.data = 'STOP'
            self.pub_impact.publish(msg)
            self.get_logger().error(f"[IMPACT] {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    # 너희 패턴 그대로: DSR 호출 전용 hidden node
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node

    monitor_node = SafetyMonitor()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(monitor_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        monitor_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()