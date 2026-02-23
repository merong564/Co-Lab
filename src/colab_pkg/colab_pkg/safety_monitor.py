#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
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
    - /dsr01/safety/shock (std_msgs/String) 발행
    - 예: "IMPACT_DF:23.40 MAG:51.20"
    """
    def __init__(self):
        super().__init__('safety_monitor', namespace=ROBOT_ID)

        from DSR_ROBOT2 import get_tool_force, DR_TOOL
        self.get_tool_force = get_tool_force
        self.DR_TOOL = DR_TOOL

        self.pub_shock = self.create_publisher(String, 'safety/shock', 10)

        # ===== 튜닝 =====
        self.HZ = 20.0
        self.DF_THRESHOLD = 25.0    # ΔF 임계값 (현장 튜닝)
        self.COOLDOWN_SEC = 1.0     # 연속 발행 방지
        self.WARMUP_FRAMES = 8      # 시작 안정화 프레임

        self.prev_mag = None
        self.last_fire_ts = 0.0
        self.warmup_cnt = 0

        self.timer = self.create_timer(1.0 / self.HZ, self.loop)
        self.get_logger().info("SafetyMonitor Ready. Topic: /dsr01/safety/shock")

    def loop(self):
        try:
            f = self.get_tool_force(self.DR_TOOL)  # 보통 [Fx,Fy,Fz,Mx,My,Mz]
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

        now = time.time()
        if df >= self.DF_THRESHOLD and (now - self.last_fire_ts) >= self.COOLDOWN_SEC:
            self.last_fire_ts = now
            msg = String()
            msg.data = f"IMPACT_DF:{df:.2f} MAG:{mag:.2f}"
            self.pub_shock.publish(msg)
            self.get_logger().error(f"[SHOCK] {msg.data}")


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