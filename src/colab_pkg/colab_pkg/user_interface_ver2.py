import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from colab_interfaces.srv import RobotCommand

ROBOT_ID = "dsr01"

class SystemController(Node):
    def __init__(self):
        super().__init__('system_controller', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # STOP 플래그
        self.is_stop_requested = False

        # ✅ 공용 stop 발행(모든 task 노드가 이 토픽을 구독하면 됨)
        self.pub_stop = self.create_publisher(
            String,
            'stop',
            10
        )

        # ✅ UI STOP 구독
        self.sub_stop_ui = self.create_subscription(
            String,
            'stop/ui',
            self.stop_ui_callback,
            10,
            callback_group=self.callback_group
        )

        # ✅ Impact STOP 구독
        self.sub_stop_impact = self.create_subscription(
            String,
            'stop/impact',
            self.stop_impact_callback,
            10,
            callback_group=self.callback_group
        )

        # (선택) 기존 /stop 직접 발행도 수신하고 싶으면 유지
        # self.sub_stop = self.create_subscription(
        #     String,
        #     'stop',
        #     self.stop_callback,
        #     10,
        #     callback_group=self.callback_group
        # )

        # Service Server
        self.srv_start = self.create_service(
            RobotCommand,
            'start_process',
            self.handle_start_process,
            callback_group=self.callback_group
        )

        # Service Clients
        self.cli_scale = self.create_client(RobotCommand, 'set_tare', callback_group=self.callback_group)
        self.cli_transfer = self.create_client(RobotCommand, 'execute_transfer', callback_group=self.callback_group)
        self.cli_pouring = self.create_client(RobotCommand, 'execute_pouring', callback_group=self.callback_group)
        self.cli_mixing = self.create_client(RobotCommand, 'execute_mixing', callback_group=self.callback_group)

        self.check_services_availability()

    def check_services_availability(self):
        clients = [
            ('ScaleDriver', self.cli_scale),
            ('TaskTransfer', self.cli_transfer),
            ('TaskPouring', self.cli_pouring),
            ('TaskMixing', self.cli_mixing)
        ]
        for name, client in clients:
            self.get_logger().info(f'Waiting for {name} server...')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting again...')
        self.get_logger().info('All Service Servers Connected!')

    # =========================
    # STOP 입력(각 채널별)
    # =========================
    def stop_ui_callback(self, msg: String):
        if msg.data == "STOP":
            self.request_stop(source="UI")

    def stop_impact_callback(self, msg: String):
        if msg.data == "STOP":
            self.request_stop(source="IMPACT")

    # (선택) /stop 직접 발행 감지용
    # def stop_callback(self, msg: String):
    #     if msg.data == "STOP":
    #         self.request_stop(source="DIRECT_STOP_TOPIC")

    def request_stop(self, source: str):
        """
        ✅ UI/IMPACT에서 STOP 들어오면:
        1) 컨트롤러 내부 플래그 ON
        2) 공용 /stop 으로 STOP 브로드캐스트 (Task 노드들 즉시 멈추게)
        """
        if self.is_stop_requested:
            return  # 중복 발행 방지

        self.is_stop_requested = True
        self.get_logger().warn(f"EMERGENCY STOP REQUEST RECEIVED from {source}! Broadcasting /stop ...")

        self.pub_stop.publish(String(data="STOP"))

    def check_stop(self):
        if self.is_stop_requested:
            self.get_logger().warn("Stopping current operation sequence.")
            return True
        return False

    async def handle_start_process(self, request, response):
        self.get_logger().info("=" * 40)
        self.get_logger().info("[Process Start]")

        # 시작 시 초기화
        self.is_stop_requested = False

        try:
            for target, weight in zip(request.targets, request.target_weights):
                self.get_logger().info(f"[Task] Target: {target}, Target Weight: {weight}g")

                if self.check_stop(): raise Exception("Process Aborted by User")
                if not await self.call_service(self.cli_scale, mode="TARE"):
                    raise Exception("Scale Tare Failed")

                if self.check_stop(): raise Exception("Process Aborted by User")
                if not await self.call_service(self.cli_transfer, mode="PICKUP", targets=[target]):
                    raise Exception(f"Transfer Pickup Failed for {target}")

                if self.check_stop(): raise Exception("Process Aborted by User")
                if not await self.call_service(self.cli_pouring, mode="POUR", targets=[target], target_weights=[weight]):
                    raise Exception(f"Pouring Failed for {target}")

                if self.check_stop(): raise Exception("Process Aborted by User")
                if not await self.call_service(self.cli_transfer, mode="RETURN", targets=[target]):
                    raise Exception(f"Transfer Return Failed for {target}")

            if self.check_stop(): raise Exception("Process Aborted by User")
            if not await self.call_service(self.cli_mixing, mode="MIX", mixing_duration=request.mixing_duration):
                raise Exception("Mixing Failed")

            if self.check_stop(): raise Exception("Process Aborted by User")
            if not await self.call_service(self.cli_transfer, mode="RETURN", targets=["BEAKER"]):
                raise Exception("Final Return Failed for BEAKER")

            response.success = True
            response.message = "All tasks completed successfully."
            self.get_logger().info("[Process Complete] All tasks finished.")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"[Process Failed/Aborted] {e}")

        return response

    async def call_service(self, client, mode="", targets=None, target_weights=None, mixing_duration=0.0):
        if self.is_stop_requested:
            return False

        req = RobotCommand.Request()
        req.mode = mode

        if targets is not None:
            req.targets = targets
        if target_weights is not None:
            req.target_weights = target_weights

        req.mixing_duration = float(mixing_duration)

        self.get_logger().info(f" -> Requesting {client.srv_name} | Mode: {mode}")

        future = client.call_async(req)
        result = await future

        if result.success:
            self.get_logger().info(f"    Success: {result.message}")
            return True
        else:
            self.get_logger().error(f"    Failed: {result.message}")
            return False


def main(args=None):
    rclpy.init(args=args)
    controller = SystemController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        print("[System Controller] Ready...")
        print(" - UI STOP  : publish 'STOP' to /dsr01/stop/ui")
        print(" - Impact   : publish 'STOP' to /dsr01/stop/impact")
        print(" - Then controller broadcasts /dsr01/stop")
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()