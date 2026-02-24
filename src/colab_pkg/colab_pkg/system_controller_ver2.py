import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from colab_interfaces.srv import RobotCommand

ROBOT_ID = "dsr01"

class SystemController(Node):
    def __init__(self):
        super().__init__('SystemController', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # 중단 요청 플래그
        self.is_stop_requested = False

        # ✅ [추가] stop 토픽 퍼블리셔 (/dsr01/stop)
        self.pub_stop = self.create_publisher(
            String,
            'stop',      # namespace=dsr01 이므로 /dsr01/stop
            10
        )

        # Stop 토픽 구독 (/dsr01/stop/impact)
        self.sub_stop = self.create_subscription(
            String,
            'stop/impact',
            self.stop_callback,
            10,
            callback_group=self.callback_group
        )

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

        # self.check_services_availability()

    def stop_callback(self, msg: String):
        """ /dsr01/stop/impact 에서 'STOP' 오면 /dsr01/stop 으로 'STOP' 재발행 """
        data = (msg.data or "").strip().upper()
        if data != "STOP":
            return

        self.get_logger().warn("EMERGENCY STOP REQUEST RECEIVED! Aborting process...")
        self.is_stop_requested = True

        # ✅ [추가] stop 토픽으로 STOP 발행
        out = String()
        out.data = "STOP"
        self.pub_stop.publish(out)
        self.get_logger().warn("Published 'STOP' to /dsr01/stop")

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

    async def handle_start_process(self, request, response):
        self.get_logger().info("=" * 40)
        self.get_logger().info("[Process Start]")

        self.is_stop_requested = False

        try:
            for target, weight in zip(request.targets, request.target_weights):
                self.get_logger().info(f"[Task] Target: {target}, Target Weight: {weight}g")

                # if self.check_stop(): raise Exception("Process Aborted by User")
                # if not await self.call_service(self.cli_scale, mode="TARE"):
                #     raise Exception("Scale Tare Failed")

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

    def check_stop(self):
        if self.is_stop_requested:
            self.get_logger().warn("Stopping current operation sequence.")
            return True
        return False

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
            # [추가] 모든 서비스에 대해 False 반환 시 서비스명 및 모드를 명시하여 로그 출력
            self.get_logger().error(f"    [Service Error] {client.srv_name} returned False for mode: {mode}")
            
            self.get_logger().error(f"    Failed: {result.message}")
            return False

def main(args=None):
    rclpy.init(args=args)
    controller = SystemController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        print(" [System Controller] Ready... Send 'STOP' to /dsr01/stop/impact to abort.")
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()