import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String  # [추가] String 메시지 타입
import sys

from colab_interfaces.srv import RobotCommand

ROBOT_ID = "dsr01"

class SystemController(Node):
    def __init__(self):
        super().__init__('SystemController', namespace=ROBOT_ID)
        
        self.callback_group = ReentrantCallbackGroup()

        # [추가] 중단 요청 플래그
        self.is_stop_requested = False

        # [추가] Stop 토픽 구독 (Reentrant 그룹 사용 필수 - 작업 중에도 수신해야 함)
        self.sub_stop = self.create_subscription(
            String,
            'stop',
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
        self.get_logger().info('All Service Servers Connected!') # [수정] 이모티콘 제거

    def stop_callback(self, msg):
        """[추가] /stop 토픽 수신 시 호출되는 콜백"""
        if msg.data == 'STOP':
            self.get_logger().warn("EMERGENCY STOP REQUEST RECEIVED! Aborting process...") # [수정] 이모티콘 제거
            self.is_stop_requested = True

    async def handle_start_process(self, request, response):
        self.get_logger().info("="*40)
        self.get_logger().info("[Process Start]") # [수정] 단일 타겟 출력에서 변경
        
        # [추가] 작업 시작 전 플래그 초기화
        self.is_stop_requested = False
        
        try:
            # [추가] 요청받은 여러 재료 리스트를 순회하며 작업 수행
            for target, weight in zip(request.targets, request.target_weights):
                self.get_logger().info(f"[Task] Target: {target}, Target Weight: {weight}g")
                
                # 1. ScaleDriver: Tare
                if self.check_stop(): raise Exception("Process Aborted by User")
                if not await self.call_service(self.cli_scale, mode="TARE"):
                    raise Exception("Scale Tare Failed")

                # 2. TaskTransfer: Pickup
                if self.check_stop(): raise Exception("Process Aborted by User")
                # [추가] 현재 타겟 전달
                if not await self.call_service(self.cli_transfer, mode="PICKUP", targets=[target]):
                    raise Exception(f"Transfer Pickup Failed for {target}")

                # 3. TaskPouring: Pouring
                if self.check_stop(): raise Exception("Process Aborted by User")
                # [추가] 현재 무게 전달
                if not await self.call_service(self.cli_pouring, mode="POUR", targets=[target],target_weights=[weight]):
                    raise Exception(f"Pouring Failed for {target}")

                # 4. TaskTransfer: Return
                if self.check_stop(): raise Exception("Process Aborted by User")
                # [추가] 현재 타겟 전달
                if not await self.call_service(self.cli_transfer, mode="RETURN", targets=[target]):
                    raise Exception(f"Transfer Return Failed for {target}")
            
            # 5. TaskMixing: Mixing (모든 재료 투입 후 1회 실행)
            if self.check_stop(): raise Exception("Process Aborted by User")
            if not await self.call_service(self.cli_mixing, mode="MIX", mixing_duration=request.mixing_duration):
                raise Exception("Mixing Failed")

            response.success = True
            response.message = "All tasks completed successfully."
            self.get_logger().info("[Process Complete] All tasks finished.") # [수정] 이모티콘 제거

        except Exception as e:
            # [수정] 중단 또는 에러 발생 시 처리
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"[Process Failed/Aborted] {e}") # [수정] 이모티콘 제거

        return response

    def check_stop(self):
        """[추가] 중단 요청이 들어왔는지 확인하는 헬퍼 함수"""
        if self.is_stop_requested:
            self.get_logger().warn("Stopping current operation sequence.") # [수정] 이모티콘 제거
            return True
        return False

    # [수정] 기존 target_weight 단일 변수 대신 targets, target_weights 리스트 파라미터로 변경
    async def call_service(self, client, mode="", targets=None, target_weights=None, mixing_duration=0.0):
        # [추가] 서비스 호출 직전에도 STOP 확인
        if self.is_stop_requested:
            return False

        req = RobotCommand.Request()
        req.mode = mode
        
        # [추가] 리스트 데이터 할당
        if targets is not None:
            req.targets = targets
        if target_weights is not None:
            req.target_weights = target_weights
            
        req.mixing_duration = float(mixing_duration)

        self.get_logger().info(f" -> Requesting {client.srv_name} | Mode: {mode}")
        
        future = client.call_async(req)
        
        # [중요] ReentrantCallbackGroup 덕분에 await 중에도 stop_callback이 실행되어 self.is_stop_requested가 True로 바뀔 수 있음
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
        print(" [System Controller] Ready... Send 'STOP' to /stop to abort.")
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()