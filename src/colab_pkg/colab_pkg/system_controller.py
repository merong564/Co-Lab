import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup # [추가] 동시성 처리를 위한 그룹
from rclpy.executors import MultiThreadedExecutor
import sys

# 서비스 타입 임포트
from colab_interfaces.srv import RobotCommand

class SystemController(Node):
    def __init__(self):
        super().__init__('SystemController')
        
        # [추가] ReentrantCallbackGroup 사용: 서비스 콜백 내에서 다른 서비스 클라이언트를 호출하기 위함
        self.callback_group = ReentrantCallbackGroup()

        # [추가] Service Server 생성: UserInterface로부터 명령 수신
        self.srv_start = self.create_service(
            RobotCommand, 
            '/start_process', 
            self.handle_start_process, 
            callback_group=self.callback_group
        )

        # [추가] Service Client 생성: 각 노드별 서비스 연결
        self.cli_scale = self.create_client(RobotCommand, '/set_tare', callback_group=self.callback_group)
        self.cli_transfer = self.create_client(RobotCommand, '/execute_transfer', callback_group=self.callback_group)
        self.cli_pouring = self.create_client(RobotCommand, '/execute_pouring', callback_group=self.callback_group) # 기존 이름 유지
        self.cli_mixing = self.create_client(RobotCommand, '/execute_mixing', callback_group=self.callback_group)
        
        # [추가] 모든 서비스 서버 연결 대기
        self.check_services_availability()

    def check_services_availability(self):
        """[추가] 연결된 모든 노드의 서비스 가용성 확인"""
        clients = [
            ('ScaleDriver', self.cli_scale),
            ('TaskTransfer', self.cli_transfer),
            ('TaskPouring', self.cli_pouring),
            ('TaskMixing', self.cli_mixing)
        ]
        
        for name, client in clients:
            self.get_logger().info(f'Waiting for {name} server...')
            while not client.wait_for_service(timeout_sec=1.0):         # 각 서비스가 연결될 때까지 1초 간격으로 대기
                self.get_logger().info(f'{name} service not available, waiting again...')
        
        self.get_logger().info('✅ All Service Servers Connected!')

    async def handle_start_process(self, request, response):
        """[추가] 전체 공정 순차 실행 로직 (UserInterface 요청 처리)"""
        self.get_logger().info("="*40)
        self.get_logger().info(f"[Process Start] Target: {request.target_weight}g, Mix: {request.mixing_duration}s")
        
        try:
            # 1. ScaleDriver: 영점 조절 (Tare)
            if not await self.call_service(self.cli_scale, mode="TARE"):    # await: call_service가 완료되어 True/False를 반환할 때까지 여기서 멈춰 기다립니다.
                raise Exception("Scale Tare Failed")

            # 2. TaskTransfer: 용기 픽업 (Pickup)
            if not await self.call_service(self.cli_transfer, mode="PICKUP"):
                raise Exception("Transfer Pickup Failed")

            # 3. TaskPouring: 용액 붓기 (Pouring)
            # 붓기 작업에는 target_weight 전달 필요
            if not await self.call_service(self.cli_pouring, mode="POUR", target_weight=request.target_weight):
                raise Exception("Pouring Failed")

            # 4. TaskMixing: 교반 (Mixing)
            # 교반 작업에는 mixing_duration 전달 필요
            if not await self.call_service(self.cli_mixing, mode="MIX", mixing_duration=request.mixing_duration):
                raise Exception("Mixing Failed")

            # 5. TaskTransfer: 용기 복귀 (Return)
            if not await self.call_service(self.cli_transfer, mode="RETURN"):
                raise Exception("Transfer Return Failed")

            # 모든 공정 성공
            response.success = True
            response.message = "All tasks completed successfully."
            self.get_logger().info("✅ [Process Complete] All tasks finished.")

        except Exception as e:
            # 공정 실패 시 처리
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"❌ [Process Failed] {e}")

        return response

    async def call_service(self, client, mode="", target_weight=0.0, mixing_duration=0.0):
        """[추가] 비동기 서비스 요청 헬퍼 함수"""
        req = RobotCommand.Request()
        req.mode = mode
        req.target_weight = float(target_weight)
        req.mixing_duration = float(mixing_duration)

        self.get_logger().info(f" -> Requesting {client.srv_name} | Mode: {mode}")
        
        # 비동기 호출 후 응답 대기 (await 사용)
        future = client.call_async(req) # 비동기 호출(call_async)을 보냅니다. 즉시 Future 객체(나중에 결과가 올 것이라는 약속)를 받습니다.
        result = await future   # await future: 서버에서 응답이 올 때까지 여기서 대기합니다.

        if result.success:
            self.get_logger().info(f"    Create Success: {result.message}")
            return True
        else:
            self.get_logger().error(f"    Failed: {result.message}")
            return False

def main(args=None):
    rclpy.init(args=args)
    
    # [추가] MultiThreadedExecutor 사용 권장 (혹은 기본 Executor에서도 ReentrantGroup 동작함)
    controller = SystemController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        # [추가] 서버 모드로 동작하므로 무한 대기 (UserInterface의 요청 대기)
        print(" [System Controller] Ready and waiting for commands from UserInterface...")
        executor.spin()  # MultiThreadedExecutor로 노드 실행 (서비스 요청 처리)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()