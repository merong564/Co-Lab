import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from colab_interfaces.srv import RobotCommand
from colab_interfaces.msg import SystemStatus # [추가]
import time

ROBOT_ID = "dsr01"

class SystemController(Node):
    def __init__(self):
        super().__init__('SystemController', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # 중단 요청 플래그
        self.is_stop_requested = False

        self.current_held_target = ""

        self.current_phase = "Ready" # [추가] 현재 공정 상태 저장 변수
        self.prev_tcp_vel = 0.0 # [추가] 이전 속도 저장 변수
        self.prev_time = self.get_clock().now() # [추가] 이전 시간 저장 변수

        # Message Publisher: 비상 정지 및 시스템 상태 토픽 발행
        self.pub_stop = self.create_publisher(String, 'stop', 10)
        self.pub_status = self.create_publisher(SystemStatus, 'system_status', 10) # [추가] 상태 퍼블리셔

        self.status_timer = self.create_timer(0.1, self.publish_status_callback, callback_group=self.callback_group) # [추가] 10Hz 상태 퍼블리시 타이머

        # Message Subscriber: 비상 정지 관련 토픽 구독
        self.sub_stop = self.create_subscription(String, 'stop/impact', self.stop_callback, 10, callback_group=self.callback_group)     # 외력 감지 시 정지
        self.sub_ui_stop = self.create_subscription(String, 'stop/ui', self.stop_callback, 10, callback_group=self.callback_group)      # UI에서 비상정지 버튼 누를 시 정지

        # Service Server
        self.srv_start = self.create_service(RobotCommand, 'start_process', self.handle_start_process, callback_group=self.callback_group)

        # Service Clients
        self.cli_scale = self.create_client(RobotCommand, 'set_tare', callback_group=self.callback_group)
        self.cli_transfer = self.create_client(RobotCommand, 'execute_transfer', callback_group=self.callback_group)
        self.cli_pouring = self.create_client(RobotCommand, 'execute_pouring', callback_group=self.callback_group)
        self.cli_mixing = self.create_client(RobotCommand, 'execute_mixing', callback_group=self.callback_group)
        self.cli_recovery = self.create_client(RobotCommand, 'execute_recovery', callback_group=self.callback_group) # [추가] 복구 노드 클라이언트


        self.check_services_availability()

    # [추가] 상태 퍼블리시 콜백 함수
    def publish_status_callback(self):
        msg = SystemStatus()
        msg.phase = self.current_phase
        self.pub_status.publish(msg)

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
            ('TaskMixing', self.cli_mixing),
            ('TaskRecovery', self.cli_recovery) # [추가] 복구 노드 연결 확인
        ]
        for name, client in clients:
            self.get_logger().info(f'Waiting for {name} server...')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting again...')
        self.get_logger().info('All Service Servers Connected!')

    def set_phase(self, new_phase):
        self.current_phase = new_phase
        msg = SystemStatus()
        msg.phase = self.current_phase
        self.pub_status.publish(msg)
        self.get_logger().info(f"[Phase Changed] -> {new_phase}")

    async def handle_start_process(self, request, response):
        self.get_logger().info("=" * 40)
        self.get_logger().info("[Process Start]")

        self.is_stop_requested = False
        # current_target = "" # [추가] 로봇이 현재 파지 중인 물체 추적

        # [추가] 프로세스 시작 시 초기화
        self.current_held_target = ""
        self.set_phase("Ready") # [수정] 상태 초기화 시 함수 사용

        try:
            for target, weight in zip(request.targets, request.target_weights):
                self.get_logger().info(f"[Task] Target: {target}, Target Weight: {weight}g")

                if self.check_stop(): raise Exception("Process Aborted by User")
                self.set_phase("Taring") # [추가] 상태 변경
                if not await self.call_service(self.cli_scale, mode="TARE"):
                    raise Exception("Scale Tare Failed")

                if self.check_stop(): raise Exception("Process Aborted by User")
                self.set_phase("Transfer") # [추가] 상태 변경
                if not await self.call_service(self.cli_transfer, mode="PICKUP", targets=[target]):
                    raise Exception(f"Transfer Pickup Failed for {target}")
                # current_target = target # [추가] 픽업 성공, 현재 물체 파지 중

                if self.check_stop(): raise Exception("Process Aborted by User")
                self.set_phase("Pouring") # [추가] 상태 변경
                if not await self.call_service(self.cli_pouring, mode="POUR", targets=[target], target_weights=[weight]):
                    raise Exception(f"Pouring Failed for {target}")

                if self.check_stop(): raise Exception("Process Aborted by User")
                self.set_phase("Return") # [추가] 상태 변경
                if not await self.call_service(self.cli_transfer, mode="RETURN", targets=[target]):
                    raise Exception(f"Transfer Return Failed for {target}")
                # current_target = "" # [추가] 리턴 성공, 빈 손 상태

            if self.check_stop(): raise Exception("Process Aborted by User")
            self.set_phase("Mixing") # [추가] 상태 변경
            if not await self.call_service(self.cli_mixing, mode="MIX", mixing_duration=request.mixing_duration):
                raise Exception("Mixing Failed")

            if self.check_stop(): raise Exception("Process Aborted by User")
            # current_target = "BEAKER" # [추가] 믹싱 완료 후 비커 파지 상태로 간주
            self.set_phase("Return") # [추가] 상태 변경
            if not await self.call_service(self.cli_transfer, mode="RETURN", targets=["BEAKER"]):
                raise Exception("Final Return Failed for BEAKER")
            # current_target = "" # [추가] 비커 리턴 성공

            response.success = True
            response.message = "All tasks completed successfully."
            self.set_phase("Ready") # [추가] 모든 작업 완료 후 대기 상태
            self.get_logger().info("[Process Complete] All tasks finished.")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"[Process Failed/Aborted] {e}")

            # [추가] 에러 발생 시 자동 복구 시퀀스 진행
            self.get_logger().info("=== [Auto-Recovery Sequence Initiated] ===")
            self.set_phase("Recovery") # [추가] 상태 변경

            # [수정] 1. 급정지로 인한 로봇의 물리적 흔들림(여진)이 완전히 멈출 때까지 대기
            self.get_logger().info("Waiting for physical vibrations to settle...")
            time.sleep(1.5)
            
            # 1. 락 해제를 위해 RESET 발행
            self.is_stop_requested = False
            msg = String()
            msg.data = "RESET"
            self.pub_stop.publish(msg)
            self.get_logger().info("Published 'RESET' to unlock working nodes.")
            
            # 2. 노드들이 RESET을 처리할 수 있도록 잠시 대기
            time.sleep(0.5)
            
            # 3. 복구 서비스 호출
            self.get_logger().info(f"Calling execute_recovery with target: '{self.current_held_target}'")
            rec_req = RobotCommand.Request()
            rec_req.mode = "RECOVER"
            if self.current_held_target:
                rec_req.targets = [self.current_held_target]
            
            rec_result = await self.cli_recovery.call_async(rec_req)
            
            if rec_result.success:
                self.get_logger().info("=== [Auto-Recovery Completed Successfully] ===")
                self.set_phase("Ready") # [추가] 복구 완료 후 대기 상태
            else:
                self.get_logger().error(f"=== [Auto-Recovery Failed]: {rec_result.message} ===")

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
            self.current_held_target = getattr(result, "held_object", "")
            if self.current_held_target:
                self.get_logger().warn(f"    [State Updated] Node reported holding: '{self.current_held_target}'")
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