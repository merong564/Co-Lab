import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 이동 속도 및 가속도 (테스트를 위해 조금 천천히 설정)
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇 초기화 및 설정"""
    # DSR_ROBOT2 모듈은 DR_init 설정 후에 임포트해야 합니다.
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
    
    print("#" * 50)
    print(f"Initializing {ROBOT_ID} ({ROBOT_MODEL})...")
    print("#" * 50)

def perform_pouring_task():
    """J6 관절을 이용한 붓기 시뮬레이션"""
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait

    print("🚀 [Simulation] 붓기 동작 테스트를 시작합니다.")

    # 1. 자율 모드 설정 (필수)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # ---------------------------------------------------
    # 자세 정의 (posj: J1, J2, J3, J4, J5, J6)
    # ---------------------------------------------------
    
    # [자세 1] 준비 자세 (Ready)
    # 컵을 수평으로 들고 있는 자세라고 가정
    # J6(마지막 숫자)가 0도
    p_ready = posj(0, 0, 90, 0, 90, 0)

    # [자세 2] 붓기 자세 (Pouring)
    # J6를 77도로 꺾음 (가상 센서가 반응할 각도)
    p_pour = posj(0, 0, 90, 0, 90, 77)

    count = 0
    
    try:
        while True:
            count += 1
            print(f"\n=== [ {count}회차 반복 시작 ] ===")

            # 1. 준비 자세로 이동 (물 멈춤)
            print("🧴 [Ready] 준비 자세로 이동 (J6 = 0도)")
            movej(p_ready, vel=VELOCITY, acc=ACC)
            # 도착 후 잠시 대기 (그래프가 멈추는지 확인용)
            wait(2.0) 

            # 2. 붓기 자세로 이동 (물 나옴)
            print(f"🚰 [Pouring] 붓는 중... (J6 = 77도)")
            movej(p_pour, vel=VELOCITY, acc=ACC)
            
            # 붓는 상태를 유지 (이 시간 동안 가상 센서가 무게를 계속 올림)
            wait(3.0) 
            
    except KeyboardInterrupt:
        print("작업 중단 요청됨.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pouring_simulation", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_pouring_task()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()