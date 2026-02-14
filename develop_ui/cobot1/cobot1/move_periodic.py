import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode
    
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # # Tool과 TCP 설정
    # set_tool(ROBOT_TOOL)
    # set_tcp(ROBOT_TCP)


# def perform_task():
#     # 1. 모드 설정을 위해 필요한 기능(set_robot_mode, ROBOT_MODE_AUTONOMOUS)을 추가로 임포트
#     from DSR_ROBOT2 import movej, movel, movec, posx, DR_MV_MOD_ABS, set_robot_mode, ROBOT_MODE_AUTONOMOUS

#     print("🚀 [Task] 작업을 시작합니다.")

#     # ★★★ 핵심 수정: 로봇을 자율 모드로 변경 (이게 없으면 안 움직임!) ★★★
#     set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    
#     # ---------------------------------------------------
#     # Step 1: 원을 그리기 시작할 위치로 먼저 이동 (Start Point)
#     # ---------------------------------------------------
#     # posx(X, Y, Z, A, B, C)
#     start_p = posx(370, 0, 400, 0, 180, 0) 
    
#     print(f"이동 중: 시작 위치로 이동합니다... {start_p}")
#     movel(start_p, vel=VELOCITY, acc=ACC)


#     # ---------------------------------------------------
#     # Step 2: 원형 이동 (MoveC) 실행
#     # ---------------------------------------------------
#     # 경유점 (Via Point)
#     via_p = posx(370, 150, 400, 0, 180, 0)
    
#     # 목표점 (Target Point)
#     target_p = posx(220, 150, 400, 0, 180, 0)

#     print("이동 중: 원형 이동(MoveC)을 수행합니다.")
    
#     # movec(경유점, 목표점, 속도, 가속도)
#     movec(via_p, target_p, vel=VELOCITY, acc=ACC)

#     print("✅ [Task] 작업 완료!")

def perform_task():
    # 1. 필요한 함수들 가져오기 (wait, posj 추가)
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS, wait
    
    print("🚀 [Task] 작업을 시작합니다.")

    # 2. (중요) 자율 모드 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 3. 두 가지 포즈 정의 (관절 각도: J1, J2, J3, J4, J5, J6)
    # 1번 자세: 차렷 (비슷한 자세)
    p1 = posj(0, 0, 90, 0, 90, 0)
    # 2번 자세: 허리(J1)를 90도 돌린 자세
    p2 = posj(90, 0, 90, 0, 90, 0)

    print("🔄 반복 동작 시작 (Ctrl+C로 멈출 때까지 계속함)")

    count = 0
    while True: # 무한 반복
        count += 1
        print(f"[{count}회차] 👉 1번 자세로 이동")
        movej(p1, vel=100, acc=100)
        wait(0.5) # 도착 후 0.5초 대기

        print(f"[{count}회차] 👈 2번 자세로 이동 (크게 움직임!)")
        movej(p2, vel=100, acc=100)
        wait(0.5)

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()