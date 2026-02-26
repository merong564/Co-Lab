# 1. 로봇 및 공정 상태 (Real-time)
string phase           # 현재 단계 (Ready, Transfer, Pouring, Mixing, Return)


# 2. 통계 데이터 (Cumulative)
int32 total_count      # 총 작업 횟수
int32 success_count    # 성공 횟수
float32 last_cycle_time # 직전 사이클 소요 시간 [sec]