import pandas as pd
import numpy as np

# 1. 데이터 불러오기 (첫 번째 줄을 제목(Header)으로 인식하도록 header=0 적용)
df = pd.read_csv('/home/rokey/Co-Lab/pouring_metrics.csv', header=0)

# 컬럼 이름 깔끔하게 덮어쓰기 (총 12개)
df.columns = [
    'Timestamp', 'Target_W', 'Final_W', 'P_GAIN', 'D_GAIN', 'MAX_TILT_STEP', 
    'STOP_THRESHOLD', 'Overshoot(g)', 'Rise_Time', 'Settling_Time', 'SS_Error(g)', 'Material'
]

# 1.5. [핵심 에러 해결] 계산할 열들을 확실하게 숫자(float)로 변환 (빈칸이나 에러값은 NaN 처리)
cols_to_numeric = ['Target_W', 'Final_W', 'P_GAIN', 'D_GAIN', 'SS_Error(g)', 'Rise_Time', 'Settling_Time']
for col in cols_to_numeric:
    df[col] = pd.to_numeric(df[col], errors='coerce')

# 2. 신규 분석 지표 생성
# ① Error Rate (오차율, %)
df['Error_Rate(%)'] = (df['SS_Error(g)'] / df['Target_W']) * 100

# ② P_Gain / D_Gain 비율 계산
df['P_D_Ratio'] = df['P_GAIN'] / df['D_GAIN']

# ③ 평균 붓기 속도 (g/s) : Rise Time이 0인 경우 0으로 처리 (ZeroDivisionError 방지)
df['Avg_Pouring_Rate(g/s)'] = np.where(df['Rise_Time'] > 0, df['Final_W'] / df['Rise_Time'], 0)

# ④ Cycle Time & Overhead Time (8초 가산 예시)
df['Cycle_Time'] = df['Settling_Time'] + 8.0
df['Overhead_Time'] = df['Cycle_Time'] - df['Settling_Time']

# 3. 데이터 깔끔하게 소수점 2자리 반올림
numeric_cols = df.select_dtypes(include=[np.number]).columns
df[numeric_cols] = df[numeric_cols].round(2)

# 4. 새로운 파일로 저장 (요청하신 경로와 파일명 적용)
save_path = '/home/rokey/Co-Lab/pouring_metrics_new.csv'
df.to_csv(save_path, index=False, encoding='utf-8-sig')

print(f"✅ 데이터 처리가 완료되어 '{save_path}'에 성공적으로 저장되었습니다!")