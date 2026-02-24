#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import firebase_admin
from firebase_admin import credentials, db
import time
import datetime

print("🔄 Firebase 초기화 중...")

try:
    # 1. 인증키 로드 및 앱 초기화
    key_path = "/home/rokey/Co-Lab/serviceAccountKey.json"
    db_url = "https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app"
    
    if not firebase_admin._apps:
        cred = credentials.Certificate(key_path)
        firebase_admin.initialize_app(cred, {'databaseURL': db_url})
        
    print("🔥 Firebase 앱 초기화 성공!\n")
    
    # 2. 실제 user_interface.py가 전송하는 데이터와 완벽히 동일한 구조의 가상 데이터
    mock_history_data = {
        'timestamp': int(time.time() * 1000),
        'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'material': '가상 테스트 레시피 (무지개10/푸른색10/자갈100)',
        'target_weight': 120.0,
        'final_weight': 121.5,    # 1.5g 오버됨
        'error_rate': 1.25,       # 오차율 1.25%
        'success': True,          # 10% 이내이므로 성공
        'ss_error_g': 1.5,        # 잔류 오차
        'cycle_time': 45.2        # 소요 시간
    }
    
    # 3. experiment_history 경로에 데이터 Push (누적 저장)
    print("📤 'experiment_history' 경로에 1회차 데이터를 전송하는 중...")
    db_ref = db.reference('experiment_history')
    new_record = db_ref.push(mock_history_data)
    
    print(f"✅ DB 쓰기 성공! (생성된 고유 ID: {new_record.key})")
    print("\n🎉 완벽합니다! 지금 바로 다음 두 가지를 확인해 보세요:")
    print("  1. Firebase 콘솔 [experiment_history] 아래에 새로운 키가 생겼는지 확인")
    print("  2. ui.html 웹 화면 하단 '실험 히스토리 표'에 [가상 테스트 레시피]가 추가되었는지 확인")

except Exception as e:
    print(f"\n❌ 통신 에러 발생: {e}")