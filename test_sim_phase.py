#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import firebase_admin
from firebase_admin import credentials, db
import time

print("🔄 Firebase 초기화 중...")

try:
    # 1. 인증키 로드 및 앱 초기화
    key_path = "/home/rokey/Co-Lab/serviceAccountKey.json"
    db_url = "https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app"
    
    if not firebase_admin._apps:
        cred = credentials.Certificate(key_path)
        firebase_admin.initialize_app(cred, {'databaseURL': db_url})
        
    print("🔥 Firebase 앱 초기화 성공!\n")
    
    # 2. 시뮬레이션할 공정 순서 리스트
    phases = ["Ready", "Transfer", "Pouring", "Mixing", "Return", "Ready"]
    
    db_ref = db.reference('system_stats')

    print("🚀 [테스트 시작] UI 화면의 'Cycle Phase' 영역을 지켜보세요!\n")
    
    # 3. 3초 간격으로 상태를 변경하며 Firebase에 전송
    for phase in phases:
        print(f"👉 현재 상태 변경: [{phase}] 전송 중...")
        
        # UI가 읽어가는 system_stats 경로의 phase 값을 업데이트
        # (시각적 효과를 위해 임의의 가짜 속도 데이터도 함께 넣습니다)
        db_ref.update({
            'phase': phase,
            'tcp_vel': 15.5 if phase != "Ready" else 0.0,
            'tcp_acc': 2.0 if phase != "Ready" else 0.0,
        })
        
        # 3초 대기 (이때 UI 화면이 부드럽게 전환됩니다)
        time.sleep(3.0)

    print("\n🎉 시뮬레이션 종료! 모든 공정이 한 바퀴를 돌았습니다.")

except Exception as e:
    print(f"\n❌ 에러 발생: {e}")