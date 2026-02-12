#include "HX711.h"    // 로드셀 제어를 위한 라이브러리 불러오기
HX711 scale;          // 로드셀 객체 생성 (이름: scale)

uint8_t dataPin = 3;  // 데이터 핀(DT)을 아두이노 3번 핀에 연결
uint8_t clockPin = 2; // 클럭 핀(SCK)을 아두이노 2번 핀에 연결
float w1, w2, previous = 0;    // 무게 값을 저장할 변수들 (w1: 현재, w2: 비교용, previous: 이전 값)

void setup()
{
  Serial.begin(115200);    // PC와 통신 속도 설정 (115200bps)
  Serial.println(__FILE__);
  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();
  scale.begin(dataPin, clockPin);    // 로드셀 시작 (핀 설정 적용)
  Serial.print("UNITS: ");
  Serial.println(scale.get_units(10));    // 보정 전, 현재 센서값 10번 평균내서 출력 (쓰레기값일 가능성 높음)
  scale.set_scale(390.345611);       // 여기에 calibration에서 얻은 보정값을 입력
  scale.tare();                      // ★ 중요: 영점 조절 (현재 상태를 0g으로 맞춤)
  Serial.print("UNITS: ");    
  Serial.println(scale.get_units(10));     // 영점 잡은 후, 0에 가까운지 확인 출력
}

void loop()
{
  // read until stable
  w1 = scale.get_units(10);    // 센서값을 10번 읽어 평균을 냄 (노이즈 줄이기)
  delay(100);                  // 0.1초 대기
  w2 = scale.get_units();      // 센서값을 1번 더 읽음 (비교용)
  while (abs(w1 - w2) > 10)    // w1(평균값)과 w2(최신값)의 차이가 10보다 크면 "아직 흔들리는 중"이라고 판단
  {
     w1 = w2;
     w2 = scale.get_units();
     delay(100);
  }
  Serial.print("UNITS: ");
  Serial.print(w1);           // 안정된 무게 값 출력
  if (w1 == 0)
  {
    Serial.println();
  }
  else
  {
    Serial.print("\t\tDELTA: ");
    Serial.println(w1 - previous);
    previous = w1;
  }
  delay(100);
}