#include "HX711.h"                 // HX711(로드셀 앰프) 제어 라이브러리 포함
HX711 myScale;                     // HX711 객체 생성 (이 이름으로 센서 읽기/보정/영점 수행)

uint8_t dataPin  = 3;              // HX711의 DT(DOUT, 데이터 출력) 핀을 아두이노 D3에 연결
uint8_t clockPin = 2;              // HX711의 SCK(클럭) 핀을 아두이노 D2에 연결

uint32_t start, stop;              // (이 코드에서는 사용 안 함) 측정 시간 기록용으로 남아있는 변수
volatile float f;                  // (이 코드에서는 사용 안 함) 인터럽트/동시성 대비용으로 남아있는 변수

void setup()
{
  Serial.begin(115200);            // 시리얼 통신 시작 (PC ↔ 아두이노, 속도 115200bps)
  Serial.println(__FILE__);        // 현재 스케치(파일) 이름 출력(디버깅용)

  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION); // 사용 중인 HX711 라이브러리 버전 출력
  Serial.println();                // 줄바꿈

  myScale.begin(dataPin, clockPin); // HX711 초기화 + 핀 설정 적용(DT/SCK)
}

void loop()
{
  calibrate();                     // 캘리브레이션 함수를 실행
                                   // ⚠ loop에서 계속 부르므로, 캘리브레이션이 끝나도 다시 시작함
                                   // (보통은 한 번만 하고 멈추게 바꾸는 게 편함)
}

void calibrate()
{
  // ===== 캘리브레이션 시작 안내 =====
  Serial.println("\n\nCALIBRATION\n===========");   // 캘리브레이션 시작 헤더 출력
  Serial.println("remove all weight from the loadcell"); // 로드셀 위에 있는 물체를 전부 치우라는 안내

  // ===== 시리얼 입력 버퍼 비우기 =====
  while (Serial.available()) Serial.read();         // 이전에 남아있는 입력(문자)을 모두 읽어서 버림(버퍼 flush)

  // ===== 사용자가 엔터 칠 때까지 대기 =====
  Serial.println("and press enter\n");              // “치웠으면 엔터 치세요” 안내
  while (Serial.available() == 0);                  // 엔터(또는 어떤 입력) 들어올 때까지 여기서 멈춰 대기

  // ===== 0점(영점) 잡기: offset 계산 =====
  Serial.println("Determine zero weight offset");   // 영점(offset) 구한다는 안내
  myScale.tare(20);                                 // 20번 측정 평균으로 현재 상태를 0g으로 설정(영점 맞춤)
                                                    // → 내부적으로 offset 값을 계산/저장함

  uint32_t offset = myScale.get_offset();           // 방금 잡힌 offset(원시 ADC 기준값)을 가져옴
  Serial.print("OFFSET: ");                         // offset 출력 라벨
  Serial.println(offset);                           // offset 값 출력
  Serial.println();                                 // 줄바꿈

  // ===== 기준추를 올리고, 실제 무게를 입력받기 =====
  Serial.println("place a weight on the loadcell"); // 로드셀 위에 기준추(알고 있는 무게)를 올리라는 안내

  // ===== 시리얼 입력 버퍼 비우기(깔끔한 입력 받기) =====
  while (Serial.available()) Serial.read();         // 이전 입력 잔여분 제거

  Serial.println("enter the weight in (whole) grams and press enter"); 
  // “기준추 무게를 ‘그램 정수’로 입력 후 엔터” 안내 (예: 500)

  uint32_t weight = 0;                              // 사용자가 입력할 기준추 무게(g)를 저장할 변수

  // ===== 엔터('\n')가 들어올 때까지 숫자만 골라서 읽기 =====
  while (Serial.peek() != '\n')                     // 다음 문자(미리보기)가 엔터가 아닐 동안 반복
  {
    if (Serial.available())                         // 시리얼로 들어온 문자가 있으면
    {
      char ch = Serial.read();                      // 문자 1개 읽기
      if (isdigit(ch))                              // 그 문자가 숫자(0~9)면
      {
        weight *= 10;                               // 자릿수 올리기 (예: 5 → 50)
        weight = weight + (ch - '0');               // 숫자 값 더하기 (예: '3'이면 3 더함)
      }
      // 숫자가 아니면(스페이스 등) 그냥 무시
    }
  }

  // ===== 사용자가 입력한 기준추 무게 출력 =====
  Serial.print("WEIGHT: ");
  Serial.println(weight);                           // 입력받은 기준추 무게(g) 출력

  // ===== scale factor(보정계수) 계산 =====
  myScale.calibrate_scale(weight, 20);              // “현재 올린 물체가 weight(g)다” 라고 알려주고,
                                                    // 20번 평균 측정으로 g 단위 변환계수(scale)를 계산/저장

  float scale = myScale.get_scale();                // 계산된 scale factor(보정계수)를 가져옴

  Serial.print("SCALE:  ");
  Serial.println(scale, 6);                         // scale을 소수점 6자리까지 출력

  // ===== 결과를 복붙용 코드 형태로 출력 =====
  Serial.print("\nuse scale.set_offset(");          // setup()에 넣을 set_offset 코드 안내 출력 시작
  Serial.print(offset);                             // offset 값 출력
  Serial.print("); and scale.set_scale(");          // 이어서 set_scale 코드 안내
  Serial.print(scale, 6);                           // scale 값 출력
  Serial.print(");\n");                             // 줄바꿈 포함 마무리

  Serial.println("in the setup of your project");   // “이걸 너 프로젝트 setup에 넣어라” 안내
  Serial.println("\n\n");                           // 보기 좋게 줄바꿈 여러 번
}
