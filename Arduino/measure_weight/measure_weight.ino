/**
 *
 * HX711 library for Arduino - example file
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/
#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;


HX711 scale;

void setup() {
  Serial.begin(115200);
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(436.6f);
  scale.tare();				        // reset the scale to 0

}

void loop() {
  if (scale.is_ready()) {
    // 2. 텍스트 없이 오직 '값'만 출력하고 줄바꿈(println)
    // get_units(1)은 1회 측정값을 의미합니다. 
    // HX711은 하드웨어적으로 속도가 느리므로(10Hz 또는 80Hz), 
    // 여기서 평균을 많이 내면(get_units(10) 등) 로봇 제어 주기가 밀릴 수 있습니다.
    float weight = scale.get_units(1); 
    
    Serial.println(weight, 2); // 소수점 2자리까지 출력
}
}
