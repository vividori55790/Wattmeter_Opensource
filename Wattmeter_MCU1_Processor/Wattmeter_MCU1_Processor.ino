/*
 * --- PROCESSOR 코드 (Nano R4 - 통신 테스트) ---
 * Serial1 (D0, D1)을 사용합니다.
 */

void setup() {
  // Serial.begin(9600); // (X) 이게 아님
  Serial1.begin(9600); // D0, D1 하드웨어 시리얼
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (Serial1.available() > 0) { // Serial1로 변경
    char cmd = Serial1.read(); // Serial1로 변경

    if (cmd == 'P') {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

      // 'K' 응답을 Serial1 (D1 핀)으로 보냄
      Serial1.write('K');
    }
  }
}