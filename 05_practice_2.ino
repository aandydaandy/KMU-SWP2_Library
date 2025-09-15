#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // 1. 처음 1초 켜기
  digitalWrite(PIN_LED, 0);
  delay(1000);

  // 2. 1초 동안 5회 깜빡 (ON 100ms + OFF 100ms)
  for (int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }

  // 3. LED 끄고 무한루프
  digitalWrite(PIN_LED, 1);
  while (1) { }   // 종료 상태 유지
}
