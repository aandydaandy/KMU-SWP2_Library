#include <Arduino.h>

// ----- 하드웨어 설정 -----
#define PIN_IR   A0        // 적외선 거리 센서 핀 (ADC)
#define BAUDRATE 1000000   // 시리얼 통신 속도

// ----- Curve fitting 설정 -----
#define MAX_POINTS  16     // 최대 측정 점 개수
#define MAX_DEGREE  3      // 최대 다항식 차수 (3차까지)

// 측정 데이터 (x: 전압 ADC, y: 거리 cm)
float x_data[MAX_POINTS];
float y_data[MAX_POINTS];

// 함수 프로토타입
int   readIntFromSerial(const char *msg, int minVal, int maxVal);
void  waitEnterKey(const char *msg);
unsigned int measureIR();
void  polyfit(const float *x, const float *y, int nPoints,
              int degree, float *coef);
void  printEquation(float *coef, int degree);

void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial) ;  // USB 시리얼 준비될 때까지 대기 (레오나르도 계열용)

  Serial.println();
  Serial.println("===== IR Sensor Curve Fitting Program =====");
  Serial.println("거리센서 곡선맞춤을 자동으로 수행합니다.");
  Serial.println();

  // 1) Curve fitting 방식 / 차수 입력 (여기서는 다항식 차수만 입력 받음)
  int degree = readIntFromSerial(
      "사용할 다항식 차수를 입력하세요 (1~3): ", 1, MAX_DEGREE);

  // 2) 측정할 점 개수 / 거리 설정
  //    과제 조건: 0, 5, 10, ..., 30 cm (총 7개 점)
  const int nPoints = 7;
  float distances[nPoints];

  for (int i = 0; i < nPoints; i++) {
    distances[i] = 5.0 * i;   // 0, 5, 10, ..., 30
  }

  Serial.println();
  Serial.println("탁구공의 위치를 0, 5, 10, ..., 30 cm 로 옮기면서");
  Serial.println("각 위치마다 [Enter] 키를 누르면 센서 전압을 측정합니다.");
  Serial.println();

  // 3) 각 거리에서 Enter 입력 → 센서 전압 측정
  for (int i = 0; i < nPoints; i++) {
    Serial.print(">> 탁구공을 ");
    Serial.print(distances[i]);
    Serial.println(" cm 위치에 놓고 [Enter]를 누르세요.");

    waitEnterKey("측정 중...");

    unsigned int adc = measureIR();

    x_data[i] = (float)adc;       // x: ADC 값
    y_data[i] = distances[i];     // y: 실제 거리(cm)

    Serial.print("   ADC = ");
    Serial.print(adc);
    Serial.println(" (0~1023)");
    Serial.println();
  }

  // 4) curve fitting 수행 (최소제곱법으로 다항식 계수 계산)
  float coef[MAX_DEGREE + 1];  // coef[0] + coef[1]*x + ...
  polyfit(x_data, y_data, nPoints, degree, coef);

  // 5) 결과 equation을 시리얼 모니터로 출력
  Serial.println("===== Curve Fitting Result =====");
  Serial.println("거리 = f(ADC값) 형태의 식을 출력합니다.");
  Serial.println("변수 이름: v  (ADC 측정값)");
  Serial.println();

  printEquation(coef, degree);

  Serial.println();
  Serial.println("아래 함수는 아두이노 스케치에 바로 붙여넣어 사용할 수 있습니다.");
  Serial.println();

  // Horner 형태로 함수 출력 (실제 코드에 바로 사용 가능)
  Serial.println("float ir_distance(int v)");
  Serial.println("{");
  Serial.print("  return ");

  // Horner form: (...((a_n * v + a_{n-1}) * v + ... ) * v + a_0)
  Serial.print(coef[degree], 8);
  for (int i = degree - 1; i >= 0; i--) {
    Serial.print(" * v + ");
    Serial.print(coef[i], 8);
  }
  Serial.println(";");
  Serial.println("}");
  Serial.println();
  Serial.println("===== Program Finished =====");
}

void loop()
{
  // 한 번만 실행하면 되므로 loop 는 비워둠
}

// -------------------------------------------------------------
// 유틸 함수들
// -------------------------------------------------------------

// 정수 입력 받기
int readIntFromSerial(const char *msg, int minVal, int maxVal)
{
  while (true) {
    Serial.print(msg);

    while (!Serial.available())
      ; // 입력 기다리기

    String line = Serial.readStringUntil('\n');
    line.trim();
    int value = line.toInt();

    if (value >= minVal && value <= maxVal) {
      Serial.print("  -> 입력값: ");
      Serial.println(value);
      Serial.println();
      return value;
    }

    Serial.println("  !! 잘못된 입력입니다. 다시 입력하세요.");
  }
}

// Enter 키 대기
void waitEnterKey(const char *msg)
{
  Serial.println(msg);
  // 버퍼 비우기
  while (Serial.available()) Serial.read();

  // '\n' 또는 '\r' 하나라도 들어오면 통과
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
    }
  }
}

// IR 센서 값을 여러 번 읽어 평균(노이즈 줄이기)
unsigned int measureIR()
{
  const int N = 20;   // 샘플 개수
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(PIN_IR);
    delay(10);
  }
  return (unsigned int)(sum / N);
}

// -------------------------------------------------------------
// 다항식 curve fitting (최소제곱법, 1~3차)
//  x: ADC 값 배열, y: 거리(cm) 배열
// -------------------------------------------------------------
void polyfit(const float *x, const float *y, int nPoints,
             int degree, float *coef)
{
  int m = degree + 1;              // 미지수 개수
  float X[2 * MAX_DEGREE + 2];     // Σ x^k
  float B[MAX_DEGREE + 1];         // Σ x^k y
  float A[MAX_DEGREE + 1][MAX_DEGREE + 2]; // 확장 행렬

  // Σ x^k 계산
  for (int k = 0; k < 2 * m; k++) {
    X[k] = 0.0;
    for (int i = 0; i < nPoints; i++) {
      X[k] += pow(x[i], k);
    }
  }

  // Σ x^k y 계산
  for (int k = 0; k < m; k++) {
    B[k] = 0.0;
    for (int i = 0; i < nPoints; i++) {
      B[k] += pow(x[i], k) * y[i];
    }
  }

  // 정규방정식 행렬 A 구성
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      A[i][j] = X[i + j];
    }
    A[i][m] = B[i];   // 우변
  }

  // 가우스 소거법으로 Ax = B 풀기
  for (int i = 0; i < m; i++) {
    // 피벗 선택 (부분 피벗팅)
    int maxRow = i;
    for (int k = i + 1; k < m; k++) {
      if (fabs(A[k][i]) > fabs(A[maxRow][i])) {
        maxRow = k;
      }
    }
    // 행 교환
    if (maxRow != i) {
      for (int j = i; j <= m; j++) {
        float tmp = A[i][j];
        A[i][j] = A[maxRow][j];
        A[maxRow][j] = tmp;
      }
    }

    // 피벗으로 나눠서 1 만들기
    float pivot = A[i][i];
    if (pivot == 0) continue;  // (이상한 경우 방지용)

    for (int j = i; j <= m; j++) {
      A[i][j] /= pivot;
    }

    // 다른 행에서 i열 제거
    for (int k = 0; k < m; k++) {
      if (k == i) continue;
      float factor = A[k][i];
      for (int j = i; j <= m; j++) {
        A[k][j] -= factor * A[i][j];
      }
    }
  }

  // 해 추출
  for (int i = 0; i < m; i++) {
    coef[i] = A[i][m];
  }
}

// 식을 사람이 보기 좋게 출력 (d = a0 + a1*v + a2*v*v + ...)
void printEquation(float *coef, int degree)
{
  Serial.print("d = ");

  for (int i = 0; i <= degree; i++) {
    if (i > 0) Serial.print(" + ");

    Serial.print(coef[i], 8);

    if (i >= 1) {
      Serial.print(" * v");
      for (int p = 2; p <= i; p++) {
        Serial.print(" * v");
      }
    }
  }

  Serial.println(";");
}
