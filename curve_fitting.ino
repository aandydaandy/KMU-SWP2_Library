#include <Arduino.h>
#include <math.h>   // pow, fabs 사용

// ----- 하드웨어 설정 -----
#define PIN_IR   A0        // IR 센서 연결 핀 (아날로그 A0)
#define BAUDRATE 1000000   // 시리얼 통신 속도 (시리얼 모니터도 동일하게 설정)

// ----- Curve fitting 설정 -----
#define MAX_POINTS  16     // 최대 측정 점 개수
#define MAX_DEGREE  3      // 지원하는 최대 다항식 차수 (3차까지)

// 측정 데이터 저장 배열 (x: ADC, y: 거리 cm)
float x_data[MAX_POINTS];
float y_data[MAX_POINTS];

// 함수 선언
int   readIntFromSerial(const char *msg, int minVal, int maxVal);
void  waitEnterKey(const char *msg);
unsigned int measureIR();
void  polyfit(const float *x, const float *y, int nPoints, int degree, float *coef);
void  printEquation(float *coef, int degree);
void  printIrDistanceFunction(float *coef, int degree);

void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial) ;   // USB 시리얼 준비 대기 (UNO면 그냥 바로 지나감)

  Serial.println();
  Serial.println("===== IR Sensor Curve Fitting Program =====");
  Serial.println("IR 센서 전압(ADC) -> 거리(cm) 곡선맞춤을 수행합니다.");
  Serial.println();

  // 1) 사용할 다항식 차수 입력
  int degree = readIntFromSerial("사용할 다항식 차수 (1~3): ", 1, MAX_DEGREE);

  // 2) 측정할 거리 값들 (0, 5, 10, ..., 30 cm)
  const int nPoints = 7;
  float distances[nPoints];
  for (int i = 0; i < nPoints; i++) {
    distances[i] = 5.0 * i;   // 0, 5, 10, 15, 20, 25, 30
  }

  Serial.println("각 거리마다 탁구공을 놓고 [Enter] 키를 누르면 ADC 값을 측정합니다.");
  Serial.println("거리는 0, 5, 10, 15, 20, 25, 30 cm 입니다.");
  Serial.println();

  // 3) 거리별로 IR 센서 값 측정
  for (int i = 0; i < nPoints; i++) {
    Serial.print(">> 탁구공을 ");
    Serial.print(distances[i]);
    Serial.println(" cm 위치에 놓고 [Enter] 키를 누르세요.");

    waitEnterKey("측정 중...");

    unsigned int adc = measureIR();
    x_data[i] = (float)adc;       // x: ADC 값
    y_data[i] = distances[i];     // y: 실제 거리(cm)

    Serial.print("   측정된 ADC 값: ");
    Serial.println(adc);
    Serial.println();
  }

  // 4) curve fitting 수행
  float coef[MAX_DEGREE + 1];   // coef[0] + coef[1]*v + coef[2]*v^2 + ...
  polyfit(x_data, y_data, nPoints, degree, coef);

  // 5) 결과 출력
  Serial.println("===== Curve Fitting Result =====");
  Serial.println("거리 = f(ADC값) 형태의 식을 출력합니다.");
  Serial.println("변수 이름: v  (ADC 측정값)");
  Serial.println();

  printEquation(coef, degree);

  Serial.println();
  Serial.println("아래 함수는 아두이노 스케치에 바로 붙여넣어 사용할 수 있습니다.");
  Serial.println();

  printIrDistanceFunction(coef, degree);

  Serial.println();
  Serial.println("===== Program Finished =====");
}

void loop()
{
  // 한 번만 실행하면 되므로 비워둠
}

// -------------------------------------------------------------
// 유틸리티 함수들
// -------------------------------------------------------------

// 시리얼에서 정수 입력 받기
int readIntFromSerial(const char *msg, int minVal, int maxVal)
{
  while (true) {
    Serial.print(msg);

    while (!Serial.available())
      ;   // 입력 기다리기

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

// Enter 키가 눌릴 때까지 대기
void waitEnterKey(const char *msg)
{
  Serial.println(msg);

  // 기존 버퍼 비우기
  while (Serial.available())
    Serial.read();

  // '\n' 또는 '\r' 들어올 때까지 기다리기
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        break;
      }
    }
  }
}

// IR 센서 값 여러 번 읽어서 평균
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
  int m = degree + 1;              // 미지수 개수 (a0~a_degree)
  float X[2 * MAX_DEGREE + 2];     // Σ x^k
  float B[MAX_DEGREE + 1];         // Σ x^k y
  float A[MAX_DEGREE + 1][MAX_DEGREE + 2]; // 확장 행렬

  // Σ x^k 계산 (k = 0 ~ 2m-1)
  for (int k = 0; k < 2 * m; k++) {
    X[k] = 0.0;
    for (int i = 0; i < nPoints; i++) {
      X[k] += pow(x[i], k);
    }
  }

  // Σ x^k * y 계산
  for (int k = 0; k < m; k++) {
    B[k] = 0.0;
    for (int i = 0; i < nPoints; i++) {
      B[k] += pow(x[i], k) * y[i];
    }
  }

  // 정규 방정식 행렬 A 구성
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      A[i][j] = X[i + j];
    }
    A[i][m] = B[i];   // 우변
  }

  // 가우스 소거법으로 Ax = B 풀기
  for (int i = 0; i < m; i++) {
    // 부분 피벗팅
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

    // 피벗으로 나누어 1 만들기
    float pivot = A[i][i];
    if (pivot == 0.0f) continue;  // (이상 상황 방지용)

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

  // 해 추출 (coef 배열에 저장)
  for (int i = 0; i < m; i++) {
    coef[i] = A[i][m];
  }
}

// -------------------------------------------------------------
// 사람 읽기용 식 출력
// d = a0 + a1 * v + a2 * v * v + ...
// -------------------------------------------------------------
void printEquation(float *coef, int degree)
{
  Serial.print("d = ");

  for (int i = 0; i <= degree; i++) {
    if (i > 0) Serial.print(" + ");

    Serial.print(coef[i], 8);

    if (i >= 1) {
      // i번 곱해진 v: v, v*v, v*v*v, ...
      for (int k = 0; k < i; k++) {
        Serial.print(" * v");
      }
    }
  }

  Serial.println(";");
}

// -------------------------------------------------------------
// 아두이노 코드로 바로 복붙할 함수 출력
// float ir_distance(int v) { return a0 + a1 * v + a2 * v * v + ...; }
// -------------------------------------------------------------
void printIrDistanceFunction(float *coef, int degree)
{
  Serial.println("float ir_distance(int v)");
  Serial.println("{");
  Serial.print("  return ");

  for (int i = 0; i <= degree; i++) {
    if (i > 0) Serial.print(" + ");

    Serial.print(coef[i], 8);

    if (i >= 1) {
      for (int k = 0; k < i; k++) {
        Serial.print(" * v");
      }
    }
  }

  Serial.println(";");
  Serial.println("}");
}
