#include "arduino_secrets.h" // 보안 정보(예: WiFi 패스워드 등)는 이 파일에
// 작은 변경 사항 (original comment) and another

// ==============================================================================
// 0. 라이브러리 포함 및 상수/변수 정의
// ==============================================================================
// 디스플레이 관련 라이브러리
#include <Adafruit_GFX.h>      // ILI9341 그래픽 라이브러리
#include <Adafruit_ILI9341.h>  // ILI9341 드라이버 라이브러리
// 계산 관련 라이브러리
#include <ArduinoFFT.h>        // FFT 라이브러리 (THD 계산용)
#include <math.h>              // 수학 함수 (sqrt, sin, acos 등)

// 핀 정의 (Nano R4 기준)
#define VOLTAGE_PIN    A3       // ZMPT101B 출력 (OPAMP 연결 가정)
#define CURRENT_PIN    A4       // ACS712 출력
#define RELAY_PIN      13       // 릴레이 모듈 제어 핀 (예시)
#define TFT_CS         10       // Chip Select (D10)
#define TFT_DC         9        // Data/Command (D9)
#define TFT_RST        8        // Reset (D8)

// ILI9341 객체 선언
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
// FFT 객체 선언
ArduinoFFT FFT = ArduinoFFT();

// ADC 및 Timer 설정 상수
#define SAMPLE_PERIOD_MS    1      // 1ms (1kHz 샘플링 주파수)
#define SAMPLES_PER_CALC    167    // 약 10주기 (RMS 계산 주기)
#define FFT_SAMPLES         256    // FFT 샘플 수 (2^N)
#define MAX_CURRENT_LIMIT   7.0    // 릴레이 보호 전류 (A)

// 교정 및 오프셋 계수 (14-bit ADC 기준)
const int V_OFFSET_ADC = 8192; // 2.5V (DC 오프셋)
const int I_OFFSET_ADC = 8192; // 2.5V (DC 오프셋)
const float K_V_REAL     = 0.0431; // V_RMS/ADC_RMS 변환 계수
const float K_I_REAL     = 0.0305; // A_RMS/ADC_RMS 변환 계수

// 누적 및 버퍼 변수 (ISR에서 접근하므로 volatile 선언)
volatile int sampleCount       = 0;
volatile float V_sq_sum        = 0.0;
volatile float I_sq_sum        = 0.0;
volatile float P_sum           = 0.0;
volatile bool calculationReady = false;

// FFT 버퍼 (V, I 파형 분석용 - 현재는 V만 사용)
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
int fft_count = 0;

// ==============================================================================
// 1. 함수 프로토타입 선언 및 Timer ISR
// ==============================================================================
void setup_timer_interrupt(); // 타이머 인터럽트 설정 함수 (호출 예정)
void setup_display();
void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD);
void relay_protection_check(float current);
float calculate_THD();

/**
 * @brief 타이머 인터럽트 서비스 루틴 (ISR). 1ms 주기로 실행.
 */
void timer_ISR_routine() {
  if (calculationReady) return;

  // 1. ADC 동시 샘플링 (실제 Nano R4에서 동시성을 보장하려면 추가 설정 필요)
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);

  // 2. DC 오프셋 제거 (AC 성분 추출)
  float V_ac = (float)(V_raw - V_OFFSET_ADC);
  float I_ac = (float)(I_raw - I_OFFSET_ADC);

  // 3. RMS 및 유효 전력(P) 계산 누적
  V_sq_sum += V_ac * V_ac;
  I_sq_sum += I_ac * I_ac;
  P_sum    += V_ac * I_ac;

  // 4. FFT 버퍼 저장 (FFT_SAMPLES 만큼만)
  if (fft_count < FFT_SAMPLES) {
    vReal[fft_count] = (double)V_ac;
    vImag[fft_count] = 0; // 허수부는 0
  }
  fft_count++;
  sampleCount++;

  // 5. 샘플링 횟수 도달 확인
  if (sampleCount >= SAMPLES_PER_CALC) {
    calculationReady = true;
    // (여기에 Timer를 멈추는 코드를 추가하면 계산 시간 확보에 유리합니다.)
  }
}

// ==============================================================================
// 2. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  // 릴레이 초기 상태: 전원 인가 (NO 접점 닫힘 가정)
  digitalWrite(RELAY_PIN, HIGH);

  setup_display();
  // Nano R4 환경에 맞는 Timer 설정 함수 호출 (현재 주석 처리됨)
  // setup_timer_interrupt();

  Serial.println("System Initialized. Starting Power Measurement.");
}

// ==============================================================================
// 3. Main Loop - 계산 수행 및 결과 출력
// ==============================================================================
void loop() {
  if (calculationReady) {
    // 1. 변수 안전 복사 및 초기화
    // ISR에서 더 이상 접근하지 않도록 calculationReady를 먼저 false로 설정
    calculationReady = false;

    float temp_V_sq_sum  = V_sq_sum;
    float temp_I_sq_sum  = I_sq_sum;
    float temp_P_sum     = P_sum;
    int temp_sampleCount = sampleCount;

    // ISR 변수 초기화
    V_sq_sum = 0.0;
    I_sq_sum = 0.0;
    P_sum    = 0.0;
    sampleCount = 0;


    // 2. RMS 계산 및 실제 물리량 변환
    float V_rms_adc  = sqrt(temp_V_sq_sum / temp_sampleCount);
    float I_rms_adc  = sqrt(temp_I_sq_sum / temp_sampleCount);
    float V_rms_real = V_rms_adc * K_V_REAL; // 실제 RMS 전압 (V)
    float I_rms_real = I_rms_adc * K_I_REAL; // 실제 RMS 전류 (A)

    // 3. 유효 전력 (P) 계산
    float P_avg_real = (temp_P_sum / temp_sampleCount) * K_V_REAL * K_I_REAL;

    // 4. 피상 전력 (S), 역률 (PF), 무효 전력 (Q) 계산
    float S_apparent = V_rms_real * I_rms_real; // 피상 전력 |S| (VA)
    float PF = (S_apparent != 0.0) ? (P_avg_real / S_apparent) : 0.0; // 역률
    // 역률 값 범위 클램핑 (-1.0 ~ 1.0)
    if (PF > 1.0) PF = 1.0;
    if (PF < -1.0) PF = -1.0;
    // 무효 전력 (Q) 계산 (Q = S * sin(acos(PF)))
    float Q_reactive = S_apparent * sin(acos(PF));

    // 5. THD 계산
    float THD_value = calculate_THD();

    // 6. 결과 출력
    display_power_values(V_rms_real, I_rms_real, P_avg_real, Q_reactive, PF, THD_value);

    // 7. 릴레이 보호 로직 실행
    relay_protection_check(I_rms_real);

    // 8. FFT 카운터 초기화 (다음 측정 주기 대비)
    fft_count = 0;
  }
}

// ==============================================================================
// 4. 보조 함수 구현 (릴레이, 디스플레이, THD 계산)
// ==============================================================================

/**
 * @brief 디스플레이 (ILI9341) 초기화
 */
void setup_display() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.println("Digital Wattmeter Ready");
}

/**
 * @brief 전력 측정 값을 LCD 화면에 출력
 * @param V_rms RMS 전압 (V)
 * @param I_rms RMS 전류 (A)
 * @param P_real 유효 전력 (W)
 * @param Q_reactive 무효 전력 (VAR)
 * @param PF 역률
 * @param THD 전압의 총 고조파 왜곡률 (%)
 */
void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD) {
  // 배경색을 검은색으로 설정하여 깜빡임 제거
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);

  // V_RMS 출력
  tft.setCursor(10, 40); tft.print("V: "); tft.setTextColor(ILI9341_CYAN);
  tft.print(V_rms, 2); tft.println(" V  ");

  // I_RMS 출력
  tft.setCursor(10, 70); tft.print("I: "); tft.setTextColor(ILI9341_YELLOW);
  tft.print(I_rms, 2); tft.println(" A  ");

  // P_REAL 출력
  tft.setCursor(10, 100); tft.print("P: "); tft.setTextColor(ILI9341_GREEN);
  tft.print(P_real, 2); tft.println(" W  ");

  // Q_REACT 출력
  tft.setCursor(10, 130); tft.print("Q: "); tft.setTextColor(ILI9341_MAGENTA);
  tft.print(Q_reactive, 2); tft.println(" VAR");

  // PF 출력
  tft.setCursor(10, 160); tft.print("PF: "); tft.setTextColor(ILI9341_RED);
  tft.println(PF, 3);

  // THD 출력
  tft.setCursor(10, 190); tft.print("THD: "); tft.setTextColor(ILI9341_ORANGE);
  tft.print(THD, 2); tft.println(" %");
}

/**
 * @brief 과전류 보호 로직 확인 및 릴레이 동작
 * @param current 측정된 RMS 전류 값 (A)
 */
void relay_protection_check(float current) {
  if (current > MAX_CURRENT_LIMIT) {
    digitalWrite(RELAY_PIN, LOW); // 릴레이 동작 (NO 접점 개방)
    tft.setCursor(10, 220);
    tft.setTextColor(ILI9341_RED);
    tft.println("OVERCURRENT FAULT!");
  }
}

/**
 * @brief FFT를 사용하여 전압 파형의 총 고조파 왜곡률 (THD) 계산
 * @return THD 값 (%)
 */
float calculate_THD() {
  if (fft_count < FFT_SAMPLES) return 0.0; // 샘플 부족 시 종료

  // 1. FFT 수행 (vReal에는 실수부, vImag에는 허수부)
  FFT.Compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
  // 2. 복소수를 크기(Magnitude)로 변환
  FFT.ComplexToMagnitude(vReal, vImag, FFT_SAMPLES);

  // 3. 기본파 (60Hz) 주파수 인덱스 찾기
  // FFT 주파수 해상도 = 샘플링 주파수 / FFT 샘플 수 = 1000Hz / 256 = 3.90625 Hz/인덱스
  int fundamental_idx = (int)(60.0 / (1000.0 / FFT_SAMPLES) + 0.5); // 반올림을 위해 +0.5

  // 4. THD 계산 (THD = (고조파 RMS / 기본파 RMS) * 100%)
  // 여기서 vReal[i]는 크기(Magnitude)이므로, 제곱의 합의 제곱근이 RMS에 비례함
  float fundamental_mag = vReal[fundamental_idx];
  float harmonic_sq_sum = 0.0;

  // DC 성분 (인덱스 0)와 기본파를 제외한 모든 고조파 성분의 제곱 합산
  for (int i = 1; i < FFT_SAMPLES / 2; i++) {
    if (i != fundamental_idx) {
      harmonic_sq_sum += vReal[i] * vReal[i];
    }
  }

  // THD = (고조파 크기 제곱 합의 제곱근 / 기본파 크기) * 100%
  // 참고: 인덱스 0(DC 성분)은 THD 계산에서 일반적으로 제외됨.
  float THD = 0.0;
  if (fundamental_mag > 0.0) {
      THD = (sqrt(harmonic_sq_sum) / fundamental_mag) * 100.0;
  }

  return THD;
}

// ==============================================================================
// 5. Timer 인터럽트 설정 함수 (별도로 구현 필요)
// ==============================================================================

/**
 * @brief Nano R4 등 환경에 맞게 Timer 인터럽트를 설정하는 함수
 * (1ms 주기로 timer_ISR_routine()을 호출하도록 설정해야 함)
 * (현재 비어있음 - 사용자 환경에 맞게 구현 필요)
 */
void setup_timer_interrupt() {
  // **여기에 Timer 인터럽트 설정 코드를 추가해야 합니다.**
  // 예시: Timer/Counter Control Register 설정 및 인터럽트 활성화
}