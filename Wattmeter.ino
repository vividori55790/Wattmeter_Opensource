/*
 * ==============================================================================
 * 0. 라이브러리 포함 및 상수/변수 정의 (Arduino Nano R4 - 실제 동작용)
 * ==============================================================================
 * [Ver 2.0 - 핀 수정]
 * 1. D13 (SPI SCK)과 릴레이 핀의 충돌을 해결하기 위해 메인 릴레이 핀을 D7로 변경.
 *
 * [특징]
 * 1. MCU: Arduino Nano R4 (Renesas RA4M1)
 * 2. Timer: FspTimer.h (Nano R4 호환)
 * 3. Display: Adafruit ILI9341 (TFT)
 * 4. ADC: 14-bit 해상도 (analogReadResolution(14))
 * 5. 데이터 소스: 실제 A3(전압), A4(전류) 핀에서 analogRead() 수행
 * 6. FFT Smaples: 256 (Nano R4 메모리에 최적화)
 * 7. 모든 부가기능 포함 (Q, 파형표시, 타이머, Protection, THD)
 */

// --- 필수 라이브러리 ---
#include <SPI.h>
#include <Adafruit_GFX.h>      // ILI9341 그래픽 라이브러리
#include <Adafruit_ILI9341.h> // ILI9341 드라이버 라이브러리
#include <FspTimer.h>           // Nano R4 (RA4M1)용 타이머 라이브러리
#include <arduinoFFT.h>         // FFT 라이브러리 (THD 계산용)
#include <math.h>               // 수학 함수 (sqrt, sin, acos 등)

// --- 시리얼 모드 선택 ---
// "SERIAL_MODE_PLOTTER"를 1로 설정하면 시리얼 '플로터'용 파형 데이터를 출력합니다. (부가기능 2. 파형 표시)
// "SERIAL_MODE_PLOTTER"를 0으로 설정하면 'TFT' 및 '시리얼 모니터'용 전력 계산 리포트를 출력합니다.
#define SERIAL_MODE_PLOTTER 0

// --- 핀 정의 (Nano R4 기준 - 5조 발표자료) ---
#define VOLTAGE_PIN A3 // ZMPT101B 출력 (OPAMP 경유)
#define CURRENT_PIN A4 // ACS712 출력

// --- [핀 수정] ---
// D13은 SPI의 SCK로 사용해야 하므로 릴레이 핀을 D7로 변경합니다.
#define RELAY_PIN 7    // 릴레이 모듈 제어 핀 (필수 Protection)

#define TFT_CS 10    // Chip Select (D10)
#define TFT_DC 9     // Data/Command (D9)
#define TFT_RST 8    // Reset (D8)
// SPI 핀 (D11, D13)은 <SPI.h> 라이브러리에서 자동으로 관리됩니다.

// --- [피드백 8조] 부하 제어: PFC(역률개선) 릴레이 핀 ---
#define PFC_RELAY_PIN 12 // (D12 핀 사용)

// --- ADC 및 Timer 설정 상수 ---
#define SAMPLE_PERIOD_MS 1   // 1ms (1kHz 샘플링 주파수)
#define SAMPLES_PER_CALC 167 // 약 10주기 (RMS 계산 주기 @ 60Hz)
#define FFT_SAMPLES 256      // FFT 샘플 수 (Nano R4 메모리에 최적화, 2^N) [cite: 1569-1571]
#define MAX_CURRENT_LIMIT 7.0 // 릴레이 보호 전류 (A) (1500W/220V + 마진)

// --- [피드백 5,6조] 과도 전류 대응 ---
#define OVERCURRENT_TRIP_CYCLES 3 // 이 횟수(약 0.5초)만큼 과전류 지속 시 차단

// --- [피드백 8조] 부하 제어 ---
#define TARGET_PF 0.90 // 목표 역률 (이 값 미만일 때 PFC 릴레이 동작)

// --- !!! [중요] 교정 및 오프셋 계수 (Nano R4 14-bit ADC 기준) !!! ---
// 이 값은 실제 회로 연결 후 반드시 재교정(Calibration)해야 합니다.
const int V_OFFSET_ADC = 8192; // 2.5V (DC 오프셋 @ 14-bit)
const int I_OFFSET_ADC = 8192; // 2.5V (DC 오프셋 @ 14-bit)
const float K_V_REAL = 0.0431; // V_RMS/ADC_RMS 변환 계수 (발표자료 값) [cite: 1569-1571]
const float K_I_REAL = 0.0305; // A_RMS/ADC_RMS 변환 계수 (발표자료 값) [cite: 1569-1571]

// --- 객체 선언 ---
FspTimer gpt_timer; // Nano R4 타이머 객체
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
ArduinoFFT<double> FFT;

// --- 누적 및 버퍼 변수 (ISR에서 접근하므로 volatile) ---
volatile int sampleCount = 0;
volatile float V_sq_sum = 0.0;
volatile float I_sq_sum = 0.0;
volatile float P_sum = 0.0;
volatile bool calculationReady = false;

// --- FFT 버퍼 ---
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
int fft_count = 0;

// --- [피드백 5,6조] 과도 전류 카운터 ---
int overcurrent_counter = 0;

// --- [부가기능 3. 타이머] 변수 ---
volatile unsigned long timerEndTime = 0; // 타이머 종료 시각 (0 = 비활성)
volatile bool timerActive = false; // 타이머 동작 여부
bool timerExpired = false; // 타이머 만료 플래그

// --- 시리얼 입력 버퍼 ---
char serialBuffer[20];
int bufferIndex = 0;

// ==============================================================================
// 1. 함수 프로토타입 선언 (Function Prototypes)
// ==============================================================================
void setup_timer_interrupt();
void timer_ISR_routine(); // 실제 ADC 샘플링 ISR
void setup_display();
void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD);
void relay_protection_check(float current);
void load_control_check(float P_real, float Q_reactive, float PF);
float calculate_THD();
void check_serial_input();
void check_timer_expire(); // [부가기능 3] 타이머 만료 체크 함수

// ==============================================================================
// 2. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);

  // --- ADC 해상도 설정 (Nano R4: 14-bit) ---
  analogReadResolution(14); // 0-16383 범위 [cite: 1530-1532, 1630-1632, 1754-1756]

  // --- 릴레이 핀 초기화 ---
  pinMode(RELAY_PIN, OUTPUT); // D7
  digitalWrite(RELAY_PIN, HIGH); // 릴레이 초기 상태: HIGH (안전, NO 접점 가정)
  
  // --- [피드백 8조] PFC 릴레이 핀 초기화 ---
  pinMode(PFC_RELAY_PIN, OUTPUT); // D12
  digitalWrite(PFC_RELAY_PIN, LOW); // PFC 릴레이 초기 상태: LOW (OFF)

  // --- 디스플레이 설정 ---
  // (참고: TFT_CS, TFT_DC, TFT_RST는 객체 생성 시 자동 설정되지만,
  // SPI 핀(D11, D13)은 tft.begin() 호출 시 라이브러리가 자동 관리합니다.)
  setup_display();

  // --- 타이머 인터럽트 설정 (Nano R4) ---
  setup_timer_interrupt();

#if (SERIAL_MODE_PLOTTER == 0)
  Serial.println("--- Digital Wattmeter Initialized (Nano R4) ---");
  Serial.println("--- [REAL HARDWARE MODE - V2 / Pin Fix] ---");
  Serial.println("--- Main Relay moved to D7 ---");
  Serial.println("--- 명령을 입력하세요 ---");
  Serial.println("T,분 (예: T,10) - 타이머 설정");
  Serial.println("C - 타이머 취소");
  Serial.println("=========================================");
  tft.println("System Initialized. (V2)");
  tft.println("Relay -> D7");
  
  // (중요) 실제 교정 안내
  Serial.println("!! 중요: V_OFFSET, I_OFFSET, K_V_REAL, K_I_REAL 값을 반드시 교정하세요.");
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(10, 220);
  tft.println("WARNING: CALIBRATION REQUIRED!");
  delay(2000);

#else
  Serial.println("--- Serial Plotter Mode ---");
  Serial.println("V_inst, I_inst");
#endif
}

// ==============================================================================
// 3. Main Loop
// ==============================================================================
void loop() {
  // 시리얼 입력 처리 (비차단 방식)
#if (SERIAL_MODE_PLOTTER == 0)
  check_serial_input();
#endif

  // [부가기능 3] 타이머 만료 여부 매번 확인
  check_timer_expire();

  if (calculationReady) {
    // 1. 변수 안전 복사 및 초기화
    calculationReady = false;
    // (중요) 인터럽트 비활성화 중에 변수를 복사하여 데이터 손상 방지
    noInterrupts();
    float temp_V_sq_sum = V_sq_sum;
    float temp_I_sq_sum = I_sq_sum;
    float temp_P_sum = P_sum;
    int temp_sampleCount = sampleCount;
    
    // ISR에서 다시 누적을 시작할 수 있도록 즉시 초기화
    V_sq_sum = 0.0;
    I_sq_sum = 0.0;
    P_sum = 0.0;
    sampleCount = 0;
    interrupts(); // 인터럽트 재활성화

    // 2. RMS 계산 및 실제 물리량 변환 (필수기능: V, I)
    float V_rms_adc = sqrt(temp_V_sq_sum / temp_sampleCount);
    float I_rms_adc = sqrt(temp_I_sq_sum / temp_sampleCount);
    float V_rms_real = V_rms_adc * K_V_REAL;
    float I_rms_real = I_rms_adc * K_I_REAL;

    // 3. 유효 전력 (P) 계산 (필수기능: P)
    float P_avg_real = (temp_P_sum / temp_sampleCount) * K_V_REAL * K_I_REAL;

    // 4. 피상 전력 (S), 역률 (PF), 무효 전력 (Q) 계산 (부가기능 1. 무효전력)
    float S_apparent = V_rms_real * I_rms_real;
    float PF = (S_apparent != 0.0) ? (P_avg_real / S_apparent) : 0.0;
    
    if (PF > 1.0) PF = 1.0;
    if (PF < -1.0) PF = -1.0;
    
    float Q_reactive = S_apparent * sin(acos(PF));

    // 5. THD 계산 (부가기능 5. THD)
    float THD_value = calculate_THD(); // 윈도잉 기능이 내장됨

#if (SERIAL_MODE_PLOTTER == 0)
    // 6. 결과 출력 (TFT 및 시리얼 모니터)
    display_power_values(V_rms_real, I_rms_real, P_avg_real, Q_reactive, PF, THD_value);
    
    // 7. 릴레이 보호 로직 실행 (부가기능 4. Protection)
    relay_protection_check(I_rms_real);
    
    // 8. [피드백 8조] 부하 제어 로직 실행
    load_control_check(P_avg_real, Q_reactive, PF);
#endif

    // 9. FFT 카운터 초기화
    fft_count = 0;
  }
}

// ==============================================================================
// 4. ISR 및 보조 함수 (Actual Hardware)
// ==============================================================================

/**
 * @brief Nano R4용 FspTimer 설정
 */
void setup_timer_interrupt() {
  // gpt_timer (FspTimer 객체) 설정
  gpt_timer.begin(TIMER_MODE_PERIODIC, GPT_TIMER_CH0, (SAMPLE_PERIOD_MS * 1000), MICROSECONDS);
  // 인터럽트 콜백 함수 지정
  gpt_timer.setCallback(timer_ISR_routine);
  // 타이머 시작
  gpt_timer.start();
}

/**
 * @brief 1ms 타이머 인터럽트 서비스 루틴 (실제 ADC 샘플링용)
 */
void timer_ISR_routine() {
  if (calculationReady && SERIAL_MODE_PLOTTER == 0) return;

  // 1. ADC 동기 샘플링 (14-bit)
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);

  // 2. DC 옵셋 제거 (AC 성분 추출)
  float V_ac = (float)(V_raw - V_OFFSET_ADC);
  float I_ac = (float)(I_raw - I_OFFSET_ADC);

#if (SERIAL_MODE_PLOTTER == 1)
  // --- 플로터 모드 --- (부가기능 2. 파형 표시)
  Serial.print(V_ac * K_V_REAL * 1.41421); // V_inst (추정치)
  Serial.print(",");
  Serial.println(I_ac * K_I_REAL * 1.41421); // I_inst (추정치)

#else
  // --- 전력 계산 모드 ---
  // 3. RMS 및 유효 전력(P) 계산 누적
  V_sq_sum += V_ac * V_ac;
  I_sq_sum += I_ac * I_ac;
  P_sum    += V_ac * I_ac;

  // 4. FFT 버퍼 저장 (부가기능 5. THD)
  if (fft_count < FFT_SAMPLES) {
    vReal[fft_count] = (double)V_ac;
    vImag[fft_count] = 0; // 허수부는 0
  }
  fft_count++;
  sampleCount++;

  // 5. 샘플링 횟수 도달 확인
  if (sampleCount >= SAMPLES_PER_CALC) {
    calculationReady = true;
  }
#endif
}

/**
 * @brief [부가기능 3] 타이머 만료 체크
 * Arduino의 millis() 함수를 기준으로 타이머가 만료되었는지 확인
 */
void check_timer_expire() {
  if (timerActive && millis() >= timerEndTime) {
    timerActive = false;
    timerExpired = true; // 만료 플래그 설정
    
    if (SERIAL_MODE_PLOTTER == 0) {
      Serial.println("******************************************");
      Serial.println("!! TIMER EXPIRED !!");
      Serial.println("******************************************");
    }
  }
}

/**
 * @brief 시리얼 모니터로 사용자 입력(T, C) 받기
 */
void check_serial_input() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { // 입력 완료 (Enter)
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0'; // 문자열 종료
        
        // --- [부가기능 3] 타이머 설정 명령 (T,분) ---
        if (serialBuffer[0] == 'T' || serialBuffer[0] == 't') {
          char* token = strtok(serialBuffer, ",");
          token = strtok(NULL, ","); // "분" 값 가져오기
          if (token != NULL) {
            int minutes = atoi(token);
            if (minutes > 0) {
              unsigned long durationMs = (unsigned long)minutes * 60 * 1000;
              timerEndTime = millis() + durationMs; // 현재 시간 기준
              timerActive = true;
              timerExpired = false; // 새 타이머 시작 시 만료 플래그 리셋
              
              Serial.print("\n[타이머 설정] ");
              Serial.print(minutes);
              Serial.println("분 타이머 시작.");
            } else {
              Serial.println("오류: 유효하지 않은 시간입니다.");
            }
          }
        } 
        // --- [부가기능 3] 타이머 취소 명령 (C) ---
        else if (serialBuffer[0] == 'C' || serialBuffer[0] == 'c') {
          timerActive = false;
          timerExpired = false;
          timerEndTime = 0;
          Serial.println("\n[타이머 취소] 타이머가 비활성화되었습니다.");
        } else {
          Serial.println("오류: 알 수 없는 명령입니다. (T,분 또는 C 입력)");
        }
        
        bufferIndex = 0; // 버퍼 초기화
      }
    } else if (bufferIndex < 19) {
      serialBuffer[bufferIndex++] = c;
    }
  }
}

/**
 * @brief ILI9341 디스플레이 초기 설정
 */
void setup_display() {
  tft.begin();
  tft.setRotation(3); // 가로 모드 (필요에 따라 1 또는 3)
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.println("Digital Wattmeter Ready");
}

/**
 * @brief 전력 측정 값을 시리얼 모니터 및 TFT LCD에 출력
 */
void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD) {
  // --- 1. 시리얼 모니터 출력 ---
  Serial.println("--- Measurement Results ---");
  Serial.print("  V_rms: \t"); Serial.print(V_rms, 2); Serial.println(" V");
  Serial.print("  I_rms: \t"); Serial.print(I_rms, 3); Serial.println(" A");
  Serial.print("  P_real: \t"); Serial.print(P_real, 2); Serial.println(" W");
  Serial.print("  Q_react: \t"); Serial.print(Q_reactive, 2); Serial.println(" VAR"); // 부가기능 1
  Serial.print("  PF: \t\t"); Serial.println(PF, 3);
  Serial.print("  THD_V: \t"); Serial.print(THD, 2); Serial.println(" % (Hann Window)"); // 부가기능 5
  Serial.println("---------------------------");

  // --- 2. TFT LCD 출력 ---
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);

  // V_RMS 출력 (필수)
  tft.setCursor(10, 40); tft.setTextColor(ILI9341_WHITE); tft.print("V: ");
  tft.setTextColor(ILI9341_CYAN); tft.print(V_rms, 2); tft.println(" V");

  // I_RMS 출력 (필수)
  tft.setCursor(10, 70); tft.setTextColor(ILI9341_WHITE); tft.print("I: ");
  tft.setTextColor(ILI9341_YELLOW); tft.print(I_rms, 3); tft.println(" A");

  // P_REAL 출력 (필수)
  tft.setCursor(10, 100); tft.setTextColor(ILI9341_WHITE); tft.print("P: ");
  tft.setTextColor(ILI9341_GREEN); tft.print(P_real, 2); tft.println(" W");

  // Q_REACT 출력 (부가기능 1)
  tft.setCursor(10, 130); tft.setTextColor(ILI9341_WHITE); tft.print("Q: ");
  tft.setTextColor(ILI9341_MAGENTA); tft.print(Q_reactive, 2); tft.println(" VAR");

  // PF 출력
  tft.setCursor(10, 160); tft.setTextColor(ILI9341_WHITE); tft.print("PF: ");
  tft.setTextColor(ILI9341_RED); tft.println(PF, 3);

  // THD 출력 (부가기능 5)
  tft.setCursor(10, 190); tft.setTextColor(ILI9341_WHITE); tft.print("THD: ");
  tft.setTextColor(ILI9341_ORANGE); tft.print(THD, 2); tft.println(" %");
}

/**
 * @brief [부가기능 4. Protection] 과전류 보호 로직 (과도 전류 대응)
 */
void relay_protection_check(float current) {
  static bool protectionTripped = false; // 과전류로 인한 차단 상태 기억
  
  if (current > MAX_CURRENT_LIMIT) {
    // 과전류 감지, 카운터 증가
    overcurrent_counter++;
    
    // 시리얼 및 TFT에 경고 출력
    char buffer[50];
    sprintf(buffer, "WARN: OC Count: %d/%d", overcurrent_counter, OVERCURRENT_TRIP_CYCLES);
    Serial.println(buffer);
    
    tft.setCursor(10, 220);
    tft.setTextColor(ILI9341_ORANGE);
    tft.setTextSize(2);
    tft.println(buffer);

    // 카운터가 설정된 트립 횟수에 도달하면 (과전류가 지속되면) 릴레이 차단
    if (overcurrent_counter >= OVERCURRENT_TRIP_CYCLES) {
      protectionTripped = true; // 차단 상태 Latch
      digitalWrite(RELAY_PIN, LOW); // 릴레이 동작 (차단)
      Serial.println("******************************************");
      Serial.println("!! OVERCURRENT FAULT (Persistent) !!");
      Serial.print("!! RELAY (Pin D"); Serial.print(RELAY_PIN); Serial.println(") TRIPPED (OFF) !!");
      Serial.println("******************************************");

      // TFT에 영구적인 오류 메시지 표시
      tft.fillScreen(ILI9341_RED);
      tft.setCursor(40, 100);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(3);
      tft.println("OVERCURRENT FAULT");
      tft.setCursor(80, 140);
      tft.println("RELAY OFF");
    }
    
  } else {
    // 전류가 정상 범위일 때
    overcurrent_counter = 0;
    
    // (참고) 실제 제품에서는 과전류가 해소되어도 자동으로 복구되면 안 됨.
    // 여기서는 테스트 편의를 위해 자동으로 Latch 해제
    if (protectionTripped) {
      protectionTripped = false;
      Serial.println("...Overcurrent fault auto-reset.");
    }
    
    // 과전류 트립 상태가 아닐 때만 타이머 상태 확인
    if (!protectionTripped) {
      if (timerExpired) {
        // [부가기능 3] 타이머가 만료되었으면 릴레이 OFF
        digitalWrite(RELAY_PIN, LOW);
        Serial.print("RELAY (Pin D"); Serial.print(RELAY_PIN); Serial.println(") OFF - Timer Expired");
        
        // TFT에 타이머 만료 표시
        tft.setCursor(10, 220);
        tft.setTextColor(ILI9341_RED);
        tft.setTextSize(2);
        tft.println("TIMER EXPIRED - RELAY OFF");

      } else {
        // 정상 상태 (타이머가 동작 중이거나, 설정되지 않았음)
        digitalWrite(RELAY_PIN, HIGH); 
        Serial.print("RELAY (Pin D"); Serial.print(RELAY_PIN); Serial.println(") ON - Normal");
      }
    }
  }
}

/**
 * @brief [피드백 8조] 부하 자동 제어 (역률 개선) 로직
 */
void load_control_check(float P_real, float Q_reactive, float PF) {
  // 유효전력이 10W 이상이고(의미있는 부하), 역률이 목표치(0.9)보다 낮고, 
  // 무효전력이 0보다 큰(유도성 부하, 지상역률) 경우
  if (P_real > 10.0 && PF < TARGET_PF && Q_reactive > 0.0) {
    digitalWrite(PFC_RELAY_PIN, HIGH); // PFC 릴레이 ON (커패시터 투입)
    Serial.println("[LOAD_CONTROL] Low PF (Inductive) detected. PFC Relay (Pin 12) ON.");
  } else {
    digitalWrite(PFC_RELAY_PIN, LOW); // 그 외의 경우 PFC 릴레이 OFF
    Serial.println("[LOAD_CONTROL] PF OK. PFC Relay (Pin 12) OFF.");
  }
}


/**
 * @brief [부가기능 5. THD] FFT를 사용하여 전압 파형의 THD 계산
 * @return THD 값 (%)
 */
float calculate_THD() {
  if (fft_count < FFT_SAMPLES) return 0.0;

  // [피드백 7조] 스펙트럼 누설(Leakage) 방지를 위해 HANN 윈도잉 적용
  FFT.windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);

  // 1. FFT 수행
  FFT.compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD); 
  // 2. 복소수를 크기(Magnitude)로 변환
  FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLES); 

  // 3. 기본파 (60Hz) 주파수 인덱스 찾기
  // 샘플링 주파수 = 1000Hz, FFT 샘플 수 = 256
  // 해상도 = 1000 / 256 = 3.90625 Hz/인덱스
  int fundamental_idx = (int)(FREQUENCY / (1000.0 / FFT_SAMPLES) + 0.5); // 60 / 3.90625 ≈ 15.36 -> 15

  // 4. THD 계산
  float fundamental_mag = vReal[fundamental_idx];
  float harmonic_sq_sum = 0.0;

  // DC 성분(0)과 기본파(fundamental_idx)를 제외한 고조파 성분 제곱 합산
  for (int i = 1; i < FFT_SAMPLES / 2; i++) { // i=1 부터 시작 (DC 제외)
    if (i != fundamental_idx) {
      harmonic_sq_sum += vReal[i] * vReal[i];
    }
  }

  float THD = 0.0;
  if (fundamental_mag > 0.0) {
      THD = (sqrt(harmonic_sq_sum) / fundamental_mag) * 100.0;
  }
  
  return THD;
}