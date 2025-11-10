/*
 * ==============================================================================
 * [최종 통합 코드] Wattmeter - 3화면 전환, 위상차, 통합 파형, 깜빡임 제거
 *
 * * 목적:
 * 1. 터치로 3개의 화면 (메인 전력, 위상차, 통합 파형)을 순환합니다.
 * 2. 메인 전력 화면: 실시간 RMS 및 전력 계산 (깜빡임 제거)
 * 3. 위상차 화면: 전압/전류 위상차 (각도) 및 Lead/Lag 표시 (깜빡임 제거)
 * 4. 통합 파형 화면: 전압(상단), 전류(하단) 파형을 동시에 표시합니다.
 *
 * * 핀 연결:
 * - VOLTAGE_PIN: A3 (ZMPT101B)
 * - CURRENT_PIN: A4 (ACS712)
 * - TFT_CS: 10, TFT_DC: 9, TFT_RST: 8
 * - TOUCH_CS: 7
 * ==============================================================================
 */

// --- 라이브러리 포함 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h> // 터치스크린 라이브러리
#include <math.h>

// --- 화면 상태 정의 ---
enum ScreenState {
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,  // 위상차 화면 추가
  SCREEN_COMBINED_WAVEFORM  // 통합 파형 화면
};
volatile ScreenState currentScreen = SCREEN_MAIN_POWER;
volatile bool screenNeedsRedraw = true; // 화면 다시 그리기 플래그

// --- 핀 정의 (센서) ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A4

// --- 핀 정의 (Display & Touch) ---
#define TFT_CS    10
#define TFT_DC    9
#define TFT_RST   8
#define TOUCH_CS  7

// --- ADC 오프셋 (14비트 Vcc/2) ---
#define V_OFFSET 8192
#define I_OFFSET 8192

// --- 보정 계수 (Calibration Factors) ---
// ZMPT101B (사용자 지정 값)
const float V_CALIB_RMS = 0.1706; 
// ACS712 (PPT 슬라이드 16 기준, 30A 모듈 추정)
const float I_CALIB_RMS = 0.0305;  

// --- RMS 계산 설정 (PPT 슬라이드 16 기준) ---
#define SAMPLES_PER_CALC 167 // 1kHz 샘플링 시 약 10주기 (167ms)
#define SAMPLE_PERIOD_US 1000 // 1ms (1kHz 샘플링)
const float FREQUENCY = 60.0; // 60Hz

// --- 터치스크린 교정 값 ---
#define TS_MINX 370
#define TS_MINY 450
#define TS_MAXX 3760
#define TS_MAXY 3670

// --- 객체 선언 ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);

// --- SPI 설정 (TFT/Touch 공유용) ---
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);

// --- 파형 디스플레이 설정 (세로 모드 240x320) ---
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// 통합 파형 화면 레이아웃
#define V_PLOT_Y_START 40
#define V_PLOT_Y_END 155
#define V_CENTER (V_PLOT_Y_START + ((V_PLOT_Y_END - V_PLOT_Y_START) / 2)) // 97

#define I_PLOT_Y_START 165
#define I_PLOT_Y_END 280
#define I_CENTER (I_PLOT_Y_START + ((I_PLOT_Y_END - I_PLOT_Y_START) / 2)) // 222

// --- 파형 스케일링 ---
const float V_PLOT_HEIGHT_HALF = (V_PLOT_Y_END - V_PLOT_Y_START) / 2.0;
const float I_PLOT_HEIGHT_HALF = (I_PLOT_Y_END - I_PLOT_Y_START) / 2.0;
// 1. 전압: 360V Peak 기준 (250V RMS * 1.414 = 353V)
const float VOLTS_TO_PIXELS_SCALE = V_PLOT_HEIGHT_HALF / 360.0;
// 2. 전류: 42.4A Peak 기준 (30A RMS * 1.414 = 42.4A)
const float AMPS_TO_PIXELS_SCALE = I_PLOT_HEIGHT_HALF / 42.4;

int current_x = 0;    // 현재 파형 X 좌표
int last_y_v = V_CENTER; // 이전 전압 Y 좌표
int last_y_i = I_CENTER; // 이전 전류 Y 좌표

// --- 실시간 계산값 저장을 위한 전역 변수 ---
float V_rms = 0.0;
float I_rms = 0.0;
float P_real = 0.0;
float Q_reactive = 0.0;
float S_apparent = 0.0;
float PF = 0.0;
float THD_value = 0.0; // THD는 FFT 구현이 필요하므로 0.0으로 고정
float phase_degrees = 0.0;
String lead_lag_status = "---";

// --- 깜빡임 제거를 위한 이전 값 저장 변수 ---
float prev_V_rms = 0.0, prev_I_rms = 0.0, prev_P_real = 0.0;
float prev_Q_reactive = 0.0, prev_PF = 0.0, prev_THD_value = 0.0;
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";

// ==============================================================================
// 1. Setup 함수 (병합)
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Wattmeter Booting...");

  // (중요!) Nano R4의 ADC 해상도를 14비트로 설정합니다.
  analogReadResolution(14);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  tft.begin();
  tft.setRotation(3); // 세로 모드 (240x320)
  
  ts.begin(SPI);
  ts.setRotation(3);
  
  Serial.println("TFT & Touch OK");
  
  screenNeedsRedraw = true; // 첫 화면을 그리도록 플래그 설정
}

// ==============================================================================
// 2. Main Loop (상태 머신 기반)
// ==============================================================================
void loop() {
  
  // 1. 터치 입력 확인 (매 루프 실행)
  checkTouchInput();

  // 2. 화면을 다시 그려야 하는 경우 (화면 전환 시 1회 실행)
  if (screenNeedsRedraw) {
    tft.fillScreen(ILI9341_BLACK); // 화면 전체 지우기
    screenNeedsRedraw = false; // 플래그 리셋
    
    // 파형 변수 초기화
    current_x = 0;
    last_y_v = V_CENTER;
    last_y_i = I_CENTER;

    // 그릴 화면의 정적 요소(제목, 그리드) 그리기
    switch(currentScreen) {
      case SCREEN_MAIN_POWER:
        displayMainScreenStatic(); // 메인 화면 제목/라벨 그리기
        break;
      case SCREEN_PHASE_DIFFERENCE:
        displayPhaseScreenStatic(); // 위상차 화면 제목/라벨 그리기
        break;
      case SCREEN_COMBINED_WAVEFORM:
        displayCombinedWaveformStatic(); // 통합 파형 화면 그리드/제목 그리기
        break;
    }
  }
  
  // 3. 현재 화면 상태에 따라 적절한 동적 작업 실행
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      runMainPowerCalculation(); // 실시간 계산 및 값 표시
      break;
    case SCREEN_PHASE_DIFFERENCE:
      runPhaseCalculation(); // 실시간 계산 및 위상차 값 표시
      break;
    case SCREEN_COMBINED_WAVEFORM:
      runCombinedWaveformLoop(); // 전압/전류 파형 그리기
      break;
  }
}

// ==============================================================================
// 3. 터치 입력 확인 함수 (3-state)
// ==============================================================================
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();

  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    // 화면 상태 3-way 토글
    if (currentScreen == SCREEN_MAIN_POWER) {
       currentScreen = SCREEN_PHASE_DIFFERENCE;
       Serial.println("Screen -> PHASE");
    } 
    else if (currentScreen == SCREEN_PHASE_DIFFERENCE) { 
       currentScreen = SCREEN_COMBINED_WAVEFORM;
       Serial.println("Screen -> COMBINED WAVEFORM"); 
    }
    else if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
       currentScreen = SCREEN_MAIN_POWER;
       Serial.println("Screen -> MAIN POWER");
    }
    
    screenNeedsRedraw = true; // 화면을 다시 그려야 함을 표시
    delay(100); // 디바운싱
  }
}

// ==============================================================================
// 4. 깜빡임 없는 텍스트/값 출력 헬퍼 함수
// ==============================================================================
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  char buffer[20];
  
  // 1. 이전 값을 검은색으로 덮어써서 지운다.
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer);
  tft.print(buffer);
  tft.print(unit);

  // 2. 새 값을 지정된 색상으로 그린다.
  tft.setTextColor(color);
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer);
  tft.print(unit);
}
// String용 헬퍼 함수
void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(x, y);
  tft.print(prev_value);

  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(value);
}


// ==============================================================================
// 5. 메인 전력 화면 그리기 (정적 요소)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  
  tft.setTextSize(2);
  tft.setCursor(10, 40);
  tft.setTextColor(ILI9341_CYAN);
  tft.println("V:");
  
  tft.setCursor(10, 70);
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("I:");
  
  tft.setCursor(10, 100);
  tft.setTextColor(ILI9341_GREEN);
  tft.println("P:");

  tft.setCursor(10, 130);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.println("Q:");

  tft.setCursor(10, 160);
  tft.setTextColor(ILI9341_RED);
  tft.println("PF:");
  
  tft.setCursor(10, 190);
  tft.setTextColor(ILI9341_ORANGE);
  tft.println("THD:");

  tft.setCursor(10, 280);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Tap screen to switch view");
  
  // 이전 값들을 현재 값으로 초기화 (깜빡임 방지)
  prev_V_rms = V_rms;
  prev_I_rms = I_rms;
  prev_P_real = P_real;
  prev_Q_reactive = Q_reactive;
  prev_PF = PF;
  prev_THD_value = THD_value;
}

// ==============================================================================
// 6. 메인 전력 화면 "값" 업데이트 함수 (깜빡임 제거됨)
// ==============================================================================
void displayMainScreenValues() {
  tft.setTextSize(2);
  
  printTFTValue(45, 40,  V_rms,     prev_V_rms,     1, ILI9341_CYAN,    " V");
  printTFTValue(45, 70,  I_rms,     prev_I_rms,     2, ILI9341_YELLOW,  " A");
  printTFTValue(45, 100, P_real,    prev_P_real,    1, ILI9341_GREEN,   " W");
  printTFTValue(45, 130, Q_reactive,prev_Q_reactive,1, ILI9341_MAGENTA, " VAR");
  printTFTValue(45, 160, PF,        prev_PF,        2, ILI9341_RED,     "");
  printTFTValue(60, 190, THD_value, prev_THD_value, 1, ILI9341_ORANGE,  " %");

  // 현재 값을 이전 값으로 저장
  prev_V_rms = V_rms;
  prev_I_rms = I_rms;
  prev_P_real = P_real;
  prev_Q_reactive = Q_reactive;
  prev_PF = PF;
  prev_THD_value = THD_value;
}

// ==============================================================================
// 7. 위상차 화면 그리기 (정적 요소)
// ==============================================================================
void displayPhaseScreenStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("PHASE DIFFERENCE");

  tft.setTextSize(2);
  tft.setCursor(10, 70);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Phase:");

  tft.setCursor(10, 130);
  tft.setTextColor(ILI9341_WHITE);
  tft.println("Status:");
  
  tft.setCursor(10, 280);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Tap screen to switch view");
  
  // 이전 값 초기화
  prev_phase_degrees = phase_degrees;
  prev_lead_lag_status = lead_lag_status;
}

// ==============================================================================
// 8. 위상차 화면 "값" 업데이트 함수 (깜빡임 제거됨)
// ==============================================================================
void displayPhaseScreenValues() {
  tft.setTextSize(3);
  printTFTValue(10, 100, phase_degrees, prev_phase_degrees, 1, ILI9341_CYAN, " deg");
  printTFTValue(10, 160, lead_lag_status, prev_lead_lag_status, ILI9341_YELLOW);

  // 현재 값을 이전 값으로 저장
  prev_phase_degrees = phase_degrees;
  prev_lead_lag_status = lead_lag_status;
}

// ==============================================================================
// 9. 통합 파형 화면 그리기 (정적 요소)
// ==============================================================================
void displayCombinedWaveformStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.println("V/I WAVEFORM (Live)");
  
  tft.setTextSize(1);
  
  // --- 전압 영역 ---
  tft.drawFastHLine(0, V_CENTER, SCREEN_WIDTH, ILI9341_WHITE); // 0V선
  tft.setCursor(5, V_PLOT_Y_START);
  tft.setTextColor(ILI9341_CYAN);
  tft.print("+360V");
  tft.setCursor(5, V_PLOT_Y_END - 5);
  tft.print("-360V");

  // --- 전류 영역 ---
  tft.drawFastHLine(0, I_CENTER, SCREEN_WIDTH, ILI9341_WHITE); // 0A선
  tft.setCursor(5, I_PLOT_Y_START);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("+42A");
  tft.setCursor(5, I_PLOT_Y_END - 5);
  tft.print("-42A");
  
  tft.setCursor(10, 280);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Tap screen to switch view");
}


// ==============================================================================
// 10. 실시간 전력 계산 (메인/위상차 화면 공용)
// ==============================================================================
void calculatePowerMetrics() {
  
  unsigned long V_sq_sum = 0;
  unsigned long I_sq_sum = 0;
  long P_sum = 0;

  // 위상차 계산용 변수
  int V_ac_bits_prev = 0;
  int I_ac_bits_prev = 0;
  long time_V_cross = -1;
  long time_I_cross = -1;
  bool found_V_cross = false;
  bool found_I_cross = false;
  
  unsigned long startTime = micros();

  for (int i = 0; i < SAMPLES_PER_CALC; i++) {
    // 1. ADC 값 동시 샘플링
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);

    // 2. AC 성분 추출 (DC 오프셋 제거)
    int V_ac_bits = V_raw - V_OFFSET;
    int I_ac_bits = I_raw - I_OFFSET;

    // 3. RMS 및 유효전력 계산을 위한 값 누적
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits;
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits;
    P_sum += (long)V_ac_bits * I_ac_bits;

    // 4. 위상차 계산을 위한 제로 크로싱 감지 (상승 엣지)
    // 한 주기(약 16.7ms) 내에서만 찾도록 함
    if (i < (SAMPLE_PERIOD_US * 1000 / FREQUENCY / SAMPLE_PERIOD_US)) {
      if (!found_V_cross && V_ac_bits_prev < 0 && V_ac_bits >= 0) {
        time_V_cross = micros();
        found_V_cross = true;
      }
      if (!found_I_cross && I_ac_bits_prev < 0 && I_ac_bits >= 0) {
        time_I_cross = micros();
        found_I_cross = true;
      }
    }
    V_ac_bits_prev = V_ac_bits;
    I_ac_bits_prev = I_ac_bits;

    // 5. 1ms 샘플링 주기 맞추기 (정확한 타이밍 제어)
    while(micros() - startTime < (i + 1) * SAMPLE_PERIOD_US);

    // 6. 계산 중 터치 입력 확인 (20ms 마다)
    if (i % 20 == 0) {
      checkTouchInput();
      if (screenNeedsRedraw) return; // 화면 전환 시 계산 중단
    }
  }

  // 7. RMS 및 평균 전력 계산
  float V_rms_adc = sqrt((float)V_sq_sum / SAMPLES_PER_CALC);
  float I_rms_adc = sqrt((float)I_sq_sum / SAMPLES_PER_CALC);
  float P_avg_adc = (float)P_sum / SAMPLES_PER_CALC;

  // 8. ADC 값을 실제 물리량으로 변환
  V_rms = V_rms_adc * V_CALIB_RMS;
  I_rms = I_rms_adc * I_CALIB_RMS;
  P_real = P_avg_adc * V_CALIB_RMS * I_CALIB_RMS;
  S_apparent = V_rms * I_rms;
  
  // 9. 역률 (PF)
  if (S_apparent == 0) {
    PF = 0;
  } else {
    PF = P_real / S_apparent;
    if (PF > 1.0) PF = 1.0;
    if (PF < -1.0) PF = -1.0; 
  }
  
  // 10. 무효전력 (Q)
  Q_reactive = sqrt(max(0.0, S_apparent * S_apparent - P_real * P_real)); // 0 미만 방지

  // 11. 위상차 (Phase) 계산
  if (found_V_cross && found_I_cross) {
    long time_diff = time_I_cross - time_V_cross;
    float period_us = 1000000.0 / FREQUENCY;
    
    // 시간 차이를 각도 차이로 변환
    phase_degrees = fmod(((float)time_diff / period_us) * 360.0, 360.0);

    if (phase_degrees > 180.0) {
        phase_degrees -= 360.0;
    } else if (phase_degrees < -180.0) {
        phase_degrees += 360.0;
    }

    if (phase_degrees > 0.0) {
        lead_lag_status = "Lag"; // 전류가 전압보다 늦음 (지상)
    } else {
        lead_lag_status = "Lead"; // 전류가 전압보다 빠름 (진상)
        phase_degrees = -phase_degrees; // 각도는 양수로 표시
    }
    
    // PF를 이용한 각도 보정 (더 정확함)
    phase_degrees = acos(PF) * (180.0 / M_PI);
    
  } else {
    phase_degrees = acos(PF) * (180.0 / M_PI);
    lead_lag_status = (PF == 1.0) ? "---" : "Lag/Lead"; // PF로만 판단
  }
  
  if (I_rms < 0.05) { // 전류가 매우 낮으면 위상차 무의미
    phase_degrees = 0.0;
    lead_lag_status = "---";
  }

  // 12. THD (임시값)
  THD_value = 0.0; 
}

// ==============================================================================
// 11. 메인 화면 실행 함수
// ==============================================================================
void runMainPowerCalculation() {
  calculatePowerMetrics(); // 167ms 소요
  if (screenNeedsRedraw) return; // 터치로 화면 전환 시 그리기 중단
  displayMainScreenValues();
}

// ==============================================================================
// 12. 위상차 화면 실행 함수
// ==============================================================================
void runPhaseCalculation() {
  calculatePowerMetrics(); // 167ms 소요
  if (screenNeedsRedraw) return; // 터치로 화면 전환 시 그리기 중단
  displayPhaseScreenValues();
}

// ==============================================================================
// 13. 통합 파형 그리기 루프 함수
// ==============================================================================
void runCombinedWaveformLoop() {
  // 1. 동시 샘플링
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);
  int V_ac_bits = V_raw - V_OFFSET;
  int I_ac_bits = I_raw - I_OFFSET;
  
  // 2. 순시값 계산 (RMS 계수 사용)
  float V_mains_instant = V_ac_bits * V_CALIB_RMS; 
  float I_mains_instant = I_ac_bits * I_CALIB_RMS;
  
  // 3. Y 좌표 계산
  int y_pos_v = V_CENTER - (int)(V_mains_instant * VOLTS_TO_PIXELS_SCALE);
  y_pos_v = constrain(y_pos_v, V_PLOT_Y_START, V_PLOT_Y_END);
  
  int y_pos_i = I_CENTER - (int)(I_mains_instant * AMPS_TO_PIXELS_SCALE);
  y_pos_i = constrain(y_pos_i, I_PLOT_Y_START, I_PLOT_Y_END);

  // 4. 파형 그리기 (전압/전류)
  tft.drawLine(current_x - 1, last_y_v, current_x, y_pos_v, ILI9341_CYAN);
  tft.drawLine(current_x - 1, last_y_i, current_x, y_pos_i, ILI9341_YELLOW);
  last_y_v = y_pos_v;
  last_y_i = y_pos_i;
  
  // 5. 다음 X 좌표 지우기 (깜빡임 최소화)
  tft.drawFastVLine(current_x + 1, V_PLOT_Y_START, (V_PLOT_Y_END - V_PLOT_Y_START), ILI9341_BLACK);
  tft.drawFastVLine(current_x + 1, I_PLOT_Y_START, (I_PLOT_Y_END - I_PLOT_Y_START), ILI9341_BLACK);
  
  // 6. X 좌표 증가 및 래핑
  current_x++;
  if (current_x >= SCREEN_WIDTH) {
    current_x = 0;
    last_y_v = y_pos_v;
    last_y_i = y_pos_i;
  }
  
  delayMicroseconds(40); // 파형 표시 속도 조절
}