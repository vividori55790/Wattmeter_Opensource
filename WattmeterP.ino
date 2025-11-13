/*
 * ==============================================================================
 * [v25 - UI와 정밀 보정 로직 병합]
 * - "친구의 v21 코드" (홈 화면, 설정 메뉴, 버튼 UI)를 기반으로 함.
 * - "우리의 v24 코드" (부팅 시 1회 오프셋 보정) 로직을 이식함.
 *
 * 1. [v24] 'setup()' 함수에 "Calibrating ADC Offset..." 루프 추가.
 * - 부팅 시 3000회 샘플링으로 "오늘의 실제 0점"을 측정.
 * 2. [v24] 'V_ADC_MIDPOINT_ACTUAL' 등 실제 0점 저장용 전역 변수 추가.
 * 3. [v25] 'V_RMS_OFFSET_CORRECTION' 등 불필요한 땜질 처방 변수를 0.0으로 설정.
 * 4. [v25] 모든 계산 함수(calculatePowerMetrics, performFFT_and_CalcTHD 등)가
 * v21의 하드코딩된 '8192.0' 대신, setup()에서 측정한 'V_ADC_MIDPOINT_ACTUAL'을
 * 사용하도록 전면 수정.
 * 5. [v21] 홈 화면, 설정 메뉴, 실시간 값 변경 기능은 모두 유지.
 * ==============================================================================
 */


// --- 라이브러리 포함 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>

// --- [v20] 화면 상태 정의 (홈 화면 기반) ---
enum ScreenState {
  SCREEN_HOME,               // [v20] 홈 메뉴
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_THD,
  SCREEN_SETTINGS,           // [v20] 설정 메뉴
  SCREEN_SETTINGS_CALIB,     // [v20] 2차 깊이: 보정
  SCREEN_SETTINGS_PROTECT,   // [v20] 2차 깊이: 보호
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; // [v20] 부팅 시 홈 화면
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2          // [v21] (메인 전류)
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5
#define TFT_CS    10
#define TFT_DC    9
#define TFT_RST   8
#define TOUCH_CS  7
#define WIFI_TX_PIN 2
#define WIFI_RX_PIN 3

// --- [v20] 실시간 설정 변수 (기존 define/const 대체) ---
float VOLTAGE_THRESHOLD = 240.0; // 과전압 임계값
float V_CALIB_RMS = 0.1775;      // 전압 보정 계수
float I_CALIB_RMS = 0.0482;      // 전류 보정 계수 (모든 채널 공통)

// --- [v20] 설정 화면용 이전 값 (깜빡임 제거) ---
float prev_VOLTAGE_THRESHOLD = 240.0;
float prev_V_CALIB_RMS = 0.1775;
float prev_I_CALIB_RMS = 0.0482;

volatile bool warningActive = false;
String warningMessage = "";

// --- FFT 상수 ---
#define FFT_N 512
#define SAMPLING_FREQ_HZ 7680.0f
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ)
#define FUNDAMENTAL_BIN 4
#define MAX_HARMONIC 40

// --- [v25] v24의 부팅 보정 로직용 전역 변수 (초기값은 8192) ---
float V_ADC_MIDPOINT_ACTUAL = 8192.0;
float I_ADC_MIDPOINT_MAIN_ACTUAL = 8192.0;
float I_ADC_MIDPOINT_1_ACTUAL = 8192.0;
float I_ADC_MIDPOINT_2_ACTUAL = 8192.0;

// --- [v25] v24의 0점 보정 로직을 사용하므로, v21의 땜질 처방(Offset)은 0.0으로 설정 ---
const float V_RMS_OFFSET_CORRECTION = 0.0; 
const float I_RMS_OFFSET_CORRECTION = 0.0; 
// const float V_ADC_MIDPOINT = 8192.0;  // [v25] 삭제 (V_ADC_MIDPOINT_ACTUAL로 대체)
// const float I_ADC_MIDPOINT = 8192.0;  // [v25] 삭제 (I_ADC_MIDPOINT_MAIN_ACTUAL 등으로 대체)

#define SAMPLES_PER_CALC 334 
#define SAMPLE_PERIOD_US 1000
const float FREQUENCY = 60.0;

// --- 터치스크린 상수 ---
#define TS_MINX 370
#define TS_MINY 450
#define TS_MAXX 3760
#define TS_MAXY 3670

// --- 객체 생성 ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS); 
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);

// --- UI 상수 ---
#define SCREEN_WIDTH 320 
#define SCREEN_HEIGHT 240 

// --- 파형 화면 상수 ---
#define PLOT_X_START 30 
#define PLOT_X_END 290 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
#define WAVEFORM_SAMPLE_PERIOD_US 100
int last_frame_y_v[PLOT_WIDTH];
int last_frame_y_i[PLOT_WIDTH];
float prev_frame_V_peak = 0.0;
float prev_frame_I_peak = 0.0;

// [v21-FIX] 전역 변수 선언
float V_peak_dynamic = 350.0; 
float I_peak_dynamic = 1.0;   

// --- [v19] 페이저 다이어그램 상수 ---
#define PHASOR_CX 235
#define PHASOR_CY 130
#define PHASOR_RADIUS 75
#define PHASOR_MAX_CURRENT 10.0

// --- 전역 변수 (물리량) ---
float V_rms = 0.0;
float I_rms = 0.0; 
float I_rms_load1 = 0.0;
float I_rms_load2 = 0.0;
float P_real = 0.0; 
float Q_reactive = 0.0; 
float S_apparent = 0.0;
float PF = 0.0;
float THD_value = 0.0;

// --- 전역 변수 (위상) ---
float phase_degrees = 0.0;
String lead_lag_status = "---"; 
float phase_main_deg = 0.0;
float phase_load1_deg = 0.0;
float phase_load2_deg = 0.0;

// --- 전역 변수 (이전 값 - 화면 클리어용) ---
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";
float prev_phase_main_deg = 0.0;
float prev_phase_load1_deg = 0.0;
float prev_phase_load2_deg = 0.0;
int prev_v_x = PHASOR_CX, prev_v_y = PHASOR_CY;
int prev_im_x = PHASOR_CX, prev_im_y = PHASOR_CY;
int prev_i1_x = PHASOR_CX, prev_i1_y = PHASOR_CY;
int prev_i2_x = PHASOR_CX, prev_i2_y = PHASOR_CY;

// --- FFT 버퍼 ---
float32_t v_samples[FFT_N];
float32_t i_samples[FFT_N];
float32_t v_fft_output[FFT_N];
float32_t i_fft_output[FFT_N];
float32_t v_mags[FFT_N / 2];
float32_t i_mags[FFT_N / 2];
float32_t thd_v_value = 0.0;
float32_t thd_i_value = 0.0;
float32_t prev_thd_v = 0.0;
float32_t prev_thd_i = 0.0;
arm_rfft_fast_instance_f32 fft_inst_v;
arm_rfft_fast_instance_f32 fft_inst_i;

// --- 퍼지 로직 변수 ---
Fuzzy *fuzzy; 
FuzzyInput *totalCurrent; 
FuzzyInput *currentChangeRate; 
FuzzyOutput *shutdownLevel; 
FuzzySet* safeCurrent;
FuzzySet* warningCurrent; 
FuzzySet* dangerCurrent; 
FuzzySet* criticalCurrent; 
FuzzySet* stableChange;
FuzzySet* slowIncrease;
FuzzySet* suddenSurge;
FuzzySet* level0;
FuzzySet* level3;
FuzzySet* level6;
FuzzySet* level10; 
float last_I_rms = 0.0;
unsigned long lastFuzzyTime = 0; 

// --- 함수 프로토타입 ---
void updateYAxisLabels();
void buildFuzzySystem();
void runFuzzyLogic(); 
void controlRelays(float level);
void waitForVoltageZeroCross();
float calculatePhase(long time_diff, float period_us);
void drawBackButton(); // [v20]
void displayHomeScreenStatic(); // [v20]
void displaySettingsScreenStatic(); // [v20]
void displaySettingsCalibStatic(); // [v20]
void displaySettingsProtectStatic(); // [v20]
void runSettingsCalib(); // [v20]
void runSettingsProtect(); // [v20]
void displaySettingsCalibValues(); // [v20]
void displaySettingsProtectValues(); // [v20]
void displayMainScreenStatic();
void displayPhaseScreenStatic();
void drawWaveformGridAndLabels();
void displayTHDScreenStatic();
void displayWarningScreenStatic();
void runMainPowerCalculation();
void runPhaseCalculation();
void runCombinedWaveformLoop();
void runTHDCalculation();
void performFFT_and_CalcTHD();
void calculatePowerMetrics();


// ==============================================================================
// 1. Setup 함수 [v25 수정]
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Wattmeter v25 (UI + Precision Calibration) Booting..."); // [v25]
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  pinMode(RELAY_1_PIN, OUTPUT); 
  pinMode(RELAY_2_PIN, OUTPUT); 
  digitalWrite(RELAY_1_PIN, LOW); 
  digitalWrite(RELAY_2_PIN, LOW); 

  tft.begin();
  tft.setRotation(3); 
  
  ts.begin(SPI);
  ts.setRotation(3); 
  Serial.println("TFT & Touch OK");

  // --- [v25] v24의 부팅 보정 로직 이식 ---
  // (TFT/Touch 초기화 '이후', 퍼지/FFT '이전에' 실행)
  tft.fillScreen(ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.print("Calibrating ADC Offset...");
  Serial.println("Calibrating ADC Offset... Please wait.");

  unsigned long V_sum = 0;
  unsigned long I_main_sum = 0;
  unsigned long I_1_sum = 0;
  unsigned long I_2_sum = 0;
  const int CALIBRATION_SAMPLES = 3000; // [v24] 3000개 샘플 평균

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    V_sum += analogRead(VOLTAGE_PIN);
    I_main_sum += analogRead(CURRENT_PIN); // [v25] v21의 핀 이름(CURRENT_PIN) 사용
    I_1_sum += analogRead(CURRENT_PIN_LOAD1);
    I_2_sum += analogRead(CURRENT_PIN_LOAD2);
    delayMicroseconds(100); // 넉넉한 샘플링 간격
  }

  // [v25] "오늘의 0점"을 측정하여 전역 변수에 '고정'시킴
  V_ADC_MIDPOINT_ACTUAL = (float)V_sum / CALIBRATION_SAMPLES;
  I_ADC_MIDPOINT_MAIN_ACTUAL = (float)I_main_sum / CALIBRATION_SAMPLES;
  I_ADC_MIDPOINT_1_ACTUAL = (float)I_1_sum / CALIBRATION_SAMPLES;
  I_ADC_MIDPOINT_2_ACTUAL = (float)I_2_sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration Done. Using static offsets for this session.");
  Serial.print("V Midpoint (A3): "); Serial.println(V_ADC_MIDPOINT_ACTUAL, 4);
  Serial.print("I-Main Midpoint (A2): "); Serial.println(I_ADC_MIDPOINT_MAIN_ACTUAL, 4);
  Serial.print("I-L1 Midpoint (A4): "); Serial.println(I_ADC_MIDPOINT_1_ACTUAL, 4);
  Serial.print("I-L2 Midpoint (A5): "); Serial.println(I_ADC_MIDPOINT_2_ACTUAL, 4);
  // --- [v25] 보정 완료 ---
  
  Serial.print("Initial V_CALIB_RMS: "); Serial.println(V_CALIB_RMS);
  Serial.print("Initial I_CALIB_RMS: "); Serial.println(I_CALIB_RMS); 
  Serial.print("Initial VOLTAGE_THRESHOLD: "); Serial.println(VOLTAGE_THRESHOLD); 
  Serial.print("Using V_RMS_OFFSET_CORRECTION: "); Serial.println(V_RMS_OFFSET_CORRECTION);
  Serial.print("Using I_RMS_OFFSET_CORRECTION: "); Serial.println(I_RMS_OFFSET_CORRECTION);

  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1); 
  currentChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  
  buildFuzzySystem(); 
  Serial.println("Fuzzy Logic System Built (using Fuzzy.h)"); 
  lastFuzzyTime = millis(); 

  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N); 
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N); 
  Serial.println("CMSIS-DSP FFT Initialized.");

  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop (v21과 동일)
// ==============================================================================
void loop() {
  checkTouchInput(); // [v20] 터치 로직이 여기로 이동하여 항상 반응
  
  if (screenNeedsRedraw) { 
    tft.fillScreen(ILI9341_WHITE); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic();
    } else { 
      // [v20] 정적 UI 그리기
      switch(currentScreen) {
        case SCREEN_HOME:
          displayHomeScreenStatic();
          break;
        case SCREEN_MAIN_POWER:
          displayMainScreenStatic(); 
          break;
        case SCREEN_PHASE_DIFFERENCE:
          displayPhaseScreenStatic();
          break;
        case SCREEN_COMBINED_WAVEFORM:
          drawWaveformGridAndLabels();
          updateYAxisLabels();      
          for(int i=0; i<PLOT_WIDTH; i++) {
              last_frame_y_v[i] = PLOT_Y_CENTER; 
              last_frame_y_i[i] = PLOT_Y_CENTER; 
          }
          prev_frame_V_peak = 0.0;
          prev_frame_I_peak = 0.0;
          break;
        case SCREEN_THD:
          displayTHDScreenStatic(); 
          break;
        case SCREEN_SETTINGS:
          displaySettingsScreenStatic();
          break;
        case SCREEN_SETTINGS_CALIB:
          displaySettingsCalibStatic();
          prev_V_CALIB_RMS = -1.0; // [v20] 강제
          prev_I_CALIB_RMS = -1.0;
          break;
        case SCREEN_SETTINGS_PROTECT:
          displaySettingsProtectStatic();
          prev_VOLTAGE_THRESHOLD = -1.0; // [v20] 강제
          break;
        case SCREEN_WARNING: 
          break;
      }
    }
    screenNeedsRedraw = false; 
  }
  
  // 3. [v20] 동적 UI 업데이트 (계산이 필요한 화면만)
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      runMainPowerCalculation(); 
      break;
    case SCREEN_PHASE_DIFFERENCE:
      runPhaseCalculation();
      break;
    case SCREEN_COMBINED_WAVEFORM:
      waitForVoltageZeroCross();
      
      // 0점 대기 중 터치/경고 발생 시 루프 탈출 (checkTouchInput은 루프 맨 위에서 이미 호출됨)
      if (screenNeedsRedraw || warningActive) break; 
      
      runCombinedWaveformLoop();
      break;
    case SCREEN_THD:
      runTHDCalculation(); 
      break;
    case SCREEN_SETTINGS_CALIB:
      runSettingsCalib(); // [v20] 설정 값 동적 표시
      break;
    case SCREEN_SETTINGS_PROTECT:
      runSettingsProtect(); // [v20] 설정 값 동적 표시
      break;
    case SCREEN_HOME: 
    case SCREEN_SETTINGS:
    case SCREEN_WARNING: 
      // 이 화면들은 정적이므로 동적 업데이트가 필요 없음
      break;
  }
}


// ==============================================================================
// 3. [v20] 터치 입력 확인 함수 (v21과 동일)
// ==============================================================================
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    // 터치 좌표 보정 (필요시)
    // p.x = map(p.x, TS_MINX, TS_MAXX, 0, SCREEN_WIDTH);
    // p.y = map(p.y, TS_MINY, TS_MAXY, 0, SCREEN_HEIGHT);
    // Serial.print("Touch at: "); Serial.print(p.x); Serial.print(", "); Serial.println(p.y);

    // --- [v20] 공통 경고 화면 터치 ---
    if (currentScreen == SCREEN_WARNING) { 
      warningActive = false;
      digitalWrite(RELAY_1_PIN, LOW); 
      digitalWrite(RELAY_2_PIN, LOW); 
      currentScreen = SCREEN_HOME; // [v20] 경고 해제 시 홈으로
      screenNeedsRedraw = true;
      delay(100); // Debounce
      return; 
    }

    // --- [v20] 공통 뒤로 가기 버튼 (<) ---
    if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
      if (p.x >= 5 && p.x <= 55 && p.y >= 5 && p.y <= 35) {
        if (currentScreen == SCREEN_SETTINGS_CALIB || currentScreen == SCREEN_SETTINGS_PROTECT) {
          currentScreen = SCREEN_SETTINGS; // 2차 깊이 -> 설정
        } else {
          currentScreen = SCREEN_HOME; // 1차 깊이 -> 홈
        }
        screenNeedsRedraw = true;
        delay(100); // Debounce
        return;
      }
    }

    // --- [v20] 화면별 버튼 로직 ---
    switch (currentScreen) {
      case SCREEN_HOME:
        // 버튼 1: MAIN POWER (X: 20-150, Y: 50-90)
        if (p.x >= 20 && p.x <= 150 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_MAIN_POWER;
          screenNeedsRedraw = true;
        }
        // 버튼 2: PHASOR (X: 170-300, Y: 50-90)
        else if (p.x >= 170 && p.x <= 300 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_PHASE_DIFFERENCE;
          screenNeedsRedraw = true;
        }
        // 버튼 3: WAVEFORM (X: 20-150, Y: 110-150)
        else if (p.x >= 20 && p.x <= 150 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_COMBINED_WAVEFORM;
          screenNeedsRedraw = true;
        }
        // 버튼 4: THD (X: 170-300, Y: 110-150)
        else if (p.x >= 170 && p.x <= 300 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_THD;
          screenNeedsRedraw = true;
        }
        // 버튼 5: SETTINGS (X: 20-300, Y: 170-210)
        else if (p.x >= 20 && p.x <= 300 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS:
        // 버튼 1: CALIBRATION (X: 20-300, Y: 70-110)
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          currentScreen = SCREEN_SETTINGS_CALIB;
          screenNeedsRedraw = true;
        }
        // 버튼 2: PROTECTION (X: 20-300, Y: 130-170)
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          currentScreen = SCREEN_SETTINGS_PROTECT;
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_CALIB:
        // V_CALIB [+] (X: 250, Y: 70)
        if (p.x >= 240 && p.x <= 300 && p.y >= 60 && p.y <= 90) V_CALIB_RMS += 0.0001;
        // V_CALIB [-] (X: 190, Y: 70)
        else if (p.x >= 180 && p.x <= 240 && p.y >= 60 && p.y <= 90) V_CALIB_RMS -= 0.0001;
        // I_CALIB [+] (X: 250, Y: 130)
        else if (p.x >= 240 && p.x <= 300 && p.y >= 120 && p.y <= 150) I_CALIB_RMS += 0.0001;
        // I_CALIB [-] (X: 190, Y: 130)
        else if (p.x >= 180 && p.x <= 240 && p.y >= 120 && p.y <= 150) I_CALIB_RMS -= 0.0001;
        break;
        
      case SCREEN_SETTINGS_PROTECT:
        // VOLT_THRESH [+] (X: 250, Y: 70)
        if (p.x >= 240 && p.x <= 300 && p.y >= 60 && p.y <= 90) VOLTAGE_THRESHOLD += 1.0;
        // VOLT_THRESH [-] (X: 190, Y: 70)
        else if (p.x >= 180 && p.x <= 240 && p.y >= 60 && p.y <= 90) VOLTAGE_THRESHOLD -= 1.0;
        break;
        
      case SCREEN_MAIN_POWER:
      case SCREEN_PHASE_DIFFERENCE:
      case SCREEN_COMBINED_WAVEFORM:
      case SCREEN_THD:
        // 뒤로 가기 버튼 외에는 터치 이벤트 없음
        break;
    }
    
    delay(100); // Debounce
  }
}

// ==============================================================================
// 4. 헬퍼 함수 (TFT 출력) (v21과 동일)
// ==============================================================================

// printTFTValue (float)
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  // [v20] 값 변경 시에만 다시 그리도록 최적화 (깜빡임 제거)
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) {
    return; // 변경 없음
  }
  
  char buffer[20];
  tft.setTextColor(ILI9341_WHITE); 
  tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit); 
}

// printTFTValue (String)
void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (value.equals(prev_value)) return; // 변경 없음
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(x, y);
  tft.print(prev_value);
  tft.setTextColor(color);
  tft.setCursor(x, y); 
  tft.print(value);
}

// [v19] 위상 계산 헬퍼 함수
float calculatePhase(long time_diff, float period_us) {
  float phase = fmod(((float)time_diff / period_us) * 360.0, 360.0);
  if (phase > 180.0) phase -= 360.0;
  else if (phase < -180.0) phase += 360.0;
  return phase;
}


// displayNetworkStatus
void displayNetworkStatus() {
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(240, 5); // [v20] 위치 수정
  tft.print("NET: OFF"); 
}

// [v20] 뒤로 가기 버튼 그리기
void drawBackButton() {
  tft.fillRoundRect(5, 5, 50, 30, 8, ILI9341_DARKGREY);
  tft.setCursor(20, 12);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("<");
}

// [v20] 공통 버튼 그리기 헬퍼
void drawButton(int x, int y, int w, int h, String text) {
  tft.fillRoundRect(x, y, w, h, 8, ILI9341_BLUE);
  tft.drawRoundRect(x, y, w, h, 8, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  // 텍스트 중앙 정렬
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(x + (w - w1) / 2, y + (h - h1) / 2);
  tft.print(text);
}


// ==============================================================================
// 5. 메인 전력 화면 그리기 (v21과 동일)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(65, 10); // [v20] 뒤로가기 버튼 고려
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  displayNetworkStatus(); 
  drawBackButton(); // [v20]
  
  tft.setTextSize(2);
  tft.setCursor(10, 40); tft.setTextColor(ILI9341_BLUE); tft.println("V:");
  tft.setCursor(10, 65); tft.setTextColor(ILI9341_ORANGE); tft.println("I-M:");
  tft.setCursor(10, 90); tft.setTextColor(ILI9341_RED); tft.println("I-1:");
  tft.setCursor(10, 115); tft.setTextColor(ILI9341_GREEN); tft.println("I-2:");
  tft.setCursor(10, 140); tft.setTextColor(ILI9341_DARKGREEN); tft.println("P:");
  tft.setCursor(10, 165); tft.setTextColor(ILI9341_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 190); tft.setTextColor(ILI9341_ORANGE); tft.println("Q:");
  tft.setCursor(10, 215); tft.setTextColor(ILI9341_BLACK); tft.println("THD:");
}

// ==============================================================================
// 6. 메인 전력 화면 "값" 업데이트 (v21과 동일)
// ==============================================================================
void displayMainScreenValues() {
  // (v19와 동일, 헬퍼 함수가 깜빡임 방지)
  tft.setTextSize(2);
  char buffer[20];
  
  tft.fillRect(45, 40, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_BLUE); 
  tft.setCursor(45, 40);
  dtostrf(V_rms, 4, 1, buffer);
  tft.print(buffer); tft.print(" V");

  tft.fillRect(75, 65, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(75, 65);
  if (I_rms < 1.0) { 
    dtostrf(I_rms * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(75, 90, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_RED);
  tft.setCursor(75, 90);
  if (I_rms_load1 < 1.0) { 
    dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load1, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(75, 115, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(75, 115);
  if (I_rms_load2 < 1.0) { 
    dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load2, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(45, 140, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_DARKGREEN);
  tft.setCursor(45, 140);
  if (P_real >= 1000.0) { 
    dtostrf(P_real / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kW");
  } else { 
    dtostrf(P_real, 4, 1, buffer);
    tft.print(buffer); tft.print(" W");
  }

  tft.fillRect(45, 165, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_MAGENTA);
  tft.setCursor(45, 165);
  dtostrf(PF, 4, 2, buffer);
  tft.print(buffer);

  tft.fillRect(45, 190, 180, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(45, 190);
  if (Q_reactive >= 1000.0) { 
    dtostrf(Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVAR");
  } else { 
    dtostrf(Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" VAR");
  }

  tft.fillRect(60, 215, 170, 20, ILI9341_WHITE); 
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(60, 215);
  dtostrf(THD_value, 4, 1, buffer);
  tft.print(buffer); tft.print(" %"); 
}

// ==============================================================================
// 7. [v19] 위상차 화면 그리기 (v21과 동일)
// ==============================================================================
void displayPhaseScreenStatic() {
  tft.setCursor(65, 10); // [v20]
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("PHASOR DIAGRAM"); 
  displayNetworkStatus();
  drawBackButton(); // [v20]
  
  tft.setTextSize(2);
  tft.setCursor(10, 50); tft.setTextColor(ILI9341_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 75); tft.setTextColor(ILI9341_BLACK); tft.println("Status:");
  
  tft.setTextSize(1);
  tft.setCursor(10, 110); tft.setTextColor(ILI9341_BLACK); tft.print("--- Phase (deg) ---");
  tft.setTextSize(2);
  tft.setCursor(10, 130); tft.setTextColor(ILI9341_ORANGE); tft.println("I-M:");
  tft.setCursor(10, 155); tft.setTextColor(ILI9341_RED); tft.println("I-1:");
  tft.setCursor(10, 180); tft.setTextColor(ILI9341_GREEN); tft.println("I-2:");

  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS, ILI9341_LIGHTGREY);
  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS / 2, ILI9341_LIGHTGREY);
  tft.drawFastHLine(PHASOR_CX - PHASOR_RADIUS, PHASOR_CY, PHASOR_RADIUS * 2, ILI9341_LIGHTGREY);
  tft.drawFastVLine(PHASOR_CX, PHASOR_CY - PHASOR_RADIUS, PHASOR_RADIUS * 2, ILI9341_LIGHTGREY);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(PHASOR_CX + PHASOR_RADIUS + 3, PHASOR_CY - 3);
  tft.print("0");

  int legendY = 195;
  tft.drawRect(155, legendY, 160, 40, ILI9341_DARKGREY);
  tft.drawFastHLine(160, legendY + 19, 10, ILI9341_BLUE);
  tft.setCursor(175, legendY + 4); tft.print("V");
  tft.drawFastHLine(200, legendY + 19, 10, ILI9341_ORANGE);
  tft.setCursor(215, legendY + 4); tft.print("I-M");
  tft.drawFastHLine(240, legendY + 19, 10, ILI9341_RED);
  tft.setCursor(255, legendY + 4); tft.print("I-1");
  tft.drawFastHLine(280, legendY + 19, 10, ILI9341_GREEN);
  tft.setCursor(295, legendY + 4); tft.print("I-2");
  
  prev_phase_degrees = 0.0;
  prev_lead_lag_status = "---"; 
  prev_phase_main_deg = 0.0;
  prev_phase_load1_deg = 0.0;
  prev_phase_load2_deg = 0.0;
  prev_v_x = PHASOR_CX; prev_v_y = PHASOR_CY;
  prev_im_x = PHASOR_CX; prev_im_y = PHASOR_CY;
  prev_i1_x = PHASOR_CX; prev_i1_y = PHASOR_CY;
  prev_i2_x = PHASOR_CX; prev_i2_y = PHASOR_CY;
}

// ==============================================================================
// 8. [v19] 위상차 화면 "값" 업데이트 함수 (v21과 동일)
// ==============================================================================
void displayPhaseScreenValues() {
  
  tft.setTextSize(2);
  printTFTValue(60, 50, PF, prev_phase_degrees, 2, ILI9341_MAGENTA, "");
  printTFTValue(10, 100, lead_lag_status, prev_lead_lag_status, ILI9341_BLACK); 

  printTFTValue(60, 130, phase_main_deg, prev_phase_main_deg, 1, ILI9341_ORANGE, "d");
  printTFTValue(60, 155, phase_load1_deg, prev_phase_load1_deg, 1, ILI9341_RED, "d");
  printTFTValue(60, 180, phase_load2_deg, prev_phase_load2_deg, 1, ILI9341_GREEN, "d");

  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, ILI9341_WHITE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, ILI9341_WHITE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, ILI9341_WHITE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, ILI9341_WHITE);

  float v_len = PHASOR_RADIUS;
  float im_len = constrain((I_rms / PHASOR_MAX_CURRENT) * PHASOR_RADIUS, 0, PHASOR_RADIUS);
  float i1_len = constrain((I_rms_load1 / PHASOR_MAX_CURRENT) * PHASOR_RADIUS, 0, PHASOR_RADIUS);
  float i2_len = constrain((I_rms_load2 / PHASOR_MAX_CURRENT) * PHASOR_RADIUS, 0, PHASOR_RADIUS);

  int v_x = PHASOR_CX + (int)(v_len * cos(0));
  int v_y = PHASOR_CY - (int)(v_len * sin(0));
  int im_x = PHASOR_CX + (int)(im_len * cos(phase_main_deg * (M_PI / 180.0)));
  int im_y = PHASOR_CY - (int)(im_len * sin(phase_main_deg * (M_PI / 180.0)));
  int i1_x = PHASOR_CX + (int)(i1_len * cos(phase_load1_deg * (M_PI / 180.0)));
  int i1_y = PHASOR_CY - (int)(i1_len * sin(phase_load1_deg * (M_PI / 180.0)));
  int i2_x = PHASOR_CX + (int)(i2_len * cos(phase_load2_deg * (M_PI / 180.0)));
  int i2_y = PHASOR_CY - (int)(i2_len * sin(phase_load2_deg * (M_PI / 180.0)));

  tft.drawLine(PHASOR_CX, PHASOR_CY, v_x, v_y, ILI9341_BLUE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, im_x, im_y, ILI9341_ORANGE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i1_x, i1_y, ILI9341_RED);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i2_x, i2_y, ILI9341_GREEN);
  
  prev_phase_degrees = PF;
  prev_lead_lag_status = lead_lag_status; 
  prev_phase_main_deg = phase_main_deg;
  prev_phase_load1_deg = phase_load1_deg;
  prev_phase_load2_deg = phase_load2_deg;
  
  prev_v_x = v_x; prev_v_y = v_y;
  prev_im_x = im_x; prev_im_y = im_y;
  prev_i1_x = i1_x; prev_i1_y = i1_y;
  prev_i2_x = i2_x; prev_i2_y = i2_y;
}

// ==============================================================================
// 9. 60Hz 0점 통과(Zero-Crossing) 대기 함수 [v25 수정]
// ==============================================================================
void waitForVoltageZeroCross() {
  long startTime = micros();
  long timeout = 20000; 
  float V_ac_bits_prev = 0; // [v25] float로 변경
  int V_raw = 0;
  float V_ac_bits = 0;  // [v25] float로 변경
  
  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = (float)V_raw - V_ADC_MIDPOINT_ACTUAL; // [v25] 수정
    if (V_ac_bits < -50) {
       V_ac_bits_prev = V_ac_bits;
       break;
    }
    if (micros() - startTime > timeout) return; 
  }

  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = (float)V_raw - V_ADC_MIDPOINT_ACTUAL; // [v25] 수정
    if (V_ac_bits_prev < 0 && V_ac_bits >= 0) return;
    V_ac_bits_prev = V_ac_bits;
    if (micros() - startTime > timeout * 2) return; 
  }
}


// ==============================================================================
// 10. [v19] 공통 파형 화면 (정적 그리드 + Vpk/Ipk 라벨) (v21과 동일)
// ==============================================================================
void drawWaveformGridAndLabels() {
  
  tft.setCursor(65, 10); // [v20]
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("V/I WAVEFORM (60Hz Sync)");
  displayNetworkStatus(); 
  drawBackButton(); // [v20]
  
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(150, 25);
  tft.print("Vpk:");
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(150, 40);
  tft.print("Ipk:");

  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), ILI9341_LIGHTGREY); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, ILI9341_LIGHTGREY); 
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), ILI9341_DARKGREY); 
  tft.drawFastVLine(PLOT_X_END, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), ILI9341_DARKGREY); 

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(0, PLOT_Y_CENTER - 4); 
  tft.print("0A"); 
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4); 
  tft.print("0V"); 

  tft.setCursor(10, SCREEN_HEIGHT - 12);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.print("Sampling Period: ");
  tft.print(WAVEFORM_SAMPLE_PERIOD_US);
  tft.print(" us");
}

// ==============================================================================
// 11. 동적 Y축 라벨 업데이트 함수 (v21과 동일)
// ==============================================================================
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, ILI9341_WHITE); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, ILI9341_WHITE); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, ILI9341_WHITE); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, ILI9341_WHITE); 
  
  tft.setTextColor(ILI9341_ORANGE);
  if (I_peak_dynamic < 1.0) { // [v21-FIX]
    dtostrf(I_peak_dynamic * 1000, 3, 0, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
  } else { 
    dtostrf(I_peak_dynamic, 3, 1, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
  }

  tft.setTextColor(ILI9341_BLUE);
  dtostrf(V_peak_dynamic, 3, 0, buffer); // [v21-FIX]
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
}


// ==============================================================================
// 12. 경고 팝업 화면 (v21과 동일)
// ==============================================================================
void displayWarningScreenStatic() {
  tft.fillScreen(ILI9341_WHITE);
  tft.drawRect(20, 50, 280, 140, ILI9341_RED);
  tft.drawRect(21, 51, 278, 138, ILI9341_RED);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(3); tft.setCursor(80, 70); tft.print("WARNING");
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2); tft.setCursor(60, 110); tft.print(warningMessage);
  tft.setTextSize(3); 
  if (warningMessage.startsWith("OVER VOLTAGE")) {
    tft.setCursor(80, 150); tft.print(V_rms, 1); tft.print(" V");
  } else {
    tft.setCursor(80, 150); tft.print(I_rms, 2); tft.print(" A");
  }
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1); tft.setCursor(100, 210);
  tft.print("Tap screen to reset"); 
}


// ==============================================================================
// 13. [v25] 실시간 전력 계산 (v24 보정 로직 적용)
// ==============================================================================
void calculatePowerMetrics() {
  
  // [v25] v24의 정밀도 향상 (double) 적용
  double V_sq_sum = 0.0; 
  double I_sq_sum = 0.0; 
  double I_sq_sum_load1 = 0.0;
  double I_sq_sum_load2 = 0.0;
  double P_sum = 0.0; 
  int V_ac_max = 0;
  int I_ac_max = 0;

  float V_ac_bits_prev = 0.0;  // [v25] float로 변경
  float I_ac_bits_prev = 0.0;  // [v25] float로 변경
  float I1_ac_bits_prev = 0.0; // [v25] float로 변경
  float I2_ac_bits_prev = 0.0; // [v25] float로 변경
  long time_V_cross = -1, time_I_cross = -1, time_I1_cross = -1, time_I2_cross = -1;
  bool found_V_cross = false, found_I_cross = false, found_I1_cross = false, found_I2_cross = false; 

  unsigned long startTimePass = micros();
  
  THD_value = thd_v_value * 100.0;
  
  runFuzzyLogic(); 

  for (int i = 0; i < SAMPLES_PER_CALC; i++) { 
    int V_raw = analogRead(VOLTAGE_PIN); 
    int I_raw = analogRead(CURRENT_PIN); 
    int I_raw_load1 = analogRead(CURRENT_PIN_LOAD1);
    int I_raw_load2 = analogRead(CURRENT_PIN_LOAD2);
    
    // [v25] v24의 보정된 오프셋 사용 (기존 8192.0 대신)
    float V_ac_bits = (float)V_raw - V_ADC_MIDPOINT_ACTUAL;
    float I_ac_bits = (float)I_raw - I_ADC_MIDPOINT_MAIN_ACTUAL;
    float I_ac_bits_load1 = (float)I_raw_load1 - I_ADC_MIDPOINT_1_ACTUAL;
    float I_ac_bits_load2 = (float)I_raw_load2 - I_ADC_MIDPOINT_2_ACTUAL;
    
    // [v25] double 변수에 누적
    V_sq_sum += (double)V_ac_bits * V_ac_bits; 
    I_sq_sum += (double)I_ac_bits * I_ac_bits; 
    I_sq_sum_load1 += (double)I_ac_bits_load1 * I_ac_bits_load1;
    I_sq_sum_load2 += (double)I_ac_bits_load2 * I_ac_bits_load2;
    P_sum += (double)V_ac_bits * I_ac_bits; 

    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits); // [v25] float 비교
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits); // [v25] float 비교
    
    if (i < (SAMPLE_PERIOD_US * 1000 / FREQUENCY / SAMPLE_PERIOD_US)) { 
      unsigned long crossTime = micros();
      // [v25] float 비교
      if (!found_V_cross && V_ac_bits_prev < 0 && V_ac_bits >= 0) {
        time_V_cross = crossTime;
        found_V_cross = true; 
      }
      if (!found_I_cross && I_ac_bits_prev < 0 && I_ac_bits >= 0) {
        time_I_cross = crossTime;
        found_I_cross = true; 
      }
      if (!found_I1_cross && I1_ac_bits_prev < 0 && I_ac_bits_load1 >= 0) {
        time_I1_cross = crossTime;
        found_I1_cross = true; 
      }
      if (!found_I2_cross && I2_ac_bits_prev < 0 && I_ac_bits_load2 >= 0) {
        time_I2_cross = crossTime;
        found_I2_cross = true; 
      }
    }
    V_ac_bits_prev = V_ac_bits; 
    I_ac_bits_prev = I_ac_bits; 
    I1_ac_bits_prev = I_ac_bits_load1;
    I2_ac_bits_prev = I_ac_bits_load2;
    
    while(micros() - startTimePass < (i + 1) * SAMPLE_PERIOD_US); 
    
    // [v20] 터치 확인은 메인 루프로 이동됨
  }

  // [v25] double 기반 계산
  float V_rms_adc = sqrt((float)(V_sq_sum / SAMPLES_PER_CALC));
  float I_rms_adc = sqrt((float)(I_sq_sum / SAMPLES_PER_CALC)); 
  float I_rms_adc_load1 = sqrt((float)(I_sq_sum_load1 / SAMPLES_PER_CALC));
  float I_rms_adc_load2 = sqrt((float)(I_sq_sum_load2 / SAMPLES_PER_CALC));
  float P_avg_adc = (float)(P_sum / SAMPLES_PER_CALC);

  // [v25] v21의 코드를 따르되, V_RMS_OFFSET_CORRECTION 변수는 전역에서 0.0으로 설정됨.
  V_rms = (V_rms_adc * V_CALIB_RMS) - V_RMS_OFFSET_CORRECTION;
  I_rms = (I_rms_adc * I_CALIB_RMS) - I_RMS_OFFSET_CORRECTION; 
  I_rms_load1 = (I_rms_adc_load1 * I_CALIB_RMS) - I_RMS_OFFSET_CORRECTION;
  I_rms_load2 = (I_rms_adc_load2 * I_CALIB_RMS) - I_RMS_OFFSET_CORRECTION;
  P_real = P_avg_adc * V_CALIB_RMS * I_CALIB_RMS; 
  
  if (V_rms < 0) V_rms = 0; 
  if (I_rms < 0) I_rms = 0; 
  if (I_rms_load1 < 0) I_rms_load1 = 0;
  if (I_rms_load2 < 0) I_rms_load2 = 0;

  V_peak_dynamic = (V_ac_max * V_CALIB_RMS) * 1.1; // [v21-FIX]
  I_peak_dynamic = (I_ac_max * I_CALIB_RMS) * 1.1; // [v21-FIX]
  if (V_peak_dynamic < 50.0) V_peak_dynamic = 50.0; 
  if (I_peak_dynamic < 1.0) I_peak_dynamic = 1.0; 

  S_apparent = V_rms * I_rms; 
  
  if (S_apparent < 0.01) { 
    PF = 0.0; P_real = 0.0; Q_reactive = 0.0; 
  } else {
    PF = P_real / S_apparent; 
    if (PF > 1.0) PF = 1.0; 
    if (PF < -1.0) PF = -1.0; 
    Q_reactive = sqrt(max(0.0, S_apparent * S_apparent - P_real * P_real)); 
  }

  float period_us = 1000000.0 / FREQUENCY; 
  phase_main_deg = 0.0;
  phase_load1_deg = 0.0;
  phase_load2_deg = 0.0;
  
  if (found_V_cross) {
    if (found_I_cross) phase_main_deg = calculatePhase(time_I_cross - time_V_cross, period_us);
    if (found_I1_cross) phase_load1_deg = calculatePhase(time_I1_cross - time_V_cross, period_us);
    if (found_I2_cross) phase_load2_deg = calculatePhase(time_I2_cross - time_V_cross, period_us);
  }

  phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
  if (phase_main_deg < -2.0) lead_lag_status = "Lead";
  else if (phase_main_deg > 2.0) lead_lag_status = "Lag";
  else lead_lag_status = "---";

  if (I_rms < 0.05) {
     phase_degrees = 0.0;
     phase_main_deg = 0.0;
     lead_lag_status = "---"; 
     PF = (V_rms > 10.0) ? 1.0 : 0.0;
     P_real = 0.0; 
     Q_reactive = 0.0; 
  }
  if (I_rms_load1 < 0.05) { I_rms_load1 = 0.0; phase_load1_deg = 0.0; }
  if (I_rms_load2 < 0.05) { I_rms_load2 = 0.0; phase_load2_deg = 0.0; }

  // [v20] 실시간 설정 변수(VOLTAGE_THRESHOLD) 사용
  if (V_rms > VOLTAGE_THRESHOLD) {
    digitalWrite(RELAY_1_PIN, HIGH);
    digitalWrite(RELAY_2_PIN, HIGH); 
    warningMessage = "OVER VOLTAGE!"; 
    warningActive = true;
    screenNeedsRedraw = true; 
  }
}

// ==============================================================================
// 14. 메인 화면 실행 함수 (v21과 동일)
// ==============================================================================
void runMainPowerCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayMainScreenValues();
}

// ==============================================================================
// 15. 위상차 화면 실행 함수 (v21과 동일)
// ==============================================================================
void runPhaseCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayPhaseScreenValues();
}

// ==============================================================================
// 16. [v19] 파형 그리기 루프 [v25 수정]
// ==============================================================================
void runCombinedWaveformLoop() {
  float volts_to_pixels_scale = (V_peak_dynamic < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_peak_dynamic); // [v21-FIX]
  float amps_to_pixels_scale = (I_peak_dynamic < 0.1) ? 0 : (PLOT_HEIGHT_HALF / I_peak_dynamic); // [v21-FIX]

  float new_frame_V_peak = 0.0;
  float new_frame_I_peak = 0.0;
  
  int new_y_v[PLOT_WIDTH];
  int new_y_i[PLOT_WIDTH];

  unsigned long startTime = micros();

  for (int i = 0; i < PLOT_WIDTH; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    
    // [v25] v24의 보정된 오프셋 사용 (기존 8192.0 대신)
    float V_ac_bits = (float)V_raw - V_ADC_MIDPOINT_ACTUAL;
    float I_ac_bits = (float)I_raw - I_ADC_MIDPOINT_MAIN_ACTUAL;
    
    // [v25] 땜질 처방(OFFSET_CORRECTION) 변수가 0.0이므로 순수하게 계산됨
    float V_mains_instant = V_ac_bits * V_CALIB_RMS + V_RMS_OFFSET_CORRECTION;
    float I_mains_instant = I_ac_bits * I_CALIB_RMS - I_RMS_OFFSET_CORRECTION;

    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    if (abs(I_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I_mains_instant);

    int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
    new_y_v[i] = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
    new_y_i[i] = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
    
    while(micros() - startTime < (i + 1) * WAVEFORM_SAMPLE_PERIOD_US);
  }

  tft.startWrite(); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, ILI9341_LIGHTGREY);
  for (int i = 1; i < PLOT_WIDTH; i++) {
       int x_curr = PLOT_X_START + i;
       int x_prev = PLOT_X_START + i - 1;
       if(last_frame_y_v[i] != PLOT_Y_CENTER || last_frame_y_v[i-1] != PLOT_Y_CENTER) {
           tft.drawLine(x_prev, last_frame_y_v[i-1], x_curr, last_frame_y_v[i], ILI9341_WHITE);
       }
       if(last_frame_y_i[i] != PLOT_Y_CENTER || last_frame_y_i[i-1] != PLOT_Y_CENTER) {
           tft.drawLine(x_prev, last_frame_y_i[i-1], x_curr, last_frame_y_i[i], ILI9341_WHITE);
       }
       tft.drawLine(x_prev, new_y_v[i-1], x_curr, new_y_v[i], ILI9341_BLUE);
       tft.drawLine(x_prev, new_y_i[i-1], x_curr, new_y_i[i], ILI9341_ORANGE);
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
       last_frame_y_v[i] = new_y_v[i];
       last_frame_y_i[i] = new_y_i[i];
  }

  V_peak_dynamic = (new_frame_V_peak < 50.0) ? 50.0 : new_frame_V_peak * 1.1;
  I_peak_dynamic = (new_frame_I_peak < 1.0) ? 1.0 : new_frame_I_peak * 1.1;
  updateYAxisLabels();

  tft.setTextSize(1);
  printTFTValue(180, 25, new_frame_V_peak, prev_frame_V_peak, 1, ILI9341_BLUE, " V");
  if (new_frame_I_peak < 1.0) {
    printTFTValue(180, 40, new_frame_I_peak * 1000.0, prev_frame_I_peak * 1000.0, 0, ILI9341_ORANGE, " mA");
  } else {
    printTFTValue(180, 40, new_frame_I_peak, prev_frame_I_peak, 2, ILI9341_ORANGE, " A");
  }
  prev_frame_V_peak = new_frame_V_peak;
  prev_frame_I_peak = new_frame_I_peak;
}
// ==============================================================================
// 17. 퍼지 로직 시스템 빌드 함수 (v21과 동일)
// ==============================================================================
void buildFuzzySystem() {
  // [v20] 참고: 퍼지 로직의 임계값은 아직 실시간 변경을 지원하지 않습니다.
  // (구현하려면 FuzzySet 객체들을 삭제하고 다시 생성하는 로직이 필요함)
  safeCurrent = new FuzzySet(0, 0, 5, 6); 
  warningCurrent = new FuzzySet(5, 6, 6, 7); 
  dangerCurrent = new FuzzySet(6.5, 7.25, 7.25, 8); 
  criticalCurrent = new FuzzySet(7.5, 8, 10, 10); 
  totalCurrent->addFuzzySet(safeCurrent);
  totalCurrent->addFuzzySet(warningCurrent); 
  totalCurrent->addFuzzySet(dangerCurrent);
  totalCurrent->addFuzzySet(criticalCurrent);
  fuzzy->addFuzzyInput(totalCurrent); 

  stableChange = new FuzzySet(-2, -1, 1, 2);
  slowIncrease = new FuzzySet(1, 3, 3, 5);
  suddenSurge = new FuzzySet(4, 6, 10, 10); 
  currentChangeRate->addFuzzySet(stableChange);
  currentChangeRate->addFuzzySet(slowIncrease);
  currentChangeRate->addFuzzySet(suddenSurge);
  fuzzy->addFuzzyInput(currentChangeRate); 

  level0 = new FuzzySet(0, 0, 0, 1);
  level3 = new FuzzySet(2, 3, 3, 4);
  level6 = new FuzzySet(5, 6, 6, 7);
  level10 = new FuzzySet(9, 10, 10, 10);
  shutdownLevel->addFuzzySet(level0);
  shutdownLevel->addFuzzySet(level3);
  shutdownLevel->addFuzzySet(level6);
  shutdownLevel->addFuzzySet(level10);
  fuzzy->addFuzzyOutput(shutdownLevel); 

  FuzzyRuleAntecedent* if_Safe_and_Stable = new FuzzyRuleAntecedent();
  if_Safe_and_Stable->joinWithAND(safeCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level0 = new FuzzyRuleConsequent();
  then_Level0->addOutput(level0);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_Safe_and_Stable, then_Level0)); 

  FuzzyRuleAntecedent* if_Warning_and_Stable = new FuzzyRuleAntecedent();
  if_Warning_and_Stable->joinWithAND(warningCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level3 = new FuzzyRuleConsequent();
  then_Level3->addOutput(level3); 
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_Warning_and_Stable, then_Level3)); 

  FuzzyRuleAntecedent* if_Danger_and_Stable = new FuzzyRuleAntecedent();
  if_Danger_and_Stable->joinWithAND(dangerCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level6 = new FuzzyRuleConsequent();
  then_Level6->addOutput(level6); 
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_Danger_and_Stable, then_Level6)); 

  FuzzyRuleAntecedent* if_Warning_and_Slow = new FuzzyRuleAntecedent(); 
  if_Warning_and_Slow->joinWithAND(warningCurrent, slowIncrease); 
  fuzzy->addFuzzyRule(new FuzzyRule(4, if_Warning_and_Slow, then_Level6)); 
  
  FuzzyRuleAntecedent* if_Critical = new FuzzyRuleAntecedent();
  if_Critical->joinSingle(criticalCurrent);
  FuzzyRuleConsequent* then_Level10 = new FuzzyRuleConsequent();
  then_Level10->addOutput(level10); 
  fuzzy->addFuzzyRule(new FuzzyRule(5, if_Critical, then_Level10)); 

  FuzzyRuleAntecedent* if_Surge = new FuzzyRuleAntecedent();
  if_Surge->joinSingle(suddenSurge);
  fuzzy->addFuzzyRule(new FuzzyRule(6, if_Surge, then_Level10));
}


// ==============================================================================
// 18. 퍼지 로직 실행 헬퍼 함수 (v21과 동일)
// ==============================================================================
void runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  if (deltaTime < 0.1) return; 

  float dI_dt = (I_rms - last_I_rms) / deltaTime;
  last_I_rms = I_rms;
  lastFuzzyTime = currentTime; 

  fuzzy->setInput(1, I_rms); 
  fuzzy->setInput(2, dI_dt); 
  fuzzy->fuzzify(); 
  float outputLevel = fuzzy->defuzzify(1); 
  controlRelays(outputLevel); 
}


// ==============================================================================
// 19. 퍼지 출력 -> 릴레이 제어 변환 함수 (v21과 동일)
// ==============================================================================
void controlRelays(float level) {
  // [v25] 참고: 이 로직은 v21(친구) 기준이며, v19(지능형 차단)와 다릅니다.
  if (level > 9.0) {
    digitalWrite(RELAY_2_PIN, HIGH);
    digitalWrite(RELAY_1_PIN, HIGH);
    if (!warningActive) {
      warningMessage = "FUZZY LOGIC TRIP";
      warningActive = true; 
      screenNeedsRedraw = true; 
    }
  } 
  else if (level > 5.0) {
    digitalWrite(RELAY_2_PIN, HIGH);
    digitalWrite(RELAY_1_PIN, HIGH);
  } 
  else if (level > 2.0) {
    digitalWrite(RELAY_2_PIN, LOW);
    digitalWrite(RELAY_1_PIN, HIGH);
  } 
  else { 
    if (!warningActive) {
      digitalWrite(RELAY_2_PIN, LOW);
      digitalWrite(RELAY_1_PIN, LOW);
    }
  }
}

// ==============================================================================
// 20. THD 화면 실행 함수 (v21과 동일)
// ==============================================================================
void runTHDCalculation() {
  performFFT_and_CalcTHD(); 
  if (screenNeedsRedraw || warningActive) return; 
  displayTHDScreenValues(); 
}

// ==============================================================================
// 21. THD 화면 정적 UI 그리기 (v21과 동일)
// ==============================================================================
void displayTHDScreenStatic() {
  tft.setCursor(65, 10); // [v20]
  tft.setTextColor(ILI9341_BLACK); 
  tft.setTextSize(2);
  tft.println("HARMONIC DISTORTION"); 
  displayNetworkStatus();
  drawBackButton(); // [v20]
  
  tft.setTextSize(3);
  tft.setCursor(10, 70); tft.setTextColor(ILI9341_BLUE); tft.println("THD-V:");
  tft.setCursor(10, 150); tft.setTextColor(ILI9341_ORANGE); tft.println("THD-I:");
  
  prev_thd_v = thd_v_value; 
  prev_thd_i = thd_i_value; 
}

// ==============================================================================
// 22. THD 화면 동적 값 업데이트 (v21과 동일)
// ==============================================================================
void displayTHDScreenValues() {
  tft.setTextSize(4);
  printTFTValue(30, 110, thd_v_value * 100.0, prev_thd_v * 100.0, 1, ILI9341_BLUE, " %"); 
  printTFTValue(30, 190, thd_i_value * 100.0, prev_thd_i * 100.0, 1, ILI9341_ORANGE, " %"); 
  prev_thd_v = thd_v_value; 
  prev_thd_i = thd_i_value; 
}


// ==============================================================================
// 23. CMSIS-DSP FFT 및 THD 계산 헬퍼 함수 (v21과 동일)
// ==============================================================================
float32_t calculateTHD(float32_t* mags, int fundamentalBin) {
  float32_t fundamental_power = mags[fundamentalBin] * mags[fundamentalBin];
  float32_t harmonics_power_sum = 0.0; 
  if (fundamental_power < 1e-10) return 0.0;

  for (int n = 2; n <= MAX_HARMONIC; n++) {
    int binIndex = fundamentalBin * n;
    if (binIndex >= (FFT_N / 2)) break;
    harmonics_power_sum += mags[binIndex] * mags[binIndex]; 
  }
  return sqrt(harmonics_power_sum / fundamental_power); 
}


// ==============================================================================
// 24. 메인 FFT/THD 연산 함수 [v25 수정]
// ==============================================================================
void performFFT_and_CalcTHD() {
  unsigned long startTime = micros(); 
  
  for (int i = 0; i < FFT_N; i++) {
    // [v25] v24의 보정된 오프셋 사용 (기존 8192.0 대신)
    v_samples[i] = (float32_t)(analogRead(VOLTAGE_PIN)) - V_ADC_MIDPOINT_ACTUAL; 
    i_samples[i] = (float32_t)(analogRead(CURRENT_PIN)) - I_ADC_MIDPOINT_MAIN_ACTUAL;
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US); 
  }
  
  // [v20] 터치 확인은 메인 루프로 이동됨

  for (int i = 0; i < FFT_N; i++) {
    float32_t window_factor = 0.5f - 0.5f * arm_cos_f32(2.0f * PI * i / (FFT_N - 1));
    v_samples[i] = v_samples[i] * window_factor; 
    i_samples[i] = i_samples[i] * window_factor; 
  }
  
  arm_rfft_fast_f32(&fft_inst_v, v_samples, v_fft_output, 0); 
  arm_cmplx_mag_f32(v_fft_output, v_mags, FFT_N / 2); 
  arm_rfft_fast_f32(&fft_inst_i, i_samples, i_fft_output, 0); 
  arm_cmplx_mag_f32(i_fft_output, i_mags, FFT_N / 2); 
  
  // [v20] 터치 확인은 메인 루프로 이동됨
  
  thd_v_value = calculateTHD(v_mags, FUNDAMENTAL_BIN); 
  thd_i_value = calculateTHD(i_mags, FUNDAMENTAL_BIN); 
  THD_value = thd_v_value * 100.0;
}

// ==============================================================================
// 25. [v20] 홈 화면 그리기 (v21과 동일)
// ==============================================================================
void displayHomeScreenStatic() {
  tft.setCursor(50, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.print("KW WATT-METER");
  
  displayNetworkStatus();

  // (x, y, w, h, text)
  drawButton(20, 50, 130, 40, "MAIN POWER");
  drawButton(170, 50, 130, 40, "PHASOR");
  drawButton(20, 110, 130, 40, "WAVEFORM");
  drawButton(170, 110, 130, 40, "THD");
  drawButton(20, 170, 280, 40, "SETTINGS");
}

// ==============================================================================
// 26. [v20] 설정 메인 화면 그리기 (v21과 동일)
// ==============================================================================
void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("SETTINGS");
  drawBackButton();
  
  drawButton(20, 70, 280, 40, "CALIBRATION");
  drawButton(20, 130, 280, 40, "PROTECTION");
}

// ==============================================================================
// 27. [v20] 설정 - 보정 화면 그리기 (v21과 동일)
// ==============================================================================
void displaySettingsCalibStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("CALIBRATION SETTINGS");
  drawBackButton();

  // V_CALIB
  tft.setCursor(10, 70);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.print("V Calib:");
  drawButton(180, 60, 60, 30, "-");
  drawButton(240, 60, 60, 30, "+");

  // I_CALIB
  tft.setCursor(10, 130);
  tft.print("I Calib:");
  drawButton(180, 120, 60, 30, "-");
  drawButton(240, 120, 60, 30, "+");
  
  tft.setTextSize(1);
  tft.setCursor(10, 200);
  tft.print("NOTE: Values reset on reboot.");
  tft.setCursor(10, 215);
  tft.print("CALIB = (Reading / Real) * Old_CALIB");
}

void runSettingsCalib() {
  displaySettingsCalibValues();
}

void displaySettingsCalibValues() {
  tft.setTextSize(2);
  // [v20] 헬퍼 함수를 사용하여 값 표시 (정밀도 4)
  printTFTValue(10, 95, V_CALIB_RMS, prev_V_CALIB_RMS, 4, ILI9341_BLUE, "");
  printTFTValue(10, 155, I_CALIB_RMS, prev_I_CALIB_RMS, 4, ILI9341_ORANGE, "");
  
  prev_V_CALIB_RMS = V_CALIB_RMS;
  prev_I_CALIB_RMS = I_CALIB_RMS;
}

// ==============================================================================
// 28. [v20] 설정 - 보호 화면 그리기 (v21과 동일)
// ==============================================================================
void displaySettingsProtectStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("PROTECTION SETTINGS");
  drawBackButton();

  // VOLTAGE_THRESHOLD
  tft.setCursor(10, 70);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.print("Over V Thresh:");
  drawButton(180, 60, 60, 30, "-");
  drawButton(240, 60, 60, 30, "+");

  tft.setTextSize(1);
  tft.setCursor(10, 200);
  tft.print("NOTE: Values reset on reboot.");
}

void runSettingsProtect() {
  displaySettingsProtectValues();
}

void displaySettingsProtectValues() {
  tft.setTextSize(2);
  // [v20] 헬퍼 함수를 사용하여 값 표시 (정밀도 1)
  printTFTValue(10, 95, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, ILI9341_RED, " V");
  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
}
