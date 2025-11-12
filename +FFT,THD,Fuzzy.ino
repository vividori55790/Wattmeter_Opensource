/*
 * ==============================================================================
 * [v17 - 최종 수정]
 * - v17의 전체 구조에 'Fuzzy.h' 라이브러리의 정확한 문법을 적용.
 * - 'eFLL.h'의 'new Trap', 'new Trian'을 'Fuzzy.h'의 'new FuzzySet(a,b,c,d)'로
 * 전부 교체함.
 * - 객체 생성 방식을 'Fuzzy.h' 예제에 맞게 포인터 방식으로 변경
 * (예: 'fuzzy.fuzzify()' -> 'fuzzy->fuzzify()')
 * ==============================================================================
 */

// --- 라이브러리 포함 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h> // 터치스크린 라이브러리
#include <math.h>
#include <Fuzzy.h> // [v17] 퍼지 로직 라이브러리
#include <FuzzySet.h>
#include <arm_math.h>

// --- 화면 상태 정의 ---
enum ScreenState {
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_THD,
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_MAIN_POWER;
volatile bool screenNeedsRedraw = true;

// --- [v17] 릴레이 핀 3개로 변경 ---
#define RELAY_1_PIN 4        // 부하 1 릴레이
#define RELAY_2_PIN 5       // 부하 2 릴레이
// --- [v17] 기존 임계값 설정 (과전압은 유지) ---
#define VOLTAGE_THRESHOLD 240.0
// #define CURRENT_THRESHOLD 10.0 // [v17] 퍼지 로직이 대체

volatile bool warningActive = false;
String warningMessage = "";

// --- 핀 정의 (센서) ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A4

// --- 핀 정의 (Display & Touch) ---
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7

#define FFT_N 512                   // FFT 샘플 개수 (2의 거듭제곱)
#define SAMPLING_FREQ_HZ 7680.0f    // 샘플링 주파수 (7.68kHz)
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ) // 약 130.2µs
#define FUNDAMENTAL_BIN 4           // 60Hz에 해당하는 FFT Bin (60Hz / 15Hz/bin)
#define MAX_HARMONIC 40

// ==============================================================================
// (v16과 동일한 설정값들...)
// ==============================================================================
const float V_RMS_OFFSET_CORRECTION = 7.1; 
const float I_RMS_OFFSET_CORRECTION = 2.5546; 
const float V_ADC_MIDPOINT = 8192.0;
const float I_ADC_MIDPOINT = 8192.0;
const float V_CALIB_RMS = 0.1775;
const float I_CALIB_RMS = 0.0482;   

#define SAMPLES_PER_CALC 334
#define SAMPLE_PERIOD_US 1000
const float FREQUENCY = 60.0;
#define TS_MINX 370
#define TS_MINY 450
#define TS_MAXX 3760
#define TS_MAXY 3670
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define PLOT_X_START 30 
#define PLOT_X_END 290 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START) 
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2))

const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
float V_peak_dynamic = 360.0;
float I_peak_dynamic = 42.4;
float frame_V_peak = 0.0;
float frame_I_peak = 0.0;
int current_x = PLOT_X_START;
int last_y_v = PLOT_Y_CENTER;
int last_y_i = PLOT_Y_CENTER;
float V_rms = 0.0;
float I_rms = 0.0;
float P_real = 0.0;
float Q_reactive = 0.0;
float S_apparent = 0.0;
float PF = 0.0;
float THD_value = 0.0;
float phase_degrees = 0.0;
String lead_lag_status = "---";
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";

float32_t v_samples[FFT_N];         // 전압 샘플 버퍼
float32_t i_samples[FFT_N];         // 전류 샘플 버퍼
float32_t v_fft_output[FFT_N];      // FFT 연산 결과 (복소수)
float32_t i_fft_output[FFT_N];      // FFT 연산 결과 (복소수)
float32_t v_mags[FFT_N / 2];        // 전압 스펙트럼 (크기)
float32_t i_mags[FFT_N / 2];        // 전류 스펙트럼 (크기)
float32_t thd_v_value = 0.0;        // THD-V (전압 고조파 왜곡률)
float32_t thd_i_value = 0.0;        // THD-I (전류 고조파 왜곡률)
float32_t prev_thd_v = 0.0;         // 깜빡임 제거용
float32_t prev_thd_i = 0.0;

arm_rfft_fast_instance_f32 fft_inst_v;
arm_rfft_fast_instance_f32 fft_inst_i;

// --- [v17 수정] 퍼지 로직 전역 포인터 변수 ---
Fuzzy *fuzzy;
FuzzyInput *totalCurrent;
FuzzyInput *currentChangeRate;
FuzzyOutput *shutdownLevel;
FuzzySet* safeCurrent;
FuzzySet* warningCurrent;
FuzzySet* dangerCurrent;
FuzzySet* criticalCurrent;
FuzzySet* stableChFuzzyange;
FuzzySet* stableChange;
FuzzySet* slowIncrease;
FuzzySet* suddenSurge;
FuzzySet* level0;
FuzzySet* level3;
FuzzySet* level6;
FuzzySet* level10;
// dI/dt 계산용
float last_I_rms = 0.0;
unsigned long lastFuzzyTime = 0;
// --- [v17] ---

// --- [v16] 함수 프로토타입 선언 ---
void updateYAxisLabels();
// --- [v17] 함수 프로토타입 선언 ---
void buildFuzzySystem();
void runFuzzyLogic();
void controlRelays(float level);


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Wattmeter v17 (Fuzzy.h 최종) Booting...");

  analogReadResolution(14);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  // [v17] 3개 릴레이 핀 초기화 (LOW = ON / 정상)
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, LOW); 
  digitalWrite(RELAY_2_PIN, LOW); 


  tft.begin();
  tft.setRotation(3); 
  
  ts.begin(SPI);
  ts.setRotation(3); 
  
  Serial.println("TFT & Touch OK");
  Serial.print("Using V_RMS_OFFSET_CORRECTION: "); Serial.println(V_RMS_OFFSET_CORRECTION);
  Serial.print("Using I_RMS_OFFSET_CORRECTION: "); Serial.println(I_RMS_OFFSET_CORRECTION);

  // [v17 수정] 퍼지 객체 생성 (포인터 초기화)
  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1);
  currentChangeRate = new FuzzyInput(2);
  shutdownLevel = new FuzzyOutput(1);
  
  // [v17] 퍼지 시스템 빌드
  buildFuzzySystem(); // 'Fuzzy.h' 문법으로 수정됨
  Serial.println("Fuzzy Logic System Built (using Fuzzy.h)");
  lastFuzzyTime = millis();
  // [v17] ---

 // [v18] FFT 인스턴스 초기화
  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N);
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N);
  Serial.println("CMSIS-DSP FFT Initialized.");
  // [v18] ---

  screenNeedsRedraw = true;
}

// ==============================================================================
// 2. Main Loop (v16과 동일)
// ==============================================================================
void loop() {
  checkTouchInput();
  
  if (screenNeedsRedraw) {
    tft.fillScreen(ILI9341_WHITE); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic();
    } else {
      switch(currentScreen) {
        case SCREEN_MAIN_POWER:
          displayMainScreenStatic();
          break;
        case SCREEN_PHASE_DIFFERENCE:
          displayPhaseScreenStatic();
          break;
        case SCREEN_COMBINED_WAVEFORM:
          drawWaveformGridAndLabels(); 
          updateYAxisLabels();      
          current_x = PLOT_X_START;
          last_y_v = PLOT_Y_CENTER;
          last_y_i = PLOT_Y_CENTER;
          frame_V_peak = 0.0;
          frame_I_peak = 0.0;
          break;
          case SCREEN_THD: // [v18] THD 화면 정적 UI 그리기
          displayTHDScreenStatic();
          break;
        case SCREEN_WARNING:
          break;
      }
    }
    screenNeedsRedraw = false;
  }
  
  // 3. 현재 화면 상태에 따라 적절한 동적 작업 실행
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      runMainPowerCalculation();
      break;
    case SCREEN_PHASE_DIFFERENCE:
      runPhaseCalculation();
      break;
    case SCREEN_COMBINED_WAVEFORM:
      runCombinedWaveformLoop(); // [v17] 경고: 이 함수는 calculatePowerMetrics()를 호출 안함
      break;
    case SCREEN_THD: // [v18] THD 계산 및 표시 함수 호출
      runTHDCalculation();
      break;
    case SCREEN_WARNING:
      break;
  }
}

// ==============================================================================
// 3. 터치 입력 확인 함수 (v17과 동일)
// ==============================================================================
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    if (currentScreen == SCREEN_WARNING) {
      warningActive = false;
      // [v17] 2개 릴레이 모두 리셋 (LOW = ON)
      digitalWrite(RELAY_1_PIN, LOW); 
      digitalWrite(RELAY_2_PIN, LOW); 
      currentScreen = SCREEN_MAIN_POWER;
      screenNeedsRedraw = true;
      delay(100); 
      return; 
    }

    if (currentScreen == SCREEN_MAIN_POWER) {
        currentScreen = SCREEN_PHASE_DIFFERENCE;
        Serial.println("Screen -> PHASE");
    } 
    else if (currentScreen == SCREEN_PHASE_DIFFERENCE) { 
        currentScreen = SCREEN_COMBINED_WAVEFORM;
        Serial.println("Screen -> COMBINED WAVEFORM"); 
    }
    else if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
        currentScreen = SCREEN_THD; // [v18] THD 화면으로 이동
        Serial.println("Screen -> THD/FFT");
    }
    else if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
        currentScreen = SCREEN_MAIN_POWER;
        Serial.println("Screen -> MAIN POWER");
    }
    
    screenNeedsRedraw = true;
    delay(100);
  }
}

// ==============================================================================
// 4. 헬퍼 함수 (수정 없음)
// ==============================================================================

// printTFTValue (float)
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
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
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(x, y);
  tft.print(prev_value);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(value);
}

// displayNetworkStatus
void displayNetworkStatus() {
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(180, 5);
  tft.print("NET: OFF");
}

// ==============================================================================
// 5. 메인 전력 화면 그리기 (수정 없음)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  
  displayNetworkStatus();
  
  tft.setTextSize(2);
  tft.setCursor(10, 40); tft.setTextColor(ILI9341_BLUE); tft.println("V:");
  tft.setCursor(10, 70); tft.setTextColor(ILI9341_ORANGE); tft.println("I:");
  tft.setCursor(10, 100); tft.setTextColor(ILI9341_DARKGREEN); tft.println("P:");
  tft.setCursor(10, 130); tft.setTextColor(ILI9341_ORANGE); tft.println("Q:");
  tft.setCursor(10, 160);
  tft.setTextColor(ILI9341_MAGENTA); tft.println("PF:");
  tft.setCursor(10, 190); tft.setTextColor(ILI9341_BLACK); tft.println("THD:");
  
  tft.setCursor(10, 280); tft.setTextSize(1); tft.setTextColor(ILI9341_DARKGREY);
  tft.print("Tap screen to switch view");
}

// ==============================================================================
// 6. 메인 전력 화면 "값" 업데이트 (수정 없음)
// ==============================================================================
void displayMainScreenValues() {
  tft.setTextSize(2);
  char buffer[20];

  // V_rms
  tft.fillRect(45, 40, 180, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLUE); 
  tft.setCursor(45, 40);
  dtostrf(V_rms, 4, 1, buffer);
  tft.print(buffer); tft.print(" V");

  // I_rms
  tft.fillRect(45, 70, 180, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(45, 70);
  if (I_rms < 1.0) {
    dtostrf(I_rms * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else {
    dtostrf(I_rms, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  // P_real
  tft.fillRect(45, 100, 180, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_DARKGREEN);
  tft.setCursor(45, 100);
  if (P_real >= 1000.0) {
    dtostrf(P_real / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kW");
  } else {
    dtostrf(P_real, 4, 1, buffer);
    tft.print(buffer); tft.print(" W");
  }

  // Q_reactive
  tft.fillRect(45, 130, 180, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(45, 130);
   if (Q_reactive >= 1000.0) {
    dtostrf(Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVAR");
  } else {
    dtostrf(Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" VAR");
  }

  // PF
  tft.fillRect(45, 160, 180, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.setCursor(45, 160);
  dtostrf(PF, 4, 2, buffer);
  tft.print(buffer);

  // THD
  tft.fillRect(60, 190, 170, 20, ILI9341_WHITE);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(60, 190);
  dtostrf(THD_value, 4, 1, buffer);
  tft.print(buffer); tft.print(" %");
}

// ==============================================================================
// 7. 위상차 화면 그리기 (수정 없음)
// ==============================================================================
void displayPhaseScreenStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("PHASE DIFFERENCE");
  
  displayNetworkStatus();
  
  tft.setTextSize(2);
  tft.setCursor(10, 70); tft.setTextColor(ILI9341_BLACK); tft.println("Phase:");
  tft.setCursor(10, 130); tft.setTextColor(ILI9341_BLACK); tft.println("Status:");
  
  tft.setCursor(10, 280); tft.setTextSize(1); tft.setTextColor(ILI9341_DARKGREY);
  tft.print("Tap screen to switch view");
  
  prev_phase_degrees = phase_degrees;
  prev_lead_lag_status = lead_lag_status;
}

// ==============================================================================
// 8. 위상차 화면 "값" 업데이트 함수 (수정 없음)
// ==============================================================================
void displayPhaseScreenValues() {
  tft.setTextSize(3);
  printTFTValue(10, 100, phase_degrees, prev_phase_degrees, 1, ILI9341_BLUE, " deg");
  printTFTValue(10, 160, lead_lag_status, prev_lead_lag_status, ILI9341_ORANGE);
  
  prev_phase_degrees = phase_degrees;
  prev_lead_lag_status = lead_lag_status;
}

// ==============================================================================
// 9. [v16] 공통 파형 화면 (정적 그리드 및 0점 라벨) - *** Test.ino 교체 ***
// ==============================================================================
void drawWaveformGridAndLabels() {
  // tft.fillScreen(ILI9341_WHITE); // 화면 전환 시 이미 지워짐
  
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("V/I WAVEFORM (Auto)");

  // Wattmeter.ino의 NET: OFF 표시 유지
  displayNetworkStatus();
  
  // --- 그래프 영역 ---
  
  // 1. 플롯 영역 테두리
  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), ILI9341_LIGHTGREY);
  
  // 2. 0점 기준선 (가로)
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, ILI9341_LIGHTGREY);
  
  // 3. Y축 (세로선)
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), ILI9341_DARKGREY); // 왼쪽 전류 축
  tft.drawFastVLine(PLOT_X_END, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), ILI9341_DARKGREY); // 오른쪽 전압 축

  
  // --- [수정] 정적 Y축 라벨 (0점) ---
  tft.setTextSize(1);
  
  // 0A
  tft.setTextColor(ILI9341_ORANGE);
  tft.setCursor(0, PLOT_Y_CENTER - 4); // 5 -> 0 (좌측 끝으로)
  tft.print("0A");
  
  // 0V
  tft.setTextColor(ILI9341_BLUE);
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4);
  tft.print("0V");

  // Wattmeter.ino의 하단 팁 유지
  tft.setCursor(10, SCREEN_HEIGHT - 12); // 280 -> SCREEN_HEIGHT - 12 (조정)
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.print("Tap screen to switch view");
}

// ==============================================================================
// 10. [v16] 동적 Y축 라벨 업데이트 함수 - *** Test.ino 신규 추가 ***
// ==============================================================================
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10];
  
  // --- 이전 라벨 지우기 ---
  // 왼쪽 (전류)
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, ILI9341_WHITE);
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, ILI9341_WHITE);
  // 오른쪽 (전압)
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, ILI9341_WHITE);
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, ILI9341_WHITE);

  // --- 새 라벨 그리기 ---

  // 4. 왼쪽: 전류(A) 축 (주황색)
  tft.setTextColor(ILI9341_ORANGE);
  
  // Wattmeter.ino의 I_rms < 1.0 (mA 표시) 로직을 여기에도 적용
  if (I_peak_dynamic < 1.0) {
    dtostrf(I_peak_dynamic * 1000, 3, 0, buffer);
    // +I peak (mA)
    tft.setCursor(0, PLOT_Y_START + 5);
    tft.print("+"); tft.print(buffer);
    // -I peak (mA)
    tft.setCursor(0, PLOT_Y_END - 10);
    tft.print("-"); tft.print(buffer);
  } else {
    dtostrf(I_peak_dynamic, 3, 1, buffer);
    // +I peak (A)
    tft.setCursor(0, PLOT_Y_START + 5);
    tft.print("+"); tft.print(buffer);
    // -I peak (A)
    tft.setCursor(0, PLOT_Y_END - 10);
    tft.print("-"); tft.print(buffer);
  }

  // 5. 오른쪽: 전압(V) 축 (파란색)
  tft.setTextColor(ILI9341_BLUE);
  dtostrf(V_peak_dynamic, 3, 0, buffer);
  
  // +V peak
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5);
  tft.print("+"); tft.print(buffer);
  
  // -V peak
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10);
  tft.print("-"); tft.print(buffer);
}


// ==============================================================================
// 11. [v2] 경고 팝업 화면 (수정 없음 - 번호만 10->11로 변경)
// ==============================================================================
void displayWarningScreenStatic() {
  tft.fillScreen(ILI9341_WHITE); 
  tft.drawRect(20, 50, 200, 220, ILI9341_RED);
  tft.drawRect(21, 51, 198, 218, ILI9341_RED); 
  
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(3); tft.setCursor(50, 80); tft.print("WARNING");
  
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2); tft.setCursor(40, 130); tft.print(warningMessage);
  
  tft.setTextSize(3);
  if (warningMessage.startsWith("OVER VOLTAGE")) {
    tft.setCursor(60, 170); tft.print(V_rms, 1); tft.print(" V");
  } else {
    tft.setCursor(60, 170); tft.print(I_rms, 2); tft.print(" A");
  }
  
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1); tft.setCursor(60, 240);
  tft.print("Tap screen to reset");
}


// ==============================================================================
// 12. 실시간 전력 계산 (메인/위상차 화면 공용) (v17과 동일)
// ==============================================================================
void calculatePowerMetrics() {
  
  // ( ... v16의 샘플링 및 계산 로직 (v17과 동일) ... )
  unsigned long V_sq_sum = 0;
  unsigned long I_sq_sum = 0;
  long P_sum = 0;
  int V_ac_max = 0;
  int I_ac_max = 0;
  int V_ac_bits_prev = 0;
  int I_ac_bits_prev = 0;
  long time_V_cross = -1;
  long time_I_cross = -1;
  bool found_V_cross = false;
  bool found_I_cross = false;
  unsigned long startTimePass = micros();

  if (I_rms < 0.05) { 
     // ( ... v17과 동일 ... )
  }
  
  // [v18] THD 값 업데이트
  // THD_value = 0.0; // [v18] 삭제
  THD_value = thd_v_value * 100.0; // THD-V 값을 %로 표시
  
  // --- [v17] 퍼지 로직 실행 ---
  runFuzzyLogic();

  for (int i = 0; i < SAMPLES_PER_CALC; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT; 
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits;
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits;
    P_sum += (long)V_ac_bits * I_ac_bits;
    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits);
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits);
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
    while(micros() - startTimePass < (i + 1) * SAMPLE_PERIOD_US);
    if (i % 20 == 0) {
      checkTouchInput();
      if (screenNeedsRedraw || warningActive) return;
    }
  }
  float V_rms_adc = sqrt((float)V_sq_sum / SAMPLES_PER_CALC);
  float I_rms_adc = sqrt((float)I_sq_sum / SAMPLES_PER_CALC);
  float P_avg_adc = (float)P_sum / SAMPLES_PER_CALC;
  V_rms = V_rms_adc * V_CALIB_RMS;
  I_rms = I_rms_adc * I_CALIB_RMS;
  P_real = P_avg_adc * V_CALIB_RMS * I_CALIB_RMS;
  V_rms -= V_RMS_OFFSET_CORRECTION;
  I_rms -= I_RMS_OFFSET_CORRECTION;
  if (V_rms < 0) V_rms = 0;
  if (I_rms < 0) I_rms = 0;
  V_peak_dynamic = (V_ac_max * V_CALIB_RMS) * 1.1;
  I_peak_dynamic = (I_ac_max * I_CALIB_RMS) * 1.1;
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
  if (found_V_cross && found_I_cross) {
    long time_diff = time_I_cross - time_V_cross;
    float period_us = 1000000.0 / FREQUENCY;
    phase_degrees = fmod(((float)time_diff / period_us) * 360.0, 360.0);
    if (phase_degrees > 180.0) phase_degrees -= 360.0;
    else if (phase_degrees < -180.0) phase_degrees += 360.0;
    if (phase_degrees > 2.0) { lead_lag_status = "Lag"; } 
    else if (phase_degrees < -2.0) { lead_lag_status = "Lead"; phase_degrees = -phase_degrees; }
    else { lead_lag_status = "---"; phase_degrees = 0.0; }
    float pf_angle = acos(abs(PF)) * (180.0 / M_PI);
    phase_degrees = pf_angle;
  } else {
    phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
    lead_lag_status = (abs(PF) > 0.98) ? "---" : "Lag/Lead";
  }
  if (I_rms < 0.05) { // 50mA 미만
     phase_degrees = 0.0; lead_lag_status = "---"; PF = 1.0; P_real = 0.0; Q_reactive = 0.0;
  }



  // --- [v17] 퍼지 로직 실행 ---
  runFuzzyLogic(); 
  // --- [v17] ---


  // ... (경고 로직) ...
  if (V_rms > VOLTAGE_THRESHOLD) {
   // [v17-2Relay] 메인 릴레이 대신 부하 1, 2를 동시 차단 (HIGH = TRIP)
    digitalWrite(RELAY_1_PIN, HIGH); 
    digitalWrite(RELAY_2_PIN, HIGH);
    warningMessage = "OVER VOLTAGE!";
    warningActive = true;
    screenNeedsRedraw = true;
  }
}

// ==============================================================================
// 13. 메인 화면 실행 함수 (v16과 동일)
// ==============================================================================
void runMainPowerCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayMainScreenValues();
}

// ==============================================================================
// 14. 위상차 화면 실행 함수 (v16과 동일)
// ==============================================================================
void runPhaseCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayPhaseScreenValues();
}

// ==============================================================================
// 15. [v16] 공통 파형 그리기 루프 함수 (v17과 동일)
// ==============================================================================
void runCombinedWaveformLoop() {
  // ( ... v16/v17의 파형 그리기 로직 (변경 없음) ... )
  float volts_to_pixels_scale = (V_peak_dynamic < 1.0) ? 
                                0 : (PLOT_HEIGHT_HALF / V_peak_dynamic);
  float amps_to_pixels_scale = (I_peak_dynamic < 0.1) ? 0 : (PLOT_HEIGHT_HALF / I_peak_dynamic);
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);
  int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
  int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
  float V_mains_instant = V_ac_bits * V_CALIB_RMS;
  float I_mains_instant = I_ac_bits * I_CALIB_RMS;
  if (abs(V_mains_instant) > frame_V_peak) {
    frame_V_peak = abs(V_mains_instant);
  }
  if (abs(I_mains_instant) > frame_I_peak) {
    frame_I_peak = abs(I_mains_instant);
  }
  int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
  y_pos_v = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
  int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
  y_pos_i = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
  int prev_x = (current_x == PLOT_X_START) ?
               (PLOT_X_END - 1) : (current_x - 1);
  tft.drawFastVLine(prev_x, PLOT_Y_START + 1, (PLOT_Y_END - PLOT_Y_START) - 2, ILI9341_WHITE);
  tft.drawPixel(prev_x, PLOT_Y_CENTER, ILI9341_LIGHTGREY);
  if (current_x > PLOT_X_START) { 
    tft.drawLine(current_x - 1, last_y_v, current_x, y_pos_v, ILI9341_BLUE);
    tft.drawLine(current_x - 1, last_y_i, current_x, y_pos_i, ILI9341_ORANGE);
  } else {
    int wrap_x = PLOT_X_END - 1;
    tft.drawLine(wrap_x, last_y_v, current_x, y_pos_v, ILI9341_BLUE); 
    tft.drawLine(wrap_x, last_y_i, current_x, y_pos_i, ILI9341_ORANGE);
  }
  last_y_v = y_pos_v;
  last_y_i = y_pos_i;
  current_x++;
  if (current_x >= PLOT_X_END) { 
    current_x = PLOT_X_START;   
    V_peak_dynamic = (frame_V_peak < 50.0) ? 50.0 : frame_V_peak * 1.1;
    I_peak_dynamic = (frame_I_peak < 1.0) ? 1.0 : frame_I_peak * 1.1;
    updateYAxisLabels();
    frame_V_peak = 0.0;
    frame_I_peak = 0.0;
  }
  delayMicroseconds(40);
}


// ==============================================================================
// 16. [v17 수정] 퍼지 로직 시스템 빌드 함수 (Fuzzy.h 문법 적용)
// ==============================================================================
void buildFuzzySystem() {
  // === 입력 1: Total Current (0A ~ 10A) ===
  // new FuzzySet(a, b, c, d) 문법 사용
  // (eFLL의 Trap(0,0,5,6) -> FuzzySet(0,0,5,6))
  safeCurrent = new FuzzySet(0, 0, 5, 6);
  // (eFLL의 Trian(5,6,7) -> FuzzySet(5,6,6,7))
  warningCurrent = new FuzzySet(5, 6, 6, 7);
  // (eFLL의 Trian(6.5, 7.25, 8) -> FuzzySet(6.5, 7.25, 7.25, 8))
  dangerCurrent = new FuzzySet(6.5, 7.25, 7.25, 8);
  // (eFLL의 Trap(7.5, 8, 10, 10) -> FuzzySet(7.5, 8, 10, 10))
  criticalCurrent = new FuzzySet(7.5, 8, 10, 10);

  totalCurrent->addFuzzySet(safeCurrent);
  totalCurrent->addFuzzySet(warningCurrent);
  totalCurrent->addFuzzySet(dangerCurrent);
  totalCurrent->addFuzzySet(criticalCurrent);
  fuzzy->addFuzzyInput(totalCurrent); // [v17 수정] 포인터 전달

  // === 입력 2: Current Change Rate (dI/dt) (-5A/s ~ +10A/s) ===
  stableChange = new FuzzySet(-2, -1, 1, 2);
  slowIncrease = new FuzzySet(1, 3, 3, 5);     // Trian(1,3,5)
  suddenSurge = new FuzzySet(4, 6, 10, 10);
  
  currentChangeRate->addFuzzySet(stableChange);
  currentChangeRate->addFuzzySet(slowIncrease);
  currentChangeRate->addFuzzySet(suddenSurge);
  fuzzy->addFuzzyInput(currentChangeRate); // [v17 수정] 포인터 전달

  // === 출력: Shutdown Level (0 ~ 10) ===
  level0 = new FuzzySet(0, 0, 0, 1);    // Trian(0,0,1)
  level3 = new FuzzySet(2, 3, 3, 4);    // Trian(2,3,4)
  level6 = new FuzzySet(5, 6, 6, 7);    // Trian(5,6,7)
  level10 = new FuzzySet(9, 10, 10, 10); // Trian(9,10,10) or Trap(9,10,10,10)

  shutdownLevel->addFuzzySet(level0);
  shutdownLevel->addFuzzySet(level3);
  shutdownLevel->addFuzzySet(level6);
  shutdownLevel->addFuzzySet(level10);
  fuzzy->addFuzzyOutput(shutdownLevel); // [v17 수정] 포인터 전달

  // === 규칙 (Rules) ===
  // (문법은 'Fuzzy.h' 예제와 동일)

  // Rule 1: IF (전류가 안전) AND (변화가 안정) THEN (Level 0)
  FuzzyRuleAntecedent* if_Safe_and_Stable = new FuzzyRuleAntecedent();
  if_Safe_and_Stable->joinWithAND(safeCurrent, stableChange);
  FuzzyRuleConsequent* then_Level0 = new FuzzyRuleConsequent();
  then_Level0->addOutput(level0);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_Safe_and_Stable, then_Level0));

  // Rule 2: IF (전류가 주의) AND (변화가 안정) THEN (Level 3)
  FuzzyRuleAntecedent* if_Warning_and_Stable = new FuzzyRuleAntecedent();
  if_Warning_and_Stable->joinWithAND(warningCurrent, stableChange);
  FuzzyRuleConsequent* then_Level3 = new FuzzyRuleConsequent();
  then_Level3->addOutput(level3);
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_Warning_and_Stable, then_Level3));

  // Rule 3: IF (전류가 위험) AND (변화가 안정) THEN (Level 6)
  FuzzyRuleAntecedent* if_Danger_and_Stable = new FuzzyRuleAntecedent();
  if_Danger_and_Stable->joinWithAND(dangerCurrent, stableChange);
  FuzzyRuleConsequent* then_Level6 = new FuzzyRuleConsequent();
  then_Level6->addOutput(level6);
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_Danger_and_Stable, then_Level6));

  // Rule 4: IF (전류가 주의) AND (변화가 서서히 증가) THEN (Level 6)
  FuzzyRuleAntecedent* if_Warning_and_Slow = new FuzzyRuleAntecedent();
  if_Warning_and_Slow->joinWithAND(warningCurrent, slowIncrease);
  fuzzy->addFuzzyRule(new FuzzyRule(4, if_Warning_and_Slow, then_Level6)); // then_Level6 재사용
  
  // Rule 5: IF (전류가 임계) THEN (Level 10)
  FuzzyRuleAntecedent* if_Critical = new FuzzyRuleAntecedent();
  if_Critical->joinSingle(criticalCurrent); // [v17 수정] 'join' -> 'joinSingle' (Fuzzy.h 예제 기반)
  FuzzyRuleConsequent* then_Level10 = new FuzzyRuleConsequent();
  then_Level10->addOutput(level10);
  fuzzy->addFuzzyRule(new FuzzyRule(5, if_Critical, then_Level10));

  // Rule 6: IF (변화가 급격) THEN (Level 10)
  FuzzyRuleAntecedent* if_Surge = new FuzzyRuleAntecedent();
  if_Surge->joinSingle(suddenSurge); // [v17 수정] 'join' -> 'joinSingle'
  fuzzy->addFuzzyRule(new FuzzyRule(6, if_Surge, then_Level10)); // then_Level10 재사용
}


// ==============================================================================
// 17. [v17 수정] 퍼지 로직 실행 헬퍼 함수 (포인터 문법 적용)
// ==============================================================================
void runFuzzyLogic() {
  // 1. dI/dt 계산 (I_rms 기준)
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; // 초 단위

  if (deltaTime < 0.1) { 
    return; 
  }

  float dI_dt = (I_rms - last_I_rms) / deltaTime;
  
  last_I_rms = I_rms;
  lastFuzzyTime = currentTime;

  // 2. 퍼지 시스템 실행
  
  // 2-1. 입력값 설정 (Fuzzification)
  // [v17 수정] 'setValue'가 FuzzyInput에 없음. 'fuzzy->setInput' 사용.
  fuzzy->setInput(1, I_rms);
  fuzzy->setInput(2, dI_dt);

  // 2-2. 퍼지 추론 실행
  fuzzy->fuzzify(); // [v17 수정] -> 사용

  // 2-3. 출력값 계산 (Defuzzification)
  float outputLevel = fuzzy->defuzzify(1); // [v17 수정] -> 사용 (1번 출력)

  // 3. 릴레이 제어
  controlRelays(outputLevel);
}


// ==============================================================================
// 18. [v17] 퍼지 출력 -> 릴레이 제어 변환 함수 (v17과 동일)
// ==============================================================================
void controlRelays(float level) {
  // (가정: HIGH = TRIP/OFF, LOW = NORMAL/ON)
  // v17 - 경고 리셋 로직과 일치시킴

  if (level > 9.0) { // 임계 (Level 10)
    digitalWrite(RELAY_2_PIN, HIGH);      // 부하 2 TRIP
    digitalWrite(RELAY_1_PIN, HIGH);      // 부하 1 TRIP
    
    if (!warningActive) {
      warningMessage = "FUZZY LOGIC TRIP";
      warningActive = true;
      screenNeedsRedraw = true;
    }
    
  } 
  else if (level > 5.0) { // 위험 (Level 6)
    digitalWrite(RELAY_2_PIN, HIGH);      // 부하 2 TRIP
    digitalWrite(RELAY_1_PIN, HIGH);      // 부하 1 TRIP
  } 
  else if (level > 2.0) { // 주의 (Level 3)
 
    digitalWrite(RELAY_2_PIN, LOW);      // 부하 2 ON
    digitalWrite(RELAY_1_PIN, HIGH);      // 부하 1 TRIP
  } 
  else { // 안전 (Level 0)
    if (!warningActive) {
      digitalWrite(RELAY_2_PIN, LOW);      // 부하 2 ON
      digitalWrite(RELAY_1_PIN, LOW);      // 부하 1 ON
    }
  }
}

// ==============================================================================
// [v18] THD 화면 실행 함수 (신규)
// ==============================================================================
void runTHDCalculation() {
  // 1. 고속 샘플링, FFT, THD 계산 실행 (V/I 동시)
  // (이 함수는 실행에 수백 ms가 걸리는 '블로킹' 함수임)
  performFFT_and_CalcTHD();
  
  // 2. 화면 갱신이 필요하거나 경고가 뜨면 중단
  if (screenNeedsRedraw || warningActive) return;

  // 3. 계산된 THD 값을 화면에 표시
  displayTHDScreenValues();
}

// ==============================================================================
// [v18] THD 화면 정적 UI 그리기 (신규)
// ==============================================================================
void displayTHDScreenStatic() {
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("HARMONIC DISTORTION (THD)");
  
  displayNetworkStatus();
  
  tft.setTextSize(3);
  tft.setCursor(10, 70); tft.setTextColor(ILI9341_BLUE); tft.println("THD-V:");
  tft.setCursor(10, 150); tft.setTextColor(ILI9341_ORANGE); tft.println("THD-I:");
  
  tft.setCursor(10, SCREEN_HEIGHT - 12); tft.setTextSize(1); tft.setTextColor(ILI9341_DARKGREY);
  tft.print("Tap screen to switch view");
  
  // 이전 값 초기화
  prev_thd_v = thd_v_value;
  prev_thd_i = thd_i_value;
}

// ==============================================================================
// [v18] THD 화면 동적 값 업데이트 (신규)
// ==============================================================================
void displayTHDScreenValues() {
  tft.setTextSize(4);
  
  // THD-V (전압)
  printTFTValue(30, 110, thd_v_value * 100.0, prev_thd_v * 100.0, 1, ILI9341_BLUE, " %");
  // THD-I (전류)
  printTFTValue(30, 190, thd_i_value * 100.0, prev_thd_i * 100.0, 1, ILI9341_ORANGE, " %");

  prev_thd_v = thd_v_value;
  prev_thd_i = thd_i_value;
}


// ==============================================================================
// [v18] CMSIS-DSP FFT 및 THD 계산 헬퍼 함수 (신규)
// ==============================================================================
float32_t calculateTHD(float32_t* mags, int fundamentalBin) {
  float32_t fundamental_power = mags[fundamentalBin] * mags[fundamentalBin];
  float32_t harmonics_power_sum = 0.0;

  if (fundamental_power == 0) return 0.0; // 0으로 나누기 방지

  // 2차부터 MAX_HARMONIC 차수까지 고조파의 제곱을 합산
  for (int n = 2; n <= MAX_HARMONIC; n++) {
    int binIndex = fundamentalBin * n;
    if (binIndex >= (FFT_N / 2)) {
      break; // 샘플링 주파수의 절반(나이퀴스트)을 넘어가면 중단
    }
    harmonics_power_sum += mags[binIndex] * mags[binIndex];
  }

  // THD = Sqrt( (V2^2 + V3^2 + ...) / V1^2 )
  return sqrt(harmonics_power_sum / fundamental_power);
}


// ==============================================================================
// [v18] 메인 FFT/THD 연산 함수 (신규)
// ==============================================================================
void performFFT_and_CalcTHD() {
  unsigned long startTime = micros();

  // --- 1. 고속 샘플링 (V/I 동시) ---
  // (R4의 analogRead()는 매우 빠르므로 130µs 주기를 맞출 수 있음)
  for (int i = 0; i < FFT_N; i++) {
    // [중요] v16의 V/I 동시 샘플링 지연 문제가 여기에도 동일하게 존재합니다.
    // (정확도를 높이려면 ADC Free-running 모드가 필요하지만, 일단 기능 구현)
    v_samples[i] = (float32_t)(analogRead(VOLTAGE_PIN)) - V_ADC_MIDPOINT;
    i_samples[i] = (float32_t)(analogRead(CURRENT_PIN)) - I_ADC_MIDPOINT;
    
    // 샘플링 주기 맞추기
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US);
  }
  
  // (샘플링 중 터치 확인 - UI 반응성을 위해)
  checkTouchInput();
  if (screenNeedsRedraw || warningActive) return;

  // --- 2. Hann 윈도우 적용 (스펙트럼 누설 방지) ---
  // (CMSIS-DSP에 Hann 함수가 없으므로 직접 구현 또는 루프로 적용)
  // (간단하게 루프로 적용)
  for (int i = 0; i < FFT_N; i++) {
    float32_t window_factor = 0.5f - 0.5f * arm_cos_f32(2.0f * PI * i / (FFT_N - 1));
    v_samples[i] = v_samples[i] * window_factor;
    i_samples[i] = i_samples[i] * window_factor;
  }
  
  // --- 3. FFT 연산 (전압) ---
  // 입력: v_samples, 출력: v_fft_output
  arm_rfft_fast_f32(&fft_inst_v, v_samples, v_fft_output, 0); 
  
  // --- 4. FFT 크기 계산 (전압) ---
  // (결과는 N/2 개 만큼만 유효함)
  // 입력: v_fft_output, 출력: v_mags
  arm_cmplx_mag_f32(v_fft_output, v_mags, FFT_N / 2);

  // --- 5. FFT 연산 (전류) ---
  arm_rfft_fast_f32(&fft_inst_i, i_samples, i_fft_output, 0);

  // --- 6. FFT 크기 계산 (전류) ---
  arm_cmplx_mag_f32(i_fft_output, i_mags, FFT_N / 2);

  // (연산 중 터치 확인 - UI 반응성을 위해)
  checkTouchInput();
  if (screenNeedsRedraw || warningActive) return;

  // --- 7. THD 계산 ---
  thd_v_value = calculateTHD(v_mags, FUNDAMENTAL_BIN);
  thd_i_value = calculateTHD(i_mags, FUNDAMENTAL_BIN);
  
  // [v18] 메인 화면용 THD 값(THD_value)도 여기서 갱신
  THD_value = thd_v_value * 100.0;
}
