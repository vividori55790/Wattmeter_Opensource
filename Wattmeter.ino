/*
 * ==============================================================================
 * [v16 - Test.ino 파형 로직 통합]
 * v15의 파형 UI(drawWaveformGridAndLabels, runCombinedWaveformLoop)를
 * Test.ino의 '지속적인 자동 스케일링' 및 '좌우 분리 Y축' 로직으로 교체합니다.
 *
 * * v16 변경 사항:
 * 1. [UI 변경] SCREEN_COMBINED_WAVEFORM 화면이 Test.ino의 UI로 변경됩니다.
 * - 좌측(전류), 우측(전압)에 Y축 라벨이 표시됩니다.
 * - 파형이 그려지는 중에도 실시간으로 피크값을 추적하여 스케일이 자동 조절됩니다.
 * 2. [함수 추가] updateYAxisLabels() 함수가 Test.ino로부터 추가되었습니다.
 * 3. [정의 변경] v15의 V_PLOT/I_PLOT 관련 정의가 삭제되고,
 * Test.ino의 PLOT_X/Y 관련 정의가 Wattmeter.ino의 240x320 화면에 맞게
 * 조정되어 추가되었습니다.
 * 4. [로직 유지] v15의 RMS 계산 로직 및 보정값은 그대로 유지됩니다.
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
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, // [v16] Test.ino 로직으로 교체됨
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_MAIN_POWER;
volatile bool screenNeedsRedraw = true;

// --- [v2] 하드웨어 및 임계값 설정 ---
#define RELAY_PIN 2
#define VOLTAGE_THRESHOLD 240.0
#define CURRENT_THRESHOLD 10.0

volatile bool warningActive = false;
String warningMessage = "";

// --- 핀 정의 (센서) ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A4

// --- 핀 정의 (Display & Touch) ---
#define TFT_CS    10
#define TFT_DC    9
#define TFT_RST   8
#define TOUCH_CS  7

// ==============================================================================
// --- [v9.1] 수동 RMS 물리량 보정 (v15 롤백) ---
// ==============================================================================
const float V_RMS_OFFSET_CORRECTION = 7.1; // V_rms 오프셋 보정값
const float I_RMS_OFFSET_CORRECTION = 2.5546; // I_rms 오프셋 보정값

// 파형 표시를 위한 이론적 0점
const float V_ADC_MIDPOINT = 8192.0;
const float I_ADC_MIDPOINT = 8192.0;
// ==============================================================================

// --- 보정 계수 (Calibration Factors) ---
const float V_CALIB_RMS = 0.1775;
const float I_CALIB_RMS = 0.0482;  

// --- RMS 계산 설정 ---
#define SAMPLES_PER_CALC 334
#define SAMPLE_PERIOD_US 1000
const float FREQUENCY = 60.0;

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

// --- [v16] 공통 파형 디스플레이 설정 (Test.ino 통합) ---
// Wattmeter.ino의 240x320 화면에 맞게 Test.ino 값 조정
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

#define PLOT_X_START 30  // 좌측 마진 (전류 축 라벨 공간)
#define PLOT_X_END 290  // 우측 마진 (전압 축 라벨 공간)
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START) // 실제 파형이 그려질 너비 (200px)

#define PLOT_Y_START 50  // 그래프 영역 시작 Y좌표
#define PLOT_Y_END 210 // 그래프 영역 끝 Y좌표
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2))
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
// --- [v15] 통합 그래프 설정 삭제 ---

// [v2] Auto-Gain을 위한 피크값 변수 (기본값)
float V_peak_dynamic = 360.0;
float I_peak_dynamic = 42.4;

// --- [v16] 자동 스케일링을 위한 현재 프레임의 피크 값 추적 변수 ---
float frame_V_peak = 0.0;
float frame_I_peak = 0.0;

// [v16] 파형 그리기를 위한 전역 변수 (Test.ino 기준)
int current_x = PLOT_X_START;
int last_y_v = PLOT_Y_CENTER;
int last_y_i = PLOT_Y_CENTER;

// --- 실시간 계산값 저장을 위한 전역 변수 ---
float V_rms = 0.0;
float I_rms = 0.0;
float P_real = 0.0;
float Q_reactive = 0.0;
float S_apparent = 0.0;
float PF = 0.0;
float THD_value = 0.0;
float phase_degrees = 0.0;
String lead_lag_status = "---";

// --- 깜빡임 제거를 위한 이전 값 저장 변수 ---
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";

// --- [v16] 함수 프로토타입 선언 ---
void updateYAxisLabels();


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Wattmeter Booting...");

  analogReadResolution(14);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); 

  tft.begin();
  tft.setRotation(3); 
  
  ts.begin(SPI);
  ts.setRotation(3);
  
  Serial.println("TFT & Touch OK");
  // [v9.1] 수동 RMS 물리량 보정값 사용
  Serial.print("Using V_RMS_OFFSET_CORRECTION: "); Serial.println(V_RMS_OFFSET_CORRECTION);
  Serial.print("Using I_RMS_OFFSET_CORRECTION: "); Serial.println(I_RMS_OFFSET_CORRECTION);

  screenNeedsRedraw = true;
}

// ==============================================================================
// 2. Main Loop (상태 머신 기반)
// ==============================================================================
void loop() {
  
  checkTouchInput();
  
  if (screenNeedsRedraw) {
    tft.fillScreen(ILI9341_WHITE); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic();
    } else {
      // [v16] Test.ino 로직은 이 부분이 필요 없음 (case에서 처리)
      // current_x = 0; 
      
      // [v16] v15의 파형 화면 진입 시 첫 좌표 계산 로직 삭제
      
      // 그릴 화면의 정적 요소(제목, 그리드) 그리기
      switch(currentScreen) {
        case SCREEN_MAIN_POWER:
          displayMainScreenStatic();
          break;
        case SCREEN_PHASE_DIFFERENCE:
          displayPhaseScreenStatic();
          break;
        case SCREEN_COMBINED_WAVEFORM:
          drawWaveformGridAndLabels(); // [v16] 교체된 함수
          updateYAxisLabels();         // [v16] Test.ino의 초기 라벨 그리기
          // [v16] Test.ino의 좌표 초기화 로직
          current_x = PLOT_X_START;
          last_y_v = PLOT_Y_CENTER;
          last_y_i = PLOT_Y_CENTER;
          // [v16] 프레임 피크값 초기화
          frame_V_peak = 0.0;
          frame_I_peak = 0.0;
          break;
        case SCREEN_WARNING:
          // displayWarningScreenStatic()이 이미 처리함
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
      runCombinedWaveformLoop(); // [v16] 교체된 함수
      break;
    case SCREEN_WARNING:
      // 경고 화면에서는 아무 작업도 하지 않음 (터치 대기)
      break;
  }
}

// ==============================================================================
// 3. 터치 입력 확인 함수 (수정 없음)
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
      digitalWrite(RELAY_PIN, LOW); 
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
// 12. 실시간 전력 계산 (메인/위상차 화면 공용) - (수정 없음 - 번호만 11->12)
// ==============================================================================
void calculatePowerMetrics() {
  
  // [v9.1] 이론상 0점(8192)을 기준으로 계산
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

  for (int i = 0; i < SAMPLES_PER_CALC; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);

    // [v9.1] 이론상 0점(MIDPOINT)을 기준으로 AC 성분 추출
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT; 

    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits;
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits;
    P_sum += (long)V_ac_bits * I_ac_bits;

    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits);
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits);
    
    // ... (위상차 제로 크로싱 감지) ...
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
  
  // [v9.1] 최종 계산된 RMS 값에서 수동 오프셋 값을 뺍니다.
  V_rms -= V_RMS_OFFSET_CORRECTION;
  I_rms -= I_RMS_OFFSET_CORRECTION;
  
  if (V_rms < 0) V_rms = 0;
  if (I_rms < 0) I_rms = 0;
  
  // [v2] 파형 스케일링을 위한 피크값 업데이트
  // [v16] 이 값은 이제 Test.ino의 파형 로직에서는 기본값으로만 사용됩니다.
  V_peak_dynamic = (V_ac_max * V_CALIB_RMS) * 1.1;
  I_peak_dynamic = (I_ac_max * I_CALIB_RMS) * 1.1;
  if (V_peak_dynamic < 50.0) V_peak_dynamic = 50.0;
  if (I_peak_dynamic < 1.0) I_peak_dynamic = 1.0;

  S_apparent = V_rms * I_rms;
  
  if (S_apparent < 0.01) {
    PF = 0.0;
    P_real = 0.0;
    Q_reactive = 0.0;
  } else {
    PF = P_real / S_apparent;
    if (PF > 1.0) PF = 1.0;
    if (PF < -1.0) PF = -1.0; 
    Q_reactive = sqrt(max(0.0, S_apparent * S_apparent - P_real * P_real));
  }
  
  // ... (위상차 로직) ...
  if (found_V_cross && found_I_cross) {
    long time_diff = time_I_cross - time_V_cross;
    float period_us = 1000000.0 / FREQUENCY;
    phase_degrees = fmod(((float)time_diff / period_us) * 360.0, 360.0);
    
    if (phase_degrees > 180.0) phase_degrees -= 360.0;
    else if (phase_degrees < -180.0) phase_degrees += 360.0;

    if (phase_degrees > 2.0) {
        lead_lag_status = "Lag";
    } else if (phase_degrees < -2.0) {
        lead_lag_status = "Lead";
        phase_degrees = -phase_degrees;
    } else {
        lead_lag_status = "---";
        phase_degrees = 0.0;
    }
    // PF 각도로 위상차 보정
    float pf_angle = acos(abs(PF)) * (180.0 / M_PI);
    phase_degrees = pf_angle;
    
  } else {
    // 제로 크로싱 실패 시 PF 기반으로 계산
    phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
    lead_lag_status = (abs(PF) > 0.98) ? "---" : "Lag/Lead";
  }
  
  if (I_rms < 0.05) { // 50mA 미만
      phase_degrees = 0.0;
      lead_lag_status = "---";
      PF = 1.0;
      P_real = 0.0;
      Q_reactive = 0.0;
  }

  THD_value = 0.0; // THD 계산은 구현되지 않음

  // ... (경고 로직) ...
  if (V_rms > VOLTAGE_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH);
    warningMessage = "OVER VOLTAGE!";
    warningActive = true;
    screenNeedsRedraw = true;
  }
  else if (I_rms > CURRENT_THRESHOLD) {
    digitalWrite(RELAY_PIN, HIGH);
    warningMessage = "OVER CURRENT!";
    warningActive = true;
    screenNeedsRedraw = true;
  }
}

// ==============================================================================
// 13. 메인 화면 실행 함수 (수정 없음 - 번호만 12->13)
// ==============================================================================
void runMainPowerCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayMainScreenValues();
}

// ==============================================================================
// 14. 위상차 화면 실행 함수 (수정 없음 - 번호만 13->14)
// ==============================================================================
void runPhaseCalculation() {
  calculatePowerMetrics();
  if (screenNeedsRedraw || warningActive) return;
  displayPhaseScreenValues();
}

// ==============================================================================
// 15. [v16] 공통 파형 그리기 루프 함수 (자동 스케일링) - *** Test.ino 교체 ***
// ==============================================================================
void runCombinedWaveformLoop() {
  // 1. 현재 스케일 값으로 스케일 팩터 계산
  float volts_to_pixels_scale = (V_peak_dynamic < 1.0) ? 
                                0 : (PLOT_HEIGHT_HALF / V_peak_dynamic);
  float amps_to_pixels_scale = (I_peak_dynamic < 0.1) ? 0 : (PLOT_HEIGHT_HALF / I_peak_dynamic);
  
  // 2. 샘플링
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);
  
  // 3. 이론상 0점(MIDPOINT)을 기준으로 AC 성분 추출 (Wattmeter.ino 변수 사용)
  int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
  int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
  
  // 4. 순시값 계산 (Wattmeter.ino 변수 사용)
  float V_mains_instant = V_ac_bits * V_CALIB_RMS;
  float I_mains_instant = I_ac_bits * I_CALIB_RMS;

  // 5. [추가] 자동 스케일링을 위한 현재 프레임의 피크 값 추적
  if (abs(V_mains_instant) > frame_V_peak) {
    frame_V_peak = abs(V_mains_instant);
  }
  if (abs(I_mains_instant) > frame_I_peak) {
    frame_I_peak = abs(I_mains_instant);
  }
  
  // 6. Y 좌표 계산 (공통 축 기준)
  int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
  y_pos_v = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
  
  int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
  y_pos_i = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
  
  // 7. 이전 X좌표 지우기 (공통 축 기준)
  int prev_x = (current_x == PLOT_X_START) ?
               (PLOT_X_END - 1) : (current_x - 1);
  tft.drawFastVLine(prev_x, PLOT_Y_START + 1, (PLOT_Y_END - PLOT_Y_START) - 2, ILI9341_WHITE);
  tft.drawPixel(prev_x, PLOT_Y_CENTER, ILI9341_LIGHTGREY);
  
  // 8. 파형 그리기 (현재 x 좌표 기준)
  if (current_x > PLOT_X_START) { 
    tft.drawLine(current_x - 1, last_y_v, current_x, y_pos_v, ILI9341_BLUE);
    tft.drawLine(current_x - 1, last_y_i, current_x, y_pos_i, ILI9341_ORANGE);
  } else {
    // x가 시작점일 때, x=끝점과 연결 (화면 래핑)
    int wrap_x = PLOT_X_END - 1;
    tft.drawLine(wrap_x, last_y_v, current_x, y_pos_v, ILI9341_BLUE); 
    tft.drawLine(wrap_x, last_y_i, current_x, y_pos_i, ILI9341_ORANGE);
  }
  
  // 9. 마지막 좌표 업데이트
  last_y_v = y_pos_v;
  last_y_i = y_pos_i;
  
  // 10. X 좌표 증가 및 래핑
  current_x++;
  if (current_x >= PLOT_X_END) { // 플롯 영역 끝에 도달하면
    current_x = PLOT_X_START;   // 플롯 영역 시작점으로
    
    // --- [추가] 한 프레임이 끝났으므로, 피크 값을 갱신하고 라벨을 다시 그림 ---
    
    // 1. 동적 피크 값 갱신 (10% 헤드룸 추가 및 최소값 보장)
    // (calculatePowerMetrics에서 계산된 값을 사용하지 않고, 자체 추적 값 사용)
    V_peak_dynamic = (frame_V_peak < 50.0) ? 50.0 : frame_V_peak * 1.1;
    I_peak_dynamic = (frame_I_peak < 1.0) ? 1.0 : frame_I_peak * 1.1;
    
    // 2. Y축 라벨 갱신
    updateYAxisLabels();
    
    // 3. 다음 프레임을 위한 피크 값 추적기 리셋
    frame_V_peak = 0.0;
    frame_I_peak = 0.0;
    // --- [추가] ---
  }
  
  delayMicroseconds(40); // 파형 표시 속도 조절 (원본 값 유지)
}