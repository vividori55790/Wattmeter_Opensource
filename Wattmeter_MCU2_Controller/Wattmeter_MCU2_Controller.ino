/*
 * ==============================================================================
 * Controller.ino (디스플레이 및 입력 헤드)
 *
 * 이 스케치는 Wattmeter 프로젝트의 '얼굴' 역할을 합니다.
 * 모든 사용자 인터페이스(TFT 디스플레이, 터치스크린 입력)를 전담합니다.
 * 이 스케치에는 센서 핀 정의, 계산 함수, 로직 함수가 없습니다.
 * * 통신:
 * - RX: Processor로부터 메인 데이터 패킷 (JSON) 또는 파형 데이터 스트림 (Text)을 수신합니다.
 * - TX: 사용자 입력(터치)에 따라 Processor에게 명령 (JSON)을 전송합니다.
 * ==============================================================================
 */
/*
 * ==============================================================================
 * v60 변경 사항 (2025-11-18):
 * 1. 설정(Settings) 메뉴 재구성:
 * - ScreenState: SCREEN_SETTINGS_ADVANCED, SCREEN_SETTINGS_TIMER 추가 .
 * - displaySettingsScreenStatic(): 4버튼 -> 3버튼 (CALIBRATION, PROTECTION, ADVANCED)으로 변경 .
 * - displaySettingsAdvancedStatic(): [THEME], [TIMER], [RESET] 버튼을 포함하는 신규 화면 추가 .
 * - checkTouchInput(): '뒤로 가기' 로직 및 신규 화면/버튼 터치 로직 전면 수정 .
 * 2. 릴레이 타이머 UI 추가:
 * - displaySettingsTimerStatic()/Values(): 타이머 설정/표시 화면 추가 .
 * - checkTouchInput() (TIMER): "SET_TIMER" (JSON) 명령을 Processor로 전송 .
 * - checkSerialInput(): Processor로부터 "T_ACTIVE", "T_LEFT_S", "T_MIN" 상태를 수신하여 전역 변수에 저장 .
 * ==============================================================================
 */
 
// --- 라이브러리 포함 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <arm_math.h> // acos() 등 UI 계산에 필요
#include <ArduinoJson.h> // 시리얼 통신용

// --- [v20] 화면 상태 정의 (홈 화면 기반) ---
// [수정] SCREEN_SETTINGS_ADVANCED, SCREEN_SETTINGS_TIMER 추가
enum ScreenState {
  SCREEN_HOME,              
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_THD,
  SCREEN_SETTINGS,          
  SCREEN_SETTINGS_CALIB,    
  SCREEN_SETTINGS_PROTECT,  
  SCREEN_RELAY_CONTROL,     
  SCREEN_SETTINGS_THEME,    
  SCREEN_SETTINGS_RESET,
  SCREEN_SETTINGS_ADVANCED, // [신규]
  SCREEN_SETTINGS_TIMER,    // [신규]
  SCREEN_CONFIRM_SAVE,      
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; 
volatile ScreenState previousScreen = SCREEN_HOME; 
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 (TFT 및 터치) ---
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7
// 릴레이 및 센서 핀 정의는 Processor.ino로 이동됨

// --- JSON 문서 (버퍼) ---
StaticJsonDocument<1024> rxJsonDoc; // 수신용 (데이터가 많음)
StaticJsonDocument<256> txJsonDoc;  // 전송용 (명령은 간단함)

// --- [v25] 다크 모드 및 테마 (원본과 동일) ---
bool isDarkMode = false;
uint16_t COLOR_BACKGROUND;
uint16_t COLOR_TEXT_PRIMARY;
uint16_t COLOR_TEXT_SECONDARY;
uint16_t COLOR_BUTTON;
uint16_t COLOR_BUTTON_TEXT;
uint16_t COLOR_BUTTON_OUTLINE;
uint16_t COLOR_GRID;
uint16_t COLOR_RED;
uint16_t COLOR_GREEN;
uint16_t COLOR_BLUE;
uint16_t COLOR_ORANGE;
uint16_t COLOR_MAGENTA;
uint16_t COLOR_DARKGREEN;

// --- [v20] 실시간 설정 변수 (Processor로부터 받은 값의 "사본") ---
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// --- [v25] 설정 임시 저장 변수 (원본과 동일) ---
float temp_VOLTAGE_THRESHOLD;
float temp_V_MULTIPLIER;
float temp_I_MULTIPLIER;
int temp_setting_step_index;

// --- [신규] 타이머 관련 전역 변수 (Processor 사본) ---
bool is_timer_active = false;
uint32_t timer_seconds_left = 0;
uint16_t timer_setting_minutes = 0; // Processor의 마스터 설정값
// --- [신규] 타이머 설정 화면용 임시 변수 ---
int temp_timer_setting_minutes = 0; // 사용자가 UI에서 수정하는 값
// --- [신규] 타이머 화면 업데이트용 이전 값 ---
bool prev_is_timer_active = false;
uint32_t prev_timer_seconds_left = 0;
int prev_temp_timer_setting_minutes = -1;


// --- [v20] 설정 화면용 이전 값 (원본과 동일) ---
float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;

bool settingsChanged = false; 

// --- 경고 상태 (Processor로부터 받은 값의 "사본") ---
volatile bool warningActive = false;
String warningMessage = "";

// --- 캘리브레이션 상수 (ADC 오프셋) ---
// Processor.ino로 이동됨

// --- 터치스크린 상수 (원본과 동일) ---
#define TS_RAW_X1 370
#define TS_RAW_Y1 450
#define TS_RAW_X2 3760
#define TS_RAW_Y2 3670

// --- 객체 생성 (원본과 동일) ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS); 
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);

// --- UI 상수 (원본과 동일) ---
#define SCREEN_WIDTH 320 
#define SCREEN_HEIGHT 240 

// --- 파형 화면 상수 (원본과 동일) ---
#define PLOT_X_START 37 
#define PLOT_X_END 285 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
#define WAVEFORM_SAMPLE_PERIOD_US 100
int last_frame_y_v[PLOT_WIDTH];
int last_frame_y_i[PLOT_WIDTH];

// [v24] 파형 화면 Y축 범위 (원본과 동일)
const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
float V_axis_max = V_AXIS_STEPS[4]; 
float I_axis_max = I_AXIS_STEPS[2]; 

// --- [v19] 페이저 다이어그램 상수 (원본과 동일) ---
#define PHASOR_CX 235
#define PHASOR_CY 115
#define PHASOR_RADIUS 75

// --- 전역 변수 (물리량 - Processor로부터 받은 "사본") ---
float V_rms = 0.0;
float I_rms = 0.0; 
float I_rms_load1 = 0.0;
float I_rms_load2 = 0.0;
float P_real = 0.0; 
float Q_reactive = 0.0; 
float S_apparent = 0.0;
float PF = 0.0; 

// --- 전역 변수 (위상 - Processor로부터 받은 "사본") ---
float phase_degrees = 0.0; // PF로 로컬 계산
String lead_lag_status = "---"; 
float phase_main_deg = 0.0;
float phase_load1_deg = 0.0;
float phase_load2_deg = 0.0;
float thd_v_value = 0.0;
float32_t thd_i_value = 0.0;

// --- 릴레이 상태 (Processor로부터 받은 "사본") ---
bool relay1_state = false;
bool relay2_state = false;
bool prev_r1_state = false; // 릴레이 화면 업데이트용
bool prev_r2_state = false; // 릴레이 화면 업데이트용

// --- 전역 변수 (이전 값 - 화면 클리어용) (원본과 동일) ---
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
// Processor.ino로 이동됨
float32_t prev_thd_v = 0.0; // 화면 표시용 이전 값은 유지
float32_t prev_thd_i = 0.0; // 화면 표시용 이전 값은 유지

// --- 퍼지 로직 변수 ---
// Processor.ino로 이동됨

// --- [v23] 설정 화면 UI용 전역 변수 (원본과 동일) ---
float setting_steps[] = {0.0001, 0.01, 0.1, 1.0, 10.0};
const int NUM_SETTING_STEPS = 5;
int setting_step_index = DEFAULT_SETTING_STEP_INDEX;
int prev_setting_step_index = -1;

int calib_selection = 0;
int prev_calib_selection = -1;
const int NUM_CALIB_SETTINGS = 3; 

int protect_selection = 0;
int prev_protect_selection = -1;
const int NUM_PROTECT_SETTINGS = 2; 

// --- 함수 프로토타입 (UI 및 통신 관련) ---
void updateYAxisLabels();
void drawBackButton(); 
void displayHomeScreenStatic(); 
void displaySettingsScreenStatic(); 
void displaySettingsCalibStatic(); 
void displaySettingsProtectStatic(); 
void runSettingsCalib(); 
void runSettingsProtect(); 
void displaySettingsCalibValues(); 
void displaySettingsProtectValues(); 
void displayRelayControlStatic(); 
void runRelayControl(); 
void adjustCalibValue(bool increase); 
void adjustProtectValue(bool increase); 
float findAxisStep(float peak, const float* steps, int num_steps); 
void setTheme(); 
void displayConfirmSaveStatic(); 
void displaySettingsResetStatic(); 
void displaySettingsThemeStatic(); 
void runSettingsTheme(); 

// --- [신규] 함수 프로토타입 ---
void displaySettingsAdvancedStatic();
void displaySettingsTimerStatic();
void displaySettingsTimerValues();

void displayMainScreenStatic();
void displayPhaseScreenStatic();
void drawWaveformGridAndLabels();
void displayTHDScreenStatic();
void displayWarningScreenStatic();

// --- 계산 함수는 제거됨 ---
// runMainPowerCalculation, runPhaseCalculation 등은
// display...Values() 함수로 대체되거나 수정됨

void displayMainScreenValues();
void displayPhaseScreenValues();
void displayTHDScreenValues();

void runCombinedWaveformLoop(); // 로직 대폭 수정

void checkSerialInput(); // 신규: 시리얼 입력 처리
void sendJsonCommand(String jsonString); // 신규: JSON 명령 전송 헬퍼

// =GA= 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Controller.ino Booting...");
  
  setTheme(); // [v25] 테마 설정
  
  // 센서 핀 설정 제거
  
  tft.begin();
  tft.setRotation(3); 
  
  ts.begin(SPI);
  ts.setRotation(3); 
  
  Serial.println("TFT & Touch OK");

  // 퍼지 로직 및 FFT 초기화 제거

  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. 항상 터치 입력을 확인합니다.
  checkTouchInput(); 
  
  // 2. 항상 시리얼 입력을 확인합니다.
  // (파형 화면이 아닐 때만 JSON 파싱)
  checkSerialInput();
  
  // 3. 화면을 새로 그려야 하는지 확인합니다.
  if (screenNeedsRedraw) { 
    tft.fillScreen(COLOR_BACKGROUND); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic(); 
    } else { 
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
          break;
        case SCREEN_THD:
          displayTHDScreenStatic(); 
          break;
        case SCREEN_SETTINGS:
          displaySettingsScreenStatic();
          break;
        case SCREEN_SETTINGS_CALIB:
          displaySettingsCalibStatic();
          prev_V_MULTIPLIER = -1.0; 
          prev_I_MULTIPLIER = -1.0;
          prev_setting_step_index = -1;
          break;
        case SCREEN_SETTINGS_PROTECT:
          displaySettingsProtectStatic();
          prev_VOLTAGE_THRESHOLD = -1.0;
          prev_setting_step_index = -1;
          break;
        case SCREEN_RELAY_CONTROL:
          displayRelayControlStatic();
          break;
        case SCREEN_SETTINGS_THEME: 
          displaySettingsThemeStatic();
          break;
        case SCREEN_SETTINGS_RESET: 
          displaySettingsResetStatic();
          break;
        // --- [신규] 고급 설정 및 타이머 화면 ---
        case SCREEN_SETTINGS_ADVANCED:
          displaySettingsAdvancedStatic();
          break;
        case SCREEN_SETTINGS_TIMER:
          displaySettingsTimerStatic();
          // 값 강제 업데이트를 위해 prev 값 초기화
          prev_is_timer_active = !is_timer_active;
          prev_timer_seconds_left = -1;
          prev_temp_timer_setting_minutes = -1;
          break;
        // ---
        case SCREEN_CONFIRM_SAVE: 
          displayConfirmSaveStatic();
          break;
        case SCREEN_WARNING: 
          break;
      }
    }
    screenNeedsRedraw = false; 
  }
  
  // 4. 동적 UI 업데이트 (계산 로직 대신, 수신된 전역 변수를 표시)
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      // runMainPowerCalculation() 대신
      displayMainScreenValues(); 
      break;
    case SCREEN_PHASE_DIFFERENCE:
      // runPhaseCalculation() 대신
      displayPhaseScreenValues();
      break;
    case SCREEN_COMBINED_WAVEFORM:
      // waitForVoltageZeroCross() 제거됨
      // runCombinedWaveformLoop()는 시리얼 스트림을 읽도록 수정됨
      runCombinedWaveformLoop();
      break;
    case SCREEN_THD:
      // runTHDCalculation() 대신
      displayTHDScreenValues(); 
      break;
    case SCREEN_SETTINGS_CALIB:
      // runSettingsCalib() -> displaySettingsCalibValues()
      displaySettingsCalibValues(); 
      break;
    case SCREEN_SETTINGS_PROTECT:
      // runSettingsProtect() -> displaySettingsProtectValues()
      displaySettingsProtectValues(); 
      break;
    case SCREEN_RELAY_CONTROL: 
      runRelayControl(); // 릴레이 상태 변경 감지
      break;
    case SCREEN_SETTINGS_THEME: 
      runSettingsTheme();
      break;
    // --- [신규] 타이머 값 업데이트 ---
    case SCREEN_SETTINGS_TIMER:
      displaySettingsTimerValues();
      break;
    // ---
    case SCREEN_HOME: 
    case SCREEN_SETTINGS:
    case SCREEN_SETTINGS_RESET:
    case SCREEN_SETTINGS_ADVANCED: // 정적 화면
    case SCREEN_CONFIRM_SAVE: 
    case SCREEN_WARNING: 
      break;
  }
}


// ==============================================================================
// 3. [신규] 시리얼 입력 확인 (Processor -> Controller)
// ==============================================================================
void checkSerialInput() {
  if (Serial.available() == 0) return;
  
  // 파형 화면은 이 함수에서 처리하지 않고,
  // runCombinedWaveformLoop()에서 직접 텍스트 스트림을 읽습니다.
  if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
    return;
  }

  // 그 외의 모든 화면은 JSON 데이터 패킷을 기대합니다.
  String line = Serial.readStringUntil('\n');
  if (line.length() == 0) return;

  // Serial.print("C: RX "); Serial.println(line); // 디버깅

  DeserializationError error = deserializeJson(rxJsonDoc, line);
  if (error) {
    Serial.print("C: deserializeJson() failed: "); 
    Serial.println(error.c_str());
    return;
  }

  const char* type = rxJsonDoc["TYPE"];
  if (type == NULL || strcmp(type, "DATA") != 0) {
    return; // "DATA" 타입 패킷이 아니면 무시
  }
  
  // --- 메인 데이터 파싱 ---
  // (값이 없으면 기본값 유지)
  V_rms = rxJsonDoc["V"] | V_rms;
  I_rms = rxJsonDoc["I"] | I_rms;
  I_rms_load1 = rxJsonDoc["I1"] | I_rms_load1;
  I_rms_load2 = rxJsonDoc["I2"] | I_rms_load2;
  P_real = rxJsonDoc["P"] | P_real;
  Q_reactive = rxJsonDoc["Q"] | Q_reactive;
  S_apparent = rxJsonDoc["S"] | S_apparent;
  PF = rxJsonDoc["PF"] | PF;
  phase_main_deg = rxJsonDoc["PH_M"] | phase_main_deg;
  phase_load1_deg = rxJsonDoc["PH_1"] | phase_load1_deg;
  phase_load2_deg = rxJsonDoc["PH_2"] | phase_load2_deg;
  lead_lag_status = rxJsonDoc["LL"] | "---";
  thd_v_value = rxJsonDoc["THD_V"] | thd_v_value;
  thd_i_value = rxJsonDoc["THD_I"] | thd_i_value;

  // 설정값 동기화
  V_MULTIPLIER = rxJsonDoc["V_MULT"] | V_MULTIPLIER;
  I_MULTIPLIER = rxJsonDoc["I_MULT"] | I_MULTIPLIER;
  VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"] | VOLTAGE_THRESHOLD;
  setting_step_index = rxJsonDoc["STEP_IDX"] | setting_step_index;

  // 릴레이 상태 동기화
  relay1_state = rxJsonDoc["R1"]; // | 연산자 사용 불가 (bool)
  relay2_state = rxJsonDoc["R2"];

  // --- [신규] 타이머 상태 수신 ---
  is_timer_active = rxJsonDoc["T_ACTIVE"];
  timer_seconds_left = rxJsonDoc["T_LEFT_S"] | 0;
  timer_setting_minutes = rxJsonDoc["T_MIN"] | 0;
  
  // 경고 상태 동기화
  bool newWarning = rxJsonDoc["WARN"];
  
  if (newWarning && !warningActive) {
    // Processor가 경고를 보냄 -> 경고 화면 표시
    warningMessage = rxJsonDoc["MSG"] | "WARNING!";
    warningActive = true;
    screenNeedsRedraw = true; // 경고 화면 강제 표시
  } else if (!newWarning && warningActive) {
    // Processor가 경고를 해제함 -> 홈 화면으로
    warningActive = false;
    currentScreen = SCREEN_HOME;
    screenNeedsRedraw = true; // 홈 화면 강제 표시
  }
}

// ==============================================================================
// 4. [신규] JSON 명령 전송 헬퍼 (Controller -> Processor)
// ==============================================================================
void sendJsonCommand(String jsonString) {
  Serial.print(jsonString); // txJsonDoc을 사용하지 않고 간단히 문자열 전송
  Serial.println(); // JSON 패킷의 끝을 알림
}


// ==============================================================================
// 5. [수정] 터치 입력 확인 함수
// ==============================================================================
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    p.x = map(p.x, TS_RAW_X1, TS_RAW_X2, SCREEN_WIDTH, 0);
    p.y = map(p.y, TS_RAW_Y1, TS_RAW_Y2, SCREEN_HEIGHT, 0);
    
    // --- [수정] 공통 경고 화면 터치 ---
    if (currentScreen == SCREEN_WARNING) { 
      // 로컬 상태를 변경하지 않음
      // Processor에게 경고 해제 명령 전송
      sendJsonCommand("{\"CMD\":\"RESET_WARNING\"}");
      
      // warningActive = false; // 제거
      // digitalWrite(RELAY_1_PIN, LOW); // 제거
      // currentScreen = SCREEN_HOME; // 제거
      // screenNeedsRedraw = true; // 제거
      
      // 화면은 checkSerialInput()이 Processor의
      // '경고 해제' 데이터 패킷을 수신하면 자동으로 전환됨
      
      delay(100); // Debounce
      return; 
    }

    // --- [수정] 공통 뒤로 가기 버튼 (<) ---
    if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
      if (p.x >= 5 && p.x <= 55 && p.y >= 5 && p.y <= 35) {
        
        // --- [신규] 파형 화면에서 나갈 때 ---
        if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
          sendJsonCommand("{\"CMD\":\"REQ_WAVEFORM\", \"MODE\":0}");
          // 시리얼 버퍼에 남아있을 수 있는 'V,I' 텍스트 스트림을 비웁니다.
          while(Serial.available()) Serial.read();
        }
        // ---
        
        if (currentScreen == SCREEN_SETTINGS_CALIB || currentScreen == SCREEN_SETTINGS_PROTECT) {
          if (settingsChanged) { 
            previousScreen = currentScreen;
            currentScreen = SCREEN_CONFIRM_SAVE;
          } else { 
             currentScreen = SCREEN_SETTINGS;
          }
        } 
        // [수정] THEME, RESET, TIMER, CONFIRM 화면은 ADVANCED로 돌아감
        else if (currentScreen == SCREEN_SETTINGS_THEME || 
                 currentScreen == SCREEN_SETTINGS_RESET || 
                 currentScreen == SCREEN_SETTINGS_TIMER || 
                 currentScreen == SCREEN_CONFIRM_SAVE) {
          // [수정] CONFIRM_SAVE에서 돌아갈 때 DISCARD 로직이 이미 실행되었으므로
          // 추가 작업 없이 ADVANCED 화면으로 이동합니다.
          // (단, CONFIRM_SAVE의 'DISCARD' 로직은 SETTINGS로 감)
          if (currentScreen == SCREEN_CONFIRM_SAVE) {
             currentScreen = SCREEN_SETTINGS; // 이전 화면(CALIB/PROTECT)이 SETTINGS 소속이므로
          } else {
             currentScreen = SCREEN_SETTINGS_ADVANCED;
          }
        }
        // [신규] ADVANCED 화면은 SETTINGS로 돌아감
        else if (currentScreen == SCREEN_SETTINGS_ADVANCED) {
          currentScreen = SCREEN_SETTINGS;
        }
        else if (currentScreen == SCREEN_SETTINGS || currentScreen == SCREEN_RELAY_CONTROL) {
          currentScreen = SCREEN_HOME;
        }
        else {
          currentScreen = SCREEN_HOME;
        }
        
        screenNeedsRedraw = true;
        delay(100); // Debounce
        return;
      }
    }

    // --- [수정] 화면별 버튼 로직 ---
    switch (currentScreen) {
      case SCREEN_HOME:
        // ( ... 버튼 1, 2 ... )
        if (p.x >= 20 && p.x <= 150 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_MAIN_POWER;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_PHASE_DIFFERENCE;
          screenNeedsRedraw = true;
        }
        
        // --- [수정] 버튼 3: WAVEFORM ---
        else if (p.x >= 20 && p.x <= 150 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_COMBINED_WAVEFORM;
          screenNeedsRedraw = true;
          // Processor에게 파형 스트리밍 시작 명령 전송
          sendJsonCommand("{\"CMD\":\"REQ_WAVEFORM\", \"MODE\":1}");
        }
        // ( ... 버튼 4, 5, 6 ... )
        else if (p.x >= 170 && p.x <= 300 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_THD;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_RELAY_CONTROL; 
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS:
        // [수정] 3버튼 레이아웃 (CALIBRATION, PROTECTION, ADVANCED)
        if (p.x >= 20 && p.x <= 300 && p.y >= 60 && p.y <= 95) { // 60 + 35
          temp_V_MULTIPLIER = V_MULTIPLIER;
          temp_I_MULTIPLIER = I_MULTIPLIER;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_CALIB;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 105 && p.y <= 140) { // 105 + 35
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_PROTECT;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 150 && p.y <= 185) { // 150 + 35
          currentScreen = SCREEN_SETTINGS_ADVANCED;
          screenNeedsRedraw = true;
        }
        break;

      // --- [신규] 고급 설정 화면 ---
      case SCREEN_SETTINGS_ADVANCED:
        if (p.x >= 20 && p.x <= 300 && p.y >= 60 && p.y <= 95) { // THEME
          currentScreen = SCREEN_SETTINGS_THEME;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 105 && p.y <= 140) { // TIMER
          // 타이머 설정 화면 진입 시, Processor의 현재 설정값을 임시 변수로 복사
          temp_timer_setting_minutes = timer_setting_minutes;
          currentScreen = SCREEN_SETTINGS_TIMER;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 150 && p.y <= 185) { // RESET
          currentScreen = SCREEN_SETTINGS_RESET;
          screenNeedsRedraw = true;
        }
        break;

      // --- [신규] 타이머 설정 화면 ---
      case SCREEN_SETTINGS_TIMER:
        // "-10"
        if (p.x >= 20 && p.x <= 80 && p.y >= 100 && p.y <= 140) {
          temp_timer_setting_minutes -= 10;
          if (temp_timer_setting_minutes < 0) temp_timer_setting_minutes = 0;
        }
        // "+10"
        else if (p.x >= 90 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          temp_timer_setting_minutes += 10;
          if (temp_timer_setting_minutes > 999) temp_timer_setting_minutes = 999; // 3자리수 제한
        }
        // "SET"
        else if (p.x >= 170 && p.x <= 230 && p.y >= 100 && p.y <= 140) {
          sendJsonCommand("{\"CMD\":\"SET_TIMER\", \"MIN\":" + String(temp_timer_setting_minutes) + "}");
        }
        // "OFF"
        else if (p.x >= 240 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          temp_timer_setting_minutes = 0; // 임시 값도 0으로
          sendJsonCommand("{\"CMD\":\"SET_TIMER\", \"MIN\":0}");
        }
        break;

      case SCREEN_SETTINGS_CALIB:
        // ( ... UP/DOWN 버튼 원본과 동일 ... )
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) {
          calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) {
          calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS;
        }
        // --- [수정] [-] / [+] 버튼 ---
        // adjustCalibValue()가 Processor에게 명령을 전송하도록 수정됨
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) {
          adjustCalibValue(false); // Decrease
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustCalibValue(true); // Increase
        }
        break;
        
      case SCREEN_SETTINGS_PROTECT:
        // ( ... UP/DOWN 버튼 원본과 동일 ... )
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) {
          protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) {
          protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS;
        }
        // --- [수정] [-] / [+] 버튼 ---
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) {
          adjustProtectValue(false); // Decrease
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustProtectValue(true); // Increase
        }
        break;
        
      // --- [수정] 릴레이 제어 화면 터치 로직 ---
      case SCREEN_RELAY_CONTROL:
        // 버튼 1: Relay 1
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          // digitalWrite(...) 대신 명령 전송 (STATE: 2 = TOGGLE)
          sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":2}");
          // screenNeedsRedraw = true; // 제거 (데이터 패킷 수신 시 자동 업데이트)
        }
        // 버튼 2: Relay 2
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          // digitalWrite(...) 대신 명령 전송
          sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":2}");
          // screenNeedsRedraw = true; // 제거
        }
        break;

      // --- [수정] 설정 초기화 ---
      case SCREEN_SETTINGS_RESET:
        // [RESET]
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          // restoreDefaultSettings() 대신 명령 전송
          sendJsonCommand("{\"CMD\":\"RESET_SETTINGS\"}");
          currentScreen = SCREEN_SETTINGS_ADVANCED; // [수정] ADVANCED로 복귀
          screenNeedsRedraw = true;
        }
        // [CANCEL] (원본과 동일)
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          currentScreen = SCREEN_SETTINGS_ADVANCED; // [수정] ADVANCED로 복귀
          screenNeedsRedraw = true;
        }
        break;

      // --- [수정] 설정 저장 확인 ---
      case SCREEN_CONFIRM_SAVE:
        // [SAVE]
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          // 값은 이미 adjust...()에서 Processor로 전송되었음.
          // 로컬 임시 변수만 업데이트 (혹은 할 필요 없음)
          settingsChanged = false; // 플래그 리셋
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        // [DISCARD] (원본과 동일 - 로컬 값 복원)
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          if (previousScreen == SCREEN_SETTINGS_CALIB) {
            V_MULTIPLIER = temp_V_MULTIPLIER;
            I_MULTIPLIER = temp_I_MULTIPLIER;
          } else if (previousScreen == SCREEN_SETTINGS_PROTECT) {
            VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD;
          }
          setting_step_index = temp_setting_step_index;
          
          // --- [신규] DISCARD 시, Processor의 값도 원래대로 복구 ---
          // (Controller가 임시 저장한 값으로 덮어쓰기 명령 전송)
          txJsonDoc.clear();
          txJsonDoc["CMD"] = "SET_CALIB";
          txJsonDoc["V_MULT"] = V_MULTIPLIER;
          txJsonDoc["I_MULT"] = I_MULTIPLIER;
          serializeJson(txJsonDoc, Serial); Serial.println();
          
          txJsonDoc.clear();
          txJsonDoc["CMD"] = "SET_PROTECT";
          txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD;
          serializeJson(txJsonDoc, Serial); Serial.println();

          txJsonDoc.clear();
          txJsonDoc["CMD"] = "SET_STEP";
          txJsonDoc["IDX"] = setting_step_index;
          serializeJson(txJsonDoc, Serial); Serial.println();
          // ---
          
          settingsChanged = false; // 플래그 리셋
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        break;

      // ( ... 나머지 케이스 (THEME, POWER, PHASE 등) 원본과 동일 ... )
      case SCREEN_SETTINGS_THEME:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          isDarkMode = false;
          setTheme();
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          isDarkMode = true;
          setTheme();
          screenNeedsRedraw = true;
        }
        break;
      case SCREEN_MAIN_POWER:
      case SCREEN_PHASE_DIFFERENCE:
      case SCREEN_COMBINED_WAVEFORM:
      case SCREEN_THD:
        break;
    }
    
    delay(100); // Debounce
  }
}

// ==============================================================================
// 6. [수정] 설정 값 변경 헬퍼 함수
// ==============================================================================

void adjustCalibValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; // 로컬 변경 플래그
  
  txJsonDoc.clear();
  txJsonDoc["CMD"] = "SET_CALIB";

  float targetValue;

  switch (calib_selection) {
    case 0: // V Multiplier
      targetValue = V_MULTIPLIER + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["V_MULT"] = targetValue;
      // 로컬 값도 즉시 변경 (임시 표시용)
      V_MULTIPLIER = targetValue;
      break;
    case 1: // I Multiplier
      targetValue = I_MULTIPLIER + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["I_MULT"] = targetValue;
      // 로컬 값도 즉시 변경 (임시 표시용)
      I_MULTIPLIER = targetValue;
      break;
    case 2: // Step
      txJsonDoc.clear(); // SET_CALIB 명령이 아님
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index; // 로컬 값 변경
        // Processor에게도 SET_STEP 명령 전송
        txJsonDoc["CMD"] = "SET_STEP";
        txJsonDoc["IDX"] = new_index;
      } else {
        return; // 명령 보내지 않음
      }
      break;
  }
  
  serializeJson(txJsonDoc, Serial);
  Serial.println();
}

void adjustProtectValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; 

  txJsonDoc.clear();
  float targetValue;

  switch (protect_selection) {
    case 0: // Over V Thresh
      targetValue = VOLTAGE_THRESHOLD + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["CMD"] = "SET_PROTECT";
      txJsonDoc["V_THR"] = targetValue;
      // 로컬 값도 즉시 변경 (임시 표시용)
      VOLTAGE_THRESHOLD = targetValue;
      break;
    case 1: // Step
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index; // 로컬 값 변경
        // Processor에게도 SET_STEP 명령 전송
        txJsonDoc["CMD"] = "SET_STEP";
        txJsonDoc["IDX"] = new_index;
      } else {
        return; // 명령 보내지 않음
      }
      break;
  }

  serializeJson(txJsonDoc, Serial);
  Serial.println();
}

// ==============================================================================
// 7. [수정] 릴레이 제어 화면 그리기
// ==============================================================================
void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton();

  // runRelayControl()에서 버튼을 동적으로 그리므로
  // 여기서는 버튼을 그리지 않습니다.
  // 강제 업데이트를 위해 prev 상태를 반전시킵니다.
  prev_r1_state = !relay1_state;
  prev_r2_state = !relay2_state;
}

void runRelayControl() {
  // Processor로부터 받은 릴레이 상태가 이전과 다를 경우 버튼을 다시 그립니다.
  if (relay1_state != prev_r1_state || relay2_state != prev_r2_state || screenNeedsRedraw) {
    String r1_status = relay1_state ? "ON" : "OFF";
    String r2_status = relay2_state ? "ON" : "OFF";
    
    // 버튼 그리기 (이전 상태를 덮어쓰기 위해)
    drawButton(20, 70, 280, 40, "Relay 1: " + r1_status);
    drawButton(20, 130, 280, 40, "Relay 2: " + r2_status);
    
    prev_r1_state = relay1_state;
    prev_r2_state = relay2_state;
  }
}

// ==============================================================================
// 8. [수정] 파형 그리기 루프
// ==============================================================================
void runCombinedWaveformLoop() {
  float volts_to_pixels_scale = (V_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_axis_max);
  float amps_to_pixels_scale = (I_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / I_axis_max);

  float new_frame_V_peak = 0.0;
  float new_frame_I_peak = 0.0;
  
  int new_y_v[PLOT_WIDTH];
  int new_y_i[PLOT_WIDTH];

  // --- [수정] 샘플링 로직 제거 ---
  // [v27] 보정 계수 계산 제거

  for (int i = 0; i < PLOT_WIDTH; i++) {
    // --- [신규] 시리얼에서 "V,I\n" 스트림 읽기 ---
    String line = "";
    while(line.length() == 0) {
      if (Serial.available()) {
         line = Serial.readStringUntil('\n');
         line.trim();
      }
      // 데이터를 기다리는 동안 터치(뒤로가기) 확인
      checkTouchInput();
      if (screenNeedsRedraw || warningActive) return; // 뒤로가기 또는 경고 발생
    }
    
    // --- [신규] 텍스트 파싱 ---
    float V_mains_instant = 0.0;
    float I_mains_instant = 0.0;
    int commaIndex = line.indexOf(',');
    if (commaIndex != -1) {
      V_mains_instant = line.substring(0, commaIndex).toFloat();
      I_mains_instant = line.substring(commaIndex + 1).toFloat();
    }
    // ---

    // --- (이하 로직 원본과 동일) ---
    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    if (abs(I_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I_mains_instant);

    int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
    new_y_v[i] = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
    new_y_i[i] = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
    
    // while(micros() ... ) 대기 로직 제거 (Processor가 타이밍을 맞춤)
  }

  tft.startWrite(); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  for (int i = 1; i < PLOT_WIDTH; i++) {
      int x_curr = PLOT_X_START + i;
      int x_prev = PLOT_X_START + i - 1;
      if(last_frame_y_v[i] != PLOT_Y_CENTER || last_frame_y_v[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_v[i-1], x_curr, last_frame_y_v[i], COLOR_BACKGROUND); 
      }
      if(last_frame_y_i[i] != PLOT_Y_CENTER || last_frame_y_i[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_i[i-1], x_curr, last_frame_y_i[i], COLOR_BACKGROUND); 
      }
      tft.drawLine(x_prev, new_y_v[i-1], x_curr, new_y_v[i], COLOR_BLUE);
      tft.drawLine(x_prev, new_y_i[i-1], x_curr, new_y_i[i], COLOR_ORANGE);
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
      last_frame_y_v[i] = new_y_v[i];
      last_frame_y_i[i] = new_y_i[i];
  }

  float new_V_axis = findAxisStep(new_frame_V_peak, V_AXIS_STEPS, NUM_V_STEPS);
  float new_I_axis = findAxisStep(new_frame_I_peak, I_AXIS_STEPS, NUM_I_STEPS);

  if (new_V_axis != V_axis_max || new_I_axis != I_axis_max) {
    V_axis_max = new_V_axis;
    I_axis_max = new_I_axis;
    updateYAxisLabels();
  }
}

// ==============================================================================
// 9. 헬퍼 함수 (TFT 출력)
// (원본 코드와 동일)
// ==============================================================================

// [v25] 테마 설정 함수
void setTheme() {
  if (isDarkMode) {
    COLOR_BACKGROUND = ILI9341_BLACK;
    COLOR_TEXT_PRIMARY = ILI9341_WHITE;
    COLOR_TEXT_SECONDARY = ILI9341_LIGHTGREY;
    COLOR_BUTTON = ILI9341_NAVY;
    COLOR_BUTTON_TEXT = ILI9341_WHITE;
    COLOR_BUTTON_OUTLINE = ILI9341_DARKGREY;
    COLOR_GRID = ILI9341_DARKGREY;
    COLOR_RED = ILI9341_PINK;       
    COLOR_GREEN = ILI9341_GREENYELLOW; 
    COLOR_BLUE = ILI9341_CYAN;       
    COLOR_ORANGE = ILI9341_YELLOW;     
    COLOR_MAGENTA = ILI9341_MAGENTA;   
    COLOR_DARKGREEN = ILI9341_GREEN;     
  } else {
    COLOR_BACKGROUND = ILI9341_WHITE;
    COLOR_TEXT_PRIMARY = ILI9341_BLACK;
    COLOR_TEXT_SECONDARY = ILI9341_DARKGREY;
    COLOR_BUTTON = ILI9341_BLUE;
    COLOR_BUTTON_TEXT = ILI9341_WHITE;
    COLOR_BUTTON_OUTLINE = ILI9341_BLACK;
    COLOR_GRID = ILI9341_LIGHTGREY;
    COLOR_RED = ILI9341_RED;
    COLOR_GREEN = ILI9341_GREEN;
    COLOR_BLUE = ILI9341_BLUE;
    COLOR_ORANGE = ILI9341_ORANGE;
    COLOR_MAGENTA = ILI9341_MAGENTA;
    COLOR_DARKGREEN = ILI9341_DARKGREEN;
  }
}

// printTFTValue (float)
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) {
    return; 
  }
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit);
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit); 
}

// printTFTValue (int) - 오버로딩
void printTFTValue(int x, int y, int value, int prev_value, uint16_t color, String unit) {
  if (value == prev_value) {
    return; 
  }
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  sprintf(buffer, "%d%s", prev_value, unit.c_str());
  tft.print(buffer);
  
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  sprintf(buffer, "%d%s", value, unit.c_str());
  tft.print(buffer); 
}

// printTFTValue (String)
void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (value.equals(prev_value)) return; 
  
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  tft.print(prev_value);
  tft.setTextColor(color); 
  tft.setCursor(x, y); 
  tft.print(value);
}

// displayNetworkStatus
void displayNetworkStatus() {
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY); 
  tft.setCursor(240, 5); 
  tft.print("NET: OFF"); 
}

// [v20] 뒤로 가기 버튼 그리기
void drawBackButton() {
  tft.fillRoundRect(5, 5, 50, 30, 8, COLOR_TEXT_SECONDARY); 
  tft.setCursor(20, 12);
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setTextSize(2);
  tft.print("<");
}

// [v20] 공통 버튼 그리기 헬퍼
void drawButton(int x, int y, int w, int h, String text) {
  tft.fillRoundRect(x, y, w, h, 8, COLOR_BUTTON); 
  tft.drawRoundRect(x, y, w, h, 8, COLOR_BUTTON_OUTLINE); 
  tft.setTextColor(COLOR_BUTTON_TEXT); 
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(x + (w - w1) / 2, y + (h - h1) / 2);
  tft.print(text);
}


// ==============================================================================
// 10. [수정] 메인 전력 화면 (값 표시)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  drawBackButton(); 
  tft.setTextSize(2);
  
  // (레이아웃 원본과 동일)
  int col1_label_x = 10;
  int col2_label_x = 170;
  int y_row1 = 40;
  int y_row2 = 65;
  int y_row3 = 90;
  int y_row4 = 115;
  int y_row5 = 140; 
  int y_row6 = 165;
  int y_row7 = 190;
  tft.setCursor(col1_label_x, y_row1); tft.setTextColor(COLOR_BLUE); tft.println("V:");
  tft.setCursor(col1_label_x, y_row2); tft.setTextColor(COLOR_ORANGE); tft.println("I:");
  tft.setCursor(col1_label_x, y_row3); tft.setTextColor(COLOR_DARKGREEN); tft.println("P:");
  tft.setCursor(col2_label_x, y_row4); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:");
  tft.setCursor(col1_label_x, y_row4); tft.setTextColor(COLOR_ORANGE); tft.println("Q:"); 
  tft.setCursor(col2_label_x, y_row1); tft.setTextColor(COLOR_RED); tft.println("I-1:");
  tft.setCursor(col2_label_x, y_row2); tft.setTextColor(COLOR_GREEN); tft.println("I-2:");
  tft.setCursor(col2_label_x, y_row3); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("S:");
  tft.setCursor(col1_label_x, y_row5); tft.setTextColor(COLOR_BLUE); tft.println("THD-V:"); 
  tft.setCursor(col1_label_x, y_row6); tft.setTextColor(COLOR_ORANGE); tft.println("THD-I:");
}

// [수정] 계산 대신 표시만 하도록
void displayMainScreenValues() {
  tft.setTextSize(2);
  char buffer[20];
  
  // (레이아웃 원본과 동일)
  int col1_value_x = 60;
  int col2_value_x = 220; 
  int col_w_half = 90; 
  int col_w_half_wide = 100;
  int y_row1 = 40;
  int y_row2 = 65;
  int y_row3 = 90;
  int y_row4 = 115;
  int y_row5 = 140; 
  int y_row6 = 165;
  int y_row7 = 190;
  
  // (printTFTValue는 깜빡임 방지 로직이 있으므로 그대로 사용)
  // (이전 값을 저장할 전역 변수들이 필요함 - 이미 있음)
  
  // --- 이 함수는 원본과 거의 동일하게 작동합니다 ---
  // --- 단, V_rms, I_rms 등의 전역 변수가 ---
  // --- checkSerialInput()에 의해 업데이트된다는 점이 다릅니다 ---
  
  static float prev_V_rms = -1.0;
  static float prev_I_rms = -1.0;
  static float prev_P_real = -1.0;
  static float prev_PF = -1.0;
  static float prev_Q_reactive = -1.0;
  static float prev_I_rms_load1 = -1.0;
  static float prev_I_rms_load2 = -1.0;
  static float prev_S_apparent = -1.0;
  static float prev_thd_v_main = -1.0; // prev_thd_v와 이름 충돌 방지
  static float prev_thd_i_main = -1.0;

  // printTFTValue (float)는 prev_value를 인자로 받습니다.
  // 이 값들을 static으로 선언하여 함수 내에서 상태를 유지합니다.

  // V
  printTFTValue(col1_value_x, y_row1, V_rms, prev_V_rms, 1, COLOR_BLUE, " V");
  prev_V_rms = V_rms;

  // I
  tft.fillRect(col1_value_x, y_row2, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, y_row2);
  if (I_rms < 1.0) { 
    dtostrf(I_rms * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }
  // (참고: mA/A 전환 로직 때문에 printTFTValue를 쓰기 까다로움)
  // (깜빡임을 감수하거나, 복잡한 prev_value 로직 필요. 일단 원본 유지)

  // P
  tft.fillRect(col1_value_x, y_row3, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_DARKGREEN);
  tft.setCursor(col1_value_x, y_row3);
  if (P_real >= 1000.0) { 
    dtostrf(P_real / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kW");
  } else { 
    dtostrf(P_real, 4, 1, buffer);
    tft.print(buffer); tft.print(" W");
  }

  // PF
  printTFTValue(col2_value_x, y_row4, PF, prev_PF, 2, COLOR_MAGENTA, "");
  prev_PF = PF;

  // Q
  tft.fillRect(col1_value_x, y_row4, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, y_row4);
  if (Q_reactive >= 1000.0) { 
    dtostrf(Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVAR");
  } else { 
    dtostrf(Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" VAR");
  }

  // I-1
  tft.fillRect(col2_value_x, y_row1, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_RED);
  tft.setCursor(col2_value_x, y_row1); 
  if (I_rms_load1 < 1.0) { 
    dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load1, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  // I-2
  tft.fillRect(col2_value_x, y_row2, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_GREEN);
  tft.setCursor(col2_value_x, y_row2); 
  if (I_rms_load2 < 1.0) { 
    dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load2, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  // S
  tft.fillRect(col2_value_x, y_row3, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setCursor(col2_value_x, y_row3); 
  if (S_apparent >= 1000.0) { 
    dtostrf(S_apparent / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVA");
  } else { 
    dtostrf(S_apparent, 4, 1, buffer);
    tft.print(buffer); tft.print(" VA");
  }

  // THD-V
  printTFTValue(col2_value_x, y_row5, thd_v_value * 100.0, prev_thd_v_main * 100.0, 1, COLOR_BLUE, " %");
  prev_thd_v_main = thd_v_value;

  // THD-I
  printTFTValue(col2_value_x, y_row6, thd_i_value * 100.0, prev_thd_i_main * 100.0, 1, COLOR_ORANGE, " %");
  prev_thd_i_main = thd_i_value;
}

// ==============================================================================
// 11. [수정] 위상차 화면 (값 표시)
// ==============================================================================
void displayPhaseScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("PHASOR DIAGRAM"); 
  drawBackButton(); 
  
  tft.setTextSize(2);
  tft.setCursor(10, 50); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 75); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("Status:"); 
  
  tft.setTextSize(1);
  tft.setCursor(10, 125); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("--- Phase (deg) ---"); 
  tft.setTextSize(2);
  tft.setCursor(10, 140); tft.setTextColor(COLOR_ORANGE); tft.println("I-M:"); 
  tft.setCursor(10, 165); tft.setTextColor(COLOR_RED); tft.println("I-1:"); 
  tft.setCursor(10, 190); tft.setTextColor(COLOR_GREEN); tft.println("I-2:"); 

  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS, COLOR_GRID); 
  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS / 2, COLOR_GRID); 
  tft.drawFastHLine(PHASOR_CX - PHASOR_RADIUS, PHASOR_CY, PHASOR_RADIUS * 2, COLOR_GRID); 
  tft.drawFastVLine(PHASOR_CX, PHASOR_CY - PHASOR_RADIUS, PHASOR_RADIUS * 2, COLOR_GRID); 
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY); 
  tft.setCursor(PHASOR_CX + PHASOR_RADIUS + 3, PHASOR_CY - 3);
  tft.print("0");                                                           
  int legendY = 200; 
  tft.drawRect(155, legendY, 160, 35, COLOR_TEXT_SECONDARY); 
  tft.drawFastHLine(160, legendY + 17, 10, COLOR_BLUE); 
  tft.setCursor(175, legendY + 4); tft.print("V");
  tft.drawFastHLine(200, legendY + 17, 10, COLOR_ORANGE); 
  tft.setCursor(215, legendY + 4); tft.print("I-M");
  tft.drawFastHLine(240, legendY + 17, 10, COLOR_RED); 
  tft.setCursor(255, legendY + 4); tft.print("I-1");
  tft.drawFastHLine(280, legendY + 17, 10, COLOR_GREEN); 
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

// [수정] 계산 대신 표시만 하도록
void displayPhaseScreenValues() {
  
  // phase_degrees는 로컬에서 계산
  phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
  if (I_rms < 0.05) {
     phase_degrees = 0.0;
  }

  tft.setTextSize(2);
  // prev_phase_degrees는 PF 값으로 대체
  printTFTValue(60, 50, PF, prev_phase_degrees, 2, COLOR_MAGENTA, "");
  printTFTValue(10, 100, lead_lag_status, prev_lead_lag_status, COLOR_TEXT_PRIMARY); 

  printTFTValue(60, 140, phase_main_deg, prev_phase_main_deg, 1, COLOR_ORANGE, "d"); 
  printTFTValue(60, 165, phase_load1_deg, prev_phase_load1_deg, 1, COLOR_RED, "d"); 
  printTFTValue(60, 190, phase_load2_deg, prev_phase_load2_deg, 1, COLOR_GREEN, "d"); 

  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, COLOR_BACKGROUND); 

  // (동적 스케일링 로직 원본과 동일)
  float max_I_rms = max(I_rms, max(I_rms_load1, I_rms_load2));
  float I_scale_denominator = (max_I_rms < 0.05) ? 0.0 : max_I_rms;
  float v_len = PHASOR_RADIUS;
  float im_len = 0.0;
  float i1_len = 0.0;
  float i2_len = 0.0;
  if (I_scale_denominator > 0.0) {
    im_len = (I_rms / I_scale_denominator) * PHASOR_RADIUS;
    i1_len = (I_rms_load1 / I_scale_denominator) * PHASOR_RADIUS;
    i2_len = (I_rms_load2 / I_scale_denominator) * PHASOR_RADIUS;
  }

  int v_x = PHASOR_CX + (int)(v_len * cos(0));
  int v_y = PHASOR_CY - (int)(v_len * sin(0));
  int im_x = PHASOR_CX + (int)(im_len * cos(phase_main_deg * (M_PI / 180.0)));
  int im_y = PHASOR_CY - (int)(im_len * sin(phase_main_deg * (M_PI / 180.0)));
  int i1_x = PHASOR_CX + (int)(i1_len * cos(phase_load1_deg * (M_PI / 180.0)));
  int i1_y = PHASOR_CY - (int)(i1_len * sin(phase_load1_deg * (M_PI / 180.0)));
  int i2_x = PHASOR_CX + (int)(i2_len * cos(phase_load2_deg * (M_PI / 180.0)));
  int i2_y = PHASOR_CY - (int)(i2_len * sin(phase_load2_deg * (M_PI / 180.0)));

  tft.drawLine(PHASOR_CX, PHASOR_CY, v_x, v_y, COLOR_BLUE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, im_x, im_y, COLOR_ORANGE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i1_x, i1_y, COLOR_RED);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i2_x, i2_y, COLOR_GREEN);
  
  prev_phase_degrees = PF; // PF 값 저장
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
// 12. 파형 화면 (그리드 및 라벨)
// (원본 코드와 동일)
// ==============================================================================
void drawWaveformGridAndLabels() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("V/I WAVEFORM (60Hz)");
  drawBackButton(); 
  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), COLOR_GRID); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 
  tft.drawFastVLine(PLOT_X_END, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 

  tft.setTextSize(1);
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(0, PLOT_Y_CENTER - 4); 
  tft.print("0A"); 
  tft.setTextColor(COLOR_BLUE);
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4); 
  tft.print("0V"); 

  tft.setCursor(10, SCREEN_HEIGHT - 12);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY); 
  tft.print("Sampling Period: ");
  tft.print(WAVEFORM_SAMPLE_PERIOD_US);
  tft.print(" us");
}

// ==============================================================================
// 13. 동적 Y축 라벨 업데이트 함수
// (원본 코드와 동일)
// ==============================================================================
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  
  tft.setTextColor(COLOR_ORANGE);
  if (I_axis_max < 1.0) { 
    dtostrf(I_axis_max * 1000, 3, 0, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("mA");
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("mA");
  } else { 
    dtostrf(I_axis_max, 3, 1, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
  }

  tft.setTextColor(COLOR_BLUE);
  dtostrf(V_axis_max, 3, 0, buffer); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("V"); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("V");
}


// ==============================================================================
// 14. 경고 팝업 화면
// (원본 코드와 동일)
// ==============================================================================
void displayWarningScreenStatic() {
  tft.fillScreen(ILI9341_RED);
  tft.drawRect(5, 5, SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, ILI9341_WHITE);
  tft.drawRect(6, 6, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 12, ILI9341_WHITE);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3); 
  tft.setCursor(75, 40); 
  tft.print("WARNING!"); 
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2); 
  
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(warningMessage, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 85); 
  tft.print(warningMessage);
  
  // [수정] 타이머 경고 시에는 값 표시 안 함
  if (warningMessage.startsWith("TIMER")) {
    // 값 표시 안 함
  } else {
    tft.setTextSize(3); 
    char buffer[20];
    if (warningMessage.startsWith("OVER VOLTAGE")) {
      dtostrf(V_rms, 4, 1, buffer);
      strcat(buffer, " V");
    } else {
      dtostrf(I_rms, 4, 2, buffer);
      strcat(buffer, " A");
    }
    tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w1, &h1);
    tft.setCursor((SCREEN_WIDTH - w1) / 2, 125); 
    tft.print(buffer);
  }
  
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2); 
  String resetMsg = "Tap screen to reset";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); 
  tft.print(resetMsg); 
}

// ==============================================================================
// 15. THD 화면
// (원본 코드와 동일)
// ==============================================================================
void displayTHDScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("HARMONIC DISTORTION"); 
  drawBackButton(); 
  
  tft.setTextSize(3);
  tft.setCursor(10, 70); tft.setTextColor(COLOR_BLUE); tft.println("THD-V:");
  tft.setCursor(10, 150); tft.setTextColor(COLOR_ORANGE); tft.println("THD-I:");
  
  prev_thd_v = thd_v_value; 
  prev_thd_i = thd_i_value; 
}

// [수정] 계산 대신 표시만
void displayTHDScreenValues() {
  tft.setTextSize(4);
  printTFTValue(30, 110, thd_v_value * 100.0, prev_thd_v * 100.0, 1, COLOR_BLUE, " %"); 
  printTFTValue(30, 190, thd_i_value * 100.0, prev_thd_i * 100.0, 1, COLOR_ORANGE, " %"); 
  prev_thd_v = thd_v_value; 
  prev_thd_i = thd_i_value; 
}

// ==============================================================================
// 16. [v20] 홈 화면 그리기
// (원본 코드와 동일)
// ==============================================================================
void displayHomeScreenStatic() {
  tft.setCursor(50, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(3);
  tft.print("KW WATT-METER");
  
  drawButton(20, 50, 130, 40, "MAIN POWER");
  drawButton(170, 50, 130, 40, "PHASOR");
  drawButton(20, 110, 130, 40, "WAVEFORM");
  drawButton(170, 110, 130, 40, "THD");
  drawButton(20, 170, 130, 40, "SETTINGS");
  drawButton(170, 170, 130, 40, "RELAY CTRL");
}

// ==============================================================================
// 17. [v20] [수정] 설정 메인 화면 그리기
// ==============================================================================
void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("SETTINGS");
  displayNetworkStatus(); 
  drawBackButton();
  
  // [수정] 3버튼 레이아웃
  drawButton(20, 60, 280, 35, "CALIBRATION");
  drawButton(20, 105, 280, 35, "PROTECTION");
  drawButton(20, 150, 280, 35, "ADVANCED");
  // 제거: drawButton(20, 160, 130, 40, "THEME");
  // 제거: drawButton(170, 160, 130, 40, "RESET");
}

// ==============================================================================
// 18. [v20] 설정 - 보정 화면 그리기
// (원본 코드와 동일)
// ==============================================================================
void displaySettingsCalibStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("CALIBRATION SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(30, 50); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("V Mult:"); 
  tft.setCursor(30, 85); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("I Mult:"); 
  tft.setCursor(30, 120); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Step:"); 

  drawButton(20, 180, 60, 40, "UP");
  drawButton(90, 180, 60, 40, "DOWN");
  drawButton(180, 180, 60, 40, "-");
  drawButton(250, 180, 60, 40, "+");
  
  prev_calib_selection = -1; 
  prev_V_MULTIPLIER = -1.0;
  prev_I_MULTIPLIER = -1.0;
  prev_setting_step_index = -1;
}

// [수정] runSettingsCalib() -> displaySettingsCalibValues()
void displaySettingsCalibValues() {
  int y_positions[] = {50, 85, 120};
  
  if (prev_calib_selection != calib_selection) {
    if (prev_calib_selection != -1) {
      tft.fillRect(10, y_positions[prev_calib_selection], 15, 18, COLOR_BACKGROUND); 
    }
    tft.setTextColor(COLOR_RED); 
    tft.setTextSize(2);
    tft.setCursor(10, y_positions[calib_selection]);
    tft.print(">");
    prev_calib_selection = calib_selection;
  }

  printTFTValue(190, 50, V_MULTIPLIER, prev_V_MULTIPLIER, 2, COLOR_BLUE, " x"); 
  printTFTValue(190, 85, I_MULTIPLIER, prev_I_MULTIPLIER, 2, COLOR_ORANGE, " x"); 
  printTFTValue(190, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_V_MULTIPLIER = V_MULTIPLIER;
  prev_I_MULTIPLIER = I_MULTIPLIER;
  prev_setting_step_index = setting_step_index;
}

// ==============================================================================
// 19. [v20] 설정 - 보호 화면 그리기
// (원본 코드와 동일)
// ==============================================================================
void displaySettingsProtectStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("PROTECTION SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(30, 70); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Over V:"); 
  tft.setCursor(30, 110); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Step:"); 

  drawButton(20, 180, 60, 40, "UP");
  drawButton(90, 180, 60, 40, "DOWN");
  drawButton(180, 180, 60, 40, "-");
  drawButton(250, 180, 60, 40, "+");
  
  prev_protect_selection = -1;
  prev_VOLTAGE_THRESHOLD = -1.0;
  prev_setting_step_index = -1;
}

// [수정] runSettingsProtect() -> displaySettingsProtectValues()
void displaySettingsProtectValues() {
  int y_positions[] = {70, 110};
  
  if (prev_protect_selection != protect_selection) {
    if (prev_protect_selection != -1) {
      tft.fillRect(10, y_positions[prev_protect_selection], 15, 18, COLOR_BACKGROUND); 
    }
    tft.setTextColor(COLOR_RED); 
    tft.setTextSize(2);
    tft.setCursor(10, y_positions[protect_selection]);
    tft.print(">");
    prev_protect_selection = protect_selection;
  }
  
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  prev_setting_step_index = setting_step_index;
}


// ==============================================================================
// 20. [v24] 파형 Y축 범위 계산 헬퍼
// (원본 코드와 동일)
// ==============================================================================
float findAxisStep(float peak, const float* steps, int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    if (peak <= steps[i]) {
      return steps[i]; 
    }
  }
  return steps[num_steps - 1]; 
}

// ==============================================================================
// 21. [v25] 신규 화면 그리기
// (원본 코드와 동일)
// ==============================================================================

// --- 테마 설정 ---
void displaySettingsThemeStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("THEME SETTINGS");
  drawBackButton();

  drawButton(20, 70, 280, 40, "LIGHT MODE");
  drawButton(20, 130, 280, 40, "DARK MODE");

  tft.setTextColor(COLOR_RED);
  tft.setTextSize(2);
  if (isDarkMode) {
    tft.setCursor(5, 140);
  } else {
    tft.setCursor(5, 80);
  }
  tft.print(">");
}

void runSettingsTheme() {
  // 정적 화면
}

// --- 설정 초기화 ---
void displaySettingsResetStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RESET SETTINGS");
  drawBackButton();

  tft.setCursor(20, 60);
  tft.print("Reset all values to");
  tft.setCursor(20, 80);
  tft.print("factory defaults?");

  drawButton(20, 100, 130, 40, "RESET");
  drawButton(170, 100, 130, 40, "CANCEL");
}

// --- 설정 저장 확인 ---
void displayConfirmSaveStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("SAVE CHANGES?");
  drawBackButton();

  tft.setCursor(20, 70);
  tft.print("Save the new settings?");

  drawButton(20, 100, 130, 40, "SAVE");
  drawButton(170, 100, 130, 40, "DISCARD");
}

// --- [신규] 고급 설정 화면 ---
void displaySettingsAdvancedStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("ADVANCED SETTINGS");
  drawBackButton();

  // 요청한 대로 CALIBRATION 버튼과 동일한 스타일 (280x35)
  drawButton(20, 60, 280, 35, "THEME");
  drawButton(20, 105, 280, 35, "TIMER");
  drawButton(20, 150, 280, 35, "RESET");
}

// --- [신규] 타이머 설정 화면 ---
void displaySettingsTimerStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RELAY TIMER SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(20, 50); tft.print("Set Minutes (0=Off):");
  tft.setCursor(20, 160); tft.print("Status:");
  tft.setCursor(20, 190); tft.print("Time Left:");

  // 버튼 레이아웃
  drawButton(20, 100, 60, 40, "-10");
  drawButton(90, 100, 60, 40, "+10");
  drawButton(170, 100, 60, 40, "SET");
  drawButton(240, 100, 60, 40, "OFF");
}

// --- [신규] 타이머 값 동적 표시 ---
void displaySettingsTimerValues() {
  // 1. 설정 중인 값 (temp_timer_setting_minutes)
  printTFTValue(240, 50, temp_timer_setting_minutes, prev_temp_timer_setting_minutes, COLOR_ORANGE, " min");
  prev_temp_timer_setting_minutes = temp_timer_setting_minutes;

  // 2. 현재 타이머 활성 상태
  String status_text = is_timer_active ? "ACTIVE" : "INACTIVE";
  String prev_status_text = prev_is_timer_active ? "ACTIVE" : "INACTIVE";
  uint16_t status_color = is_timer_active ? COLOR_GREEN : COLOR_RED;
  
  printTFTValue(150, 160, status_text, prev_status_text, status_color);
  prev_is_timer_active = is_timer_active;

  // 3. 남은 시간 (초 -> 분:초)
  if (timer_seconds_left != prev_timer_seconds_left) {
    uint16_t minutes = timer_seconds_left / 60;
    uint8_t seconds = timer_seconds_left % 60;
    
    char buffer[10];
    sprintf(buffer, "%02d:%02d", minutes, seconds);
    
    char prev_buffer[10];
    sprintf(prev_buffer, "%02d:%02d", (uint16_t)(prev_timer_seconds_left / 60), (uint8_t)(prev_timer_seconds_left % 60));

    printTFTValue(150, 190, String(buffer), String(prev_buffer), COLOR_TEXT_PRIMARY);
    prev_timer_seconds_left = timer_seconds_left;
  }
}

// --- 초기화 헬퍼 ---
// (제거됨 - Processor가 담당)
// void restoreDefaultSettings() { ... }