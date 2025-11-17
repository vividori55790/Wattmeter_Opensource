/*
 * ==============================================================================
 * [Controller.ino]
 * - 디스플레이, 터치스크린, WiFi, UI 로직 담당
 * - D0/D1 (Serial)을 통해 Processor와 통신합니다.
 * - [중요] 파형(Waveform) 화면을 위해 A2(I), A3(V) 핀을 Processor와 병렬 연결
 * ==============================================================================
 */

// --- 라이브러리 포함 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <math.h> // 파형 화면 계산용

// ==============================================================================
// [NEW] 공통 데이터 구조체 (Processor.ino에도 동일하게 정의되어야 함)
// ==============================================================================

// 1. Processor -> Controller (데이터 전송용)
struct ProcessorData {
  // 메인 전력 값
  float V_rms;
  float I_rms;
  float I_rms_load1;
  float I_rms_load2;
  float P_real;
  float Q_reactive;
  float S_apparent;
  float PF;

  // THD 값
  float thd_v_value;
  float thd_i_value;
  
  // 위상 값
  float phase_main_deg;
  float phase_load1_deg;
  float phase_load2_deg;
  char lead_lag_status[8]; // "Lead", "Lag", "---"

  // 상태 값
  bool relay1_state;
  bool relay2_state;
  bool warningActive;
  char warningMessage[20]; // "OVER VOLTAGE!" 등
};

// 2. Controller -> Processor (설정 전송용)
struct ControllerSettings {
  float VOLTAGE_THRESHOLD;
  float V_MULTIPLIER;
  float I_MULTIPLIER;
};

// 3. 통신 커맨드 (단일 바이트)
#define CMD_HEADER_DATA 'D'     // 'D' + ProcessorData struct
#define CMD_HEADER_SETTINGS 'S' // 'S' + ControllerSettings struct
#define CMD_TOGGLE_RELAY_1 '1'
#define CMD_TOGGLE_RELAY_2 '2'
#define CMD_RESET_WARNING 'W'
#define CMD_RESET_SETTINGS 'R'

// ==============================================================================

// --- [v20] 화면 상태 정의 (홈 화면 기반) ---
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
  SCREEN_CONFIRM_SAVE, 
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; // [v20] 부팅 시 홈 화면
volatile ScreenState previousScreen = SCREEN_HOME; // [v25] 저장/취소 시 돌아갈 화면
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 ---
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8
#define TOUCH_CS 7
#define WIFI_TX_PIN 2
#define WIFI_RX_PIN 3

// [NEW] 파형 화면(Waveform) 전용 ADC 핀
// 이 핀들은 Processor의 핀과 병렬로 연결되어야 합니다.
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2

// --- [v25] 다크 모드 및 테마 ---
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

// --- [v20] 실시간 설정 변수 (Controller가 소유 및 관리) ---
// [v26] "BASE" 값 (파형 화면 계산용)
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

// [v27] "DEFAULT" 값 (초기화용)
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3

// [NEW] Controller가 설정값의 마스터 복사본을 가짐
float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// --- [v25] 설정 임시 저장 변수 (취소 시 복원용) ---
float temp_VOLTAGE_THRESHOLD;
float temp_V_MULTIPLIER;
float temp_I_MULTIPLIER;
int temp_setting_step_index;

// --- [v20] 설정 화면용 이전 값 (깜빡임 제거) ---
float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;

bool settingsChanged = false; 

// [NEW] 경고 상태는 g_ProcData로부터 복사됨
volatile bool warningActive = false;
String warningMessage = "";

// --- ADC 상수 (파형 화면용) ---
const float V_ADC_MIDPOINT = 8192.0; 
const float I_ADC_MIDPOINT = 8192.0; 

// --- 터치스크린 상수 ---
#define TS_RAW_X1 370
#define TS_RAW_Y1 450
#define TS_RAW_X2 3760
#define TS_RAW_Y2 3670

// --- 객체 생성 ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS); 
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);

// --- UI 상수 ---
#define SCREEN_WIDTH 320 
#define SCREEN_HEIGHT 240 

// --- 파형 화면 상수 ---
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

const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};

float V_axis_max = V_AXIS_STEPS[4]; // 350V
float I_axis_max = I_AXIS_STEPS[2]; // 1.0A

// --- [v19] 페이저 다이어그램 상수 ---
#define PHASOR_CX 235
#define PHASOR_CY 115
#define PHASOR_RADIUS 75

// --- [NEW] 전역 데이터 (Processor로부터 수신) ---
ProcessorData g_ProcData = {0}; // Processor 데이터의 로컬 복사본
bool newDataReady = false; // 새 데이터 수신 플래그

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
float32_t prev_thd_v = 0.0;
float32_t prev_thd_i = 0.0;

// --- [v23] 설정 화면 UI용 전역 변수 ---
float setting_steps[] = {0.0001, 0.01, 0.1, 1.0, 10.0};
const int NUM_SETTING_STEPS = 5;
int setting_step_index = DEFAULT_SETTING_STEP_INDEX;
int prev_setting_step_index = -1;

int calib_selection = 0;
int prev_calib_selection = -1;
const int NUM_CALIB_SETTINGS = 3; // V_Mult, I_Mult, STEP

int protect_selection = 0;
int prev_protect_selection = -1;
const int NUM_PROTECT_SETTINGS = 2; // V_THRESH, STEP


// --- 함수 프로토타입 ---
void updateYAxisLabels();
void waitForVoltageZeroCross();
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
void restoreDefaultSettings(); // [NEW] Controller의 로컬 설정 초기화
void sendSettingsToProcessor(); // [NEW] Processor로 설정 전송
void sendCommandToProcessor(byte cmd); // [NEW] Processor로 단일 명령 전송
void checkSerialData(); // [NEW] Processor로부터 데이터 수신

void displayMainScreenStatic();
void displayPhaseScreenStatic();
void drawWaveformGridAndLabels();
void displayTHDScreenStatic();
void displayWarningScreenStatic();

// [NEW] 계산 함수가 아닌, 값 표시 함수로 이름 변경
void displayMainScreenValues(); 
void displayPhaseScreenValues(); 
void runCombinedWaveformLoop(); // 이 함수는 자체적으로 ADC를 사용
void displayTHDScreenValues(); 


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200); // D0/D1을 통한 Processor와 통신
  
  setTheme(); // [v25] 테마 설정
  
  // [NEW] 파형 화면(Waveform)을 위한 ADC 핀 설정
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 

  tft.begin();
  tft.setRotation(3); 
  
  ts.begin(SPI);
  ts.setRotation(3); 
  
  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. Processor로부터 데이터 수신 시도
  checkSerialData(); 
  
  // 2. 터치 입력 항시 확인
  checkTouchInput(); 
  
  // 3. 화면 다시 그리기
  if (screenNeedsRedraw) { 
    tft.fillScreen(COLOR_BACKGROUND); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic(); 
    } else { 
      // 정적 UI 그리기
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
        case SCREEN_CONFIRM_SAVE: 
          displayConfirmSaveStatic();
          break;
        case SCREEN_WARNING: 
          break;
      }
    }
    screenNeedsRedraw = false; 
  }
  
  // 4. [NEW] 동적 UI 업데이트 (데이터 기반)
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      if (newDataReady) displayMainScreenValues(); 
      break;
    case SCREEN_PHASE_DIFFERENCE:
      if (newDataReady) displayPhaseScreenValues();
      break;
    case SCREEN_THD:
      if (newDataReady) displayTHDScreenValues(); 
      break;
    case SCREEN_RELAY_CONTROL: 
      // [NEW] 릴레이 상태가 바뀌면 화면을 다시 그려야 함
      if (newDataReady) {
          // 상태가 바뀌었는지 확인 (단순화를 위해 매번 다시 그리도록 함)
          // (최적화: g_ProcData.relay_state와 prev_relay_state 비교)
          displayRelayControlStatic(); // 버튼 텍스트(ON/OFF) 업데이트
      }
      break;
      
    // [!!!] COMBINED_WAVEFORM은 g_ProcData와 무관하게 자체 ADC로 작동
    case SCREEN_COMBINED_WAVEFORM:
      waitForVoltageZeroCross();
      if (screenNeedsRedraw || warningActive) break; 
      runCombinedWaveformLoop();
      break;
      
    // 설정 화면은 값 변경 시 즉시 업데이트됨
    case SCREEN_SETTINGS_CALIB:
      runSettingsCalib(); 
      break;
    case SCREEN_SETTINGS_PROTECT:
      runSettingsProtect(); 
      break;
    case SCREEN_SETTINGS_THEME: 
      runSettingsTheme();
      break;

    // 정적 화면들
    case SCREEN_HOME: 
    case SCREEN_SETTINGS:
    case SCREEN_SETTINGS_RESET: 
    case SCREEN_CONFIRM_SAVE: 
    case SCREEN_WARNING: 
      break;
  }
  
  newDataReady = false; // 데이터 처리 완료
}

// ==============================================================================
// [NEW] 3. Processor 데이터 수신 함수
// ==============================================================================
void checkSerialData() {
  if (Serial.available() > 0) {
    byte cmd = Serial.read();
    
    if (cmd == CMD_HEADER_DATA) {
      if (Serial.available() >= sizeof(ProcessorData)) {
        // 데이터 구조체 읽기
        Serial.readBytes((uint8_t*)&g_ProcData, sizeof(ProcessorData));
        newDataReady = true;
        
        // [NEW] 경고 상태 동기화
        if (g_ProcData.warningActive && !warningActive) {
          // Processor가 새로운 경고를 보냄
          warningActive = true;
          warningMessage = String(g_ProcData.warningMessage);
          screenNeedsRedraw = true; // 경고 화면 강제 전환
        } else if (!g_ProcData.warningActive && warningActive) {
          // Processor의 경고가 해제됨 (예: 수동 리셋)
          warningActive = false;
          screenNeedsRedraw = true; // 홈 화면으로
        }
      }
    }
  }
}

// ==============================================================================
// [NEW] 4. Processor 명령 전송 함수
// ==============================================================================
void sendCommandToProcessor(byte cmd) {
  Serial.write(cmd);
}

void sendSettingsToProcessor() {
  ControllerSettings newSettings;
  newSettings.VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  newSettings.V_MULTIPLIER = V_MULTIPLIER;
  newSettings.I_MULTIPLIER = I_MULTIPLIER;
  
  Serial.write(CMD_HEADER_SETTINGS);
  Serial.write((uint8_t*)&newSettings, sizeof(newSettings));
}

// ==============================================================================
// 3. 터치 입력 확인 함수 (버튼 기반)
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
    
    // --- 공통 경고 화면 터치 ---
    if (currentScreen == SCREEN_WARNING) { 
      warningActive = false;
      sendCommandToProcessor(CMD_RESET_WARNING); // [NEW] Processor에 경고 리셋 명령
      currentScreen = SCREEN_HOME;
      screenNeedsRedraw = true;
      delay(100); // Debounce
      return; 
    }

    // --- 공통 뒤로 가기 버튼 (<) ---
    if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
      if (p.x >= 5 && p.x <= 55 && p.y >= 5 && p.y <= 35) {
        
        if (currentScreen == SCREEN_SETTINGS_CALIB || currentScreen == SCREEN_SETTINGS_PROTECT) {
          if (settingsChanged) { 
            previousScreen = currentScreen;
            currentScreen = SCREEN_CONFIRM_SAVE;
          } else { 
            currentScreen = SCREEN_SETTINGS;
          }
        } 
        else if (currentScreen == SCREEN_SETTINGS_THEME || currentScreen == SCREEN_SETTINGS_RESET || currentScreen == SCREEN_CONFIRM_SAVE) {
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

    // --- 화면별 버튼 로직 ---
    switch (currentScreen) {
      case SCREEN_HOME:
        if (p.x >= 20 && p.x <= 150 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_MAIN_POWER; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_PHASE_DIFFERENCE; screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_COMBINED_WAVEFORM; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_THD; screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_RELAY_CONTROL; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS:
        if (p.x >= 20 && p.x <= 300 && p.y >= 60 && p.y <= 95) {
          temp_V_MULTIPLIER = V_MULTIPLIER;
          temp_I_MULTIPLIER = I_MULTIPLIER;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_CALIB; screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 105 && p.y <= 140) {
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_PROTECT; screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 160 && p.y <= 200) {
          currentScreen = SCREEN_SETTINGS_THEME; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 160 && p.y <= 200) {
          currentScreen = SCREEN_SETTINGS_RESET; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_CALIB:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) {
          calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) {
          calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS;
        }
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) {
          adjustCalibValue(false); // Decrease
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustCalibValue(true); // Increase
        }
        break;
        
      case SCREEN_SETTINGS_PROTECT:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) {
          protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) {
          protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS;
        }
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) {
          adjustProtectValue(false); // Decrease
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustProtectValue(true); // Increase
        }
        break;
        
      case SCREEN_RELAY_CONTROL:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          sendCommandToProcessor(CMD_TOGGLE_RELAY_1); // [NEW]
          screenNeedsRedraw = true; // Processor 응답 후 상태 갱신
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          sendCommandToProcessor(CMD_TOGGLE_RELAY_2); // [NEW]
          screenNeedsRedraw = true; 
        }
        break;

      case SCREEN_SETTINGS_THEME:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          isDarkMode = false; setTheme(); screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          isDarkMode = true; setTheme(); screenNeedsRedraw = true;
        }
        break;
        
      case SCREEN_SETTINGS_RESET:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          restoreDefaultSettings(); // [NEW] Controller의 로컬 값 초기화
          sendSettingsToProcessor(); // [NEW] 초기화된 값을 Processor로 전송
          // sendCommandToProcessor(CMD_RESET_SETTINGS); // [ALT] Processor가 자체 초기화
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_CONFIRM_SAVE:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          // [NEW] 변경된 값이 이미 Controller 전역 변수에 적용됨
          sendSettingsToProcessor(); // Processor에 저장된 값 전송
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          // [NEW] 임시 변수에서 Controller 전역 변수로 복원 (Processor에 보낼 필요 없음)
          if (previousScreen == SCREEN_SETTINGS_CALIB) {
            V_MULTIPLIER = temp_V_MULTIPLIER;
            I_MULTIPLIER = temp_I_MULTIPLIER;
          } else if (previousScreen == SCREEN_SETTINGS_PROTECT) {
            VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD;
          }
          setting_step_index = temp_setting_step_index; 
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
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
// 4. 헬퍼 함수 (TFT 출력)
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
    return; // 변경 없음
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
// 5. 메인 전력 화면 그리기 (정적)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  drawBackButton(); 
  
  tft.setTextSize(2);
  
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

// ==============================================================================
// 6. 메인 전력 화면 "값" 업데이트 (g_ProcData 기반)
// ==============================================================================
void displayMainScreenValues() {
  tft.setTextSize(2);
  char buffer[20];
  
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
  
  // [NEW] 모든 값을 g_ProcData.xxx에서 읽어옴
  tft.fillRect(col1_value_x, y_row1, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE); 
  tft.setCursor(col1_value_x, y_row1);
  dtostrf(g_ProcData.V_rms, 4, 1, buffer);
  tft.print(buffer); tft.print(" V");

  tft.fillRect(col1_value_x, y_row2, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, y_row2);
  if (g_ProcData.I_rms < 1.0) { 
    dtostrf(g_ProcData.I_rms * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(g_ProcData.I_rms, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(col1_value_x, y_row3, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_DARKGREEN);
  tft.setCursor(col1_value_x, y_row3);
  if (g_ProcData.P_real >= 1000.0) { 
    dtostrf(g_ProcData.P_real / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kW");
  } else { 
    dtostrf(g_ProcData.P_real, 4, 1, buffer);
    tft.print(buffer); tft.print(" W");
  }

  tft.fillRect(col2_value_x, y_row4, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_MAGENTA);
  tft.setCursor(col2_value_x, y_row4);
  dtostrf(g_ProcData.PF, 4, 2, buffer);
  tft.print(buffer);

  tft.fillRect(col1_value_x, y_row4, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, y_row4);
  if (g_ProcData.Q_reactive >= 1000.0) { 
    dtostrf(g_ProcData.Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVAR");
  } else { 
    dtostrf(g_ProcData.Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" VAR");
  }

  tft.fillRect(col2_value_x, y_row1, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_RED);
  tft.setCursor(col2_value_x, y_row1); 
  if (g_ProcData.I_rms_load1 < 1.0) { 
    dtostrf(g_ProcData.I_rms_load1 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(g_ProcData.I_rms_load1, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(col2_value_x, y_row2, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_GREEN);
  tft.setCursor(col2_value_x, y_row2); 
  if (g_ProcData.I_rms_load2 < 1.0) { 
    dtostrf(g_ProcData.I_rms_load2 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(g_ProcData.I_rms_load2, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }

  tft.fillRect(col2_value_x, y_row3, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setCursor(col2_value_x, y_row3); 
  if (g_ProcData.S_apparent >= 1000.0) { 
    dtostrf(g_ProcData.S_apparent / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVA");
  } else { 
    dtostrf(g_ProcData.S_apparent, 4, 1, buffer);
    tft.print(buffer); tft.print(" VA");
  }

  tft.fillRect(col2_value_x, y_row5, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE);
  tft.setCursor(col2_value_x, y_row5);
  dtostrf(g_ProcData.thd_v_value * 100.0, 4, 1, buffer);
  tft.print(buffer); tft.print(" %");

  tft.fillRect(col2_value_x, y_row6, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col2_value_x, y_row6);
  dtostrf(g_ProcData.thd_i_value * 100.0, 4, 1, buffer);
  tft.print(buffer); tft.print(" %");
}

// ==============================================================================
// 7. 위상차 화면 그리기 (정적)
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
  
  // [NEW] 이전 값 초기화
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
// 8. 위상차 화면 "값" 업데이트 함수 (g_ProcData 기반)
// ==============================================================================
void displayPhaseScreenValues() {
  // [NEW] g_ProcData.xxx에서 값 읽기
  String lead_lag_status = String(g_ProcData.lead_lag_status);
  float PF = g_ProcData.PF;
  float phase_main_deg = g_ProcData.phase_main_deg;
  float phase_load1_deg = g_ProcData.phase_load1_deg;
  float phase_load2_deg = g_ProcData.phase_load2_deg;
  
  tft.setTextSize(2);
  printTFTValue(60, 50, PF, prev_phase_degrees, 2, COLOR_MAGENTA, "");
  printTFTValue(10, 100, lead_lag_status, prev_lead_lag_status, COLOR_TEXT_PRIMARY); 

  printTFTValue(60, 140, phase_main_deg, prev_phase_main_deg, 1, COLOR_ORANGE, "d"); 
  printTFTValue(60, 165, phase_load1_deg, prev_phase_load1_deg, 1, COLOR_RED, "d"); 
  printTFTValue(60, 190, phase_load2_deg, prev_phase_load2_deg, 1, COLOR_GREEN, "d"); 

  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, COLOR_BACKGROUND); 

  // --- Dynamic Current Scaling ---
  float max_I_rms = max(g_ProcData.I_rms, max(g_ProcData.I_rms_load1, g_ProcData.I_rms_load2));
  float I_scale_denominator = (max_I_rms < 0.05) ? 0.0 : max_I_rms;
  
  float v_len = PHASOR_RADIUS;
  float im_len = 0.0;
  float i1_len = 0.0;
  float i2_len = 0.0;

  if (I_scale_denominator > 0.0) {
    im_len = (g_ProcData.I_rms / I_scale_denominator) * PHASOR_RADIUS;
    i1_len = (g_ProcData.I_rms_load1 / I_scale_denominator) * PHASOR_RADIUS;
    i2_len = (g_ProcData.I_rms_load2 / I_scale_denominator) * PHASOR_RADIUS;
  }
  // --- End of Dynamic Scaling ---

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
// 9. 60Hz 0점 통과(Zero-Crossing) 대기 함수 (파형 화면용)
// ==============================================================================
void waitForVoltageZeroCross() {
  long startTime = micros();
  long timeout = 20000; 
  int V_ac_bits_prev = 0;
  int V_raw = 0;
  int V_ac_bits = 0; 
  
  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT; 
    if (V_ac_bits < -50) {
       V_ac_bits_prev = V_ac_bits;
       break;
    }
    if (micros() - startTime > timeout) return; 
  }

  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT; 
    if (V_ac_bits_prev < 0 && V_ac_bits >= 0) return;
    V_ac_bits_prev = V_ac_bits;
    if (micros() - startTime > timeout * 2) return; 
  }
}

// ==============================================================================
// 10. 공통 파형 화면 (정적 그리드)
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
// 11. 동적 Y축 라벨 업데이트 함수
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
// 12. 경고 팝업 화면
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
  // [NEW] warningMessage는 g_ProcData로부터 동기화됨
  tft.getTextBounds(warningMessage, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 85); 
  tft.print(warningMessage);
  
  tft.setTextSize(3); 
  char buffer[20];
  if (warningMessage.startsWith("OVER VOLTAGE")) {
    dtostrf(g_ProcData.V_rms, 4, 1, buffer);
    strcat(buffer, " V");
  } else {
    // [NEW] g_ProcData 값 사용
    dtostrf(g_ProcData.I_rms, 4, 2, buffer);
    strcat(buffer, " A");
  }
  tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 125); 
  tft.print(buffer);
  
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2); 
  String resetMsg = "Tap screen to reset";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); 
  tft.print(resetMsg); 
}

// ==============================================================================
// 16. 파형 그리기 루프 (Controller 자체 ADC 사용)
// ==============================================================================
void runCombinedWaveformLoop() {
  float volts_to_pixels_scale = (V_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_axis_max);
  float amps_to_pixels_scale = (I_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / I_axis_max);

  float new_frame_V_peak = 0.0;
  float new_frame_I_peak = 0.0;
  
  int new_y_v[PLOT_WIDTH];
  int new_y_i[PLOT_WIDTH];

  unsigned long startTime = micros();

  // [NEW] Controller가 소유한 설정값을 기반으로 계산
  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  for (int i = 0; i < PLOT_WIDTH; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    
    float V_mains_instant = V_ac_bits * effective_V_Calib + effective_V_Offset;
    float I_mains_instant = I_ac_bits * effective_I_Calib - effective_I_Offset;

    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    if (abs(I_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I_mains_instant);

    int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
    new_y_v[i] = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
    new_y_i[i] = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
    
    while(micros() - startTime < (i + 1) * WAVEFORM_SAMPLE_PERIOD_US);
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
// 21. THD 화면 정적 UI 그리기
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
  
  prev_thd_v = g_ProcData.thd_v_value; 
  prev_thd_i = g_ProcData.thd_i_value; 
}

// ==============================================================================
// 22. THD 화면 동적 값 업데이트 (g_ProcData 기반)
// ==============================================================================
void displayTHDScreenValues() {
  tft.setTextSize(4);
  // [NEW] g_ProcData 값 사용
  printTFTValue(30, 110, g_ProcData.thd_v_value * 100.0, prev_thd_v * 100.0, 1, COLOR_BLUE, " %"); 
  printTFTValue(30, 190, g_ProcData.thd_i_value * 100.0, prev_thd_i * 100.0, 1, COLOR_ORANGE, " %"); 
  prev_thd_v = g_ProcData.thd_v_value; 
  prev_thd_i = g_ProcData.thd_i_value; 
}

// ==============================================================================
// 25. 홈 화면 그리기
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
// 26. 설정 메인 화면 그리기
// ==============================================================================
void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("SETTINGS");
  displayNetworkStatus(); 
  drawBackButton();
  
  drawButton(20, 60, 280, 35, "CALIBRATION");
  drawButton(20, 105, 280, 35, "PROTECTION");
  drawButton(20, 160, 130, 40, "THEME");
  drawButton(170, 160, 130, 40, "RESET");
}

// ==============================================================================
// 27. 설정 - 보정 화면 그리기
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

void runSettingsCalib() {
  displaySettingsCalibValues();
}

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

  // [NEW] Controller가 소유한 전역 변수 값 표시
  printTFTValue(190, 50, V_MULTIPLIER, prev_V_MULTIPLIER, 2, COLOR_BLUE, " x"); 
  printTFTValue(190, 85, I_MULTIPLIER, prev_I_MULTIPLIER, 2, COLOR_ORANGE, " x"); 
  printTFTValue(190, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_V_MULTIPLIER = V_MULTIPLIER;
  prev_I_MULTIPLIER = I_MULTIPLIER;
  prev_setting_step_index = setting_step_index;
}

// ==============================================================================
// 28. 설정 - 보호 화면 그리기
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

void runSettingsProtect() {
  displaySettingsProtectValues();
}

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
  
  // [NEW] Controller가 소유한 전역 변수 값 표시
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  prev_setting_step_index = setting_step_index;
}

// ==============================================================================
// 29. 릴레이 제어 화면 그리기
// ==============================================================================
void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton();

  // [NEW] g_ProcData의 릴레이 상태를 읽어 버튼 텍스트 설정
  String r1_status = g_ProcData.relay1_state ? "ON" : "OFF";
  String r2_status = g_ProcData.relay2_state ? "ON" : "OFF";
  
  drawButton(20, 70, 280, 40, "Relay 1: " + r1_status);
  drawButton(20, 130, 280, 40, "Relay 2: " + r2_status);
}

void runRelayControl() {
  // [NEW] 이 함수는 루프에서 displayRelayControlStatic()으로 대체됨
  // (실시간 상태 반영을 위해)
}

// ==============================================================================
// 30. 설정 값 변경 헬퍼 함수
// ==============================================================================

void adjustCalibValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; 
  
  switch (calib_selection) {
    case 0: // V Multiplier
      V_MULTIPLIER += (increase ? step_to_apply : -step_to_apply);
      if (V_MULTIPLIER < 0) V_MULTIPLIER = 0;
      break;
    case 1: // I Multiplier
      I_MULTIPLIER += (increase ? step_to_apply : -step_to_apply);
      if (I_MULTIPLIER < 0) I_MULTIPLIER = 0;
      break;
    case 2: // Step
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index;
      }
      break;
  }
}

void adjustProtectValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; 

  switch (protect_selection) {
    case 0: // Over V Thresh
      VOLTAGE_THRESHOLD += (increase ? step_to_apply : -step_to_apply);
      if (VOLTAGE_THRESHOLD < 0) VOLTAGE_THRESHOLD = 0;
      break;
    case 1: // Step
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index;
      }
      break;
  }
}

// ==============================================================================
// 31. 파형 Y축 범위 계산 헬퍼
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
// 32. 신규 화면 그리기
// ==============================================================================

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

// [NEW] Controller의 로컬 설정값을 초기화하는 함수
void restoreDefaultSettings() {
  VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
  V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
  I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
  setting_step_index = DEFAULT_SETTING_STEP_INDEX;
}