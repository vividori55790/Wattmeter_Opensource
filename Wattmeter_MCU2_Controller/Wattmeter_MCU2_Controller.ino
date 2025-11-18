/*
 * ==============================================================================
 * 파일명: 1. Wattmeter_MCU2_Controller.ino (메인 컨트롤러)
 * 버전: v76 (Fix Missing Variables & Compilation Errors)
 * 최종 수정: 2025-11-21
 *
 * [변경 사항 - v76]
 * - 컴파일 에러 해결을 위해 누락된 전역 변수 복구:
 * : setting_steps[], setting_step_index, calib/protect selection 변수들
 * : prev_thd_v, prev_thd_i 등
 * - 기존 v75의 기능(ThingSpeak, Network, Harmonics)은 모두 유지.
 * ==============================================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <arm_math.h> 
#include <ArduinoJson.h>
#include <SoftwareSerial.h> // ESP-01 통신용

// --- 화면 상태 정의 ---
enum ScreenState {
  SCREEN_HOME,              
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_HARMONICS,          
  SCREEN_SETTINGS,
  SCREEN_SETTINGS_NETWORK,  // 네트워크 설정 화면
  SCREEN_SETTINGS_CALIB,    
  SCREEN_SETTINGS_PROTECT,  
  SCREEN_RELAY_CONTROL,     
  SCREEN_SETTINGS_THEME,    
  SCREEN_SETTINGS_RESET,
  SCREEN_SETTINGS_ADVANCED, 
  SCREEN_SETTINGS_TIMER,    
  SCREEN_CONFIRM_SAVE,      
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; 
volatile ScreenState previousScreen = SCREEN_HOME; 
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 ---
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7

// ESP-01 핀 정의 (RX, TX) - 하드웨어 연결에 맞게 수정하세요
#define ESP_RX_PIN 2 
#define ESP_TX_PIN 3 

// --- 객체 생성 ---
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS); 
SPISettings spiSettingsTFT(40000000, MSBFIRST, SPI_MODE0);
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0);

// SoftwareSerial 객체
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN); 

// --- JSON 문서 ---
StaticJsonDocument<1024> rxJsonDoc; 
StaticJsonDocument<256> txJsonDoc;  

// --- UI 상수 및 색상 ---
#define SCREEN_WIDTH 320 
#define SCREEN_HEIGHT 240 

bool isDarkMode = false;
uint16_t COLOR_BACKGROUND, COLOR_TEXT_PRIMARY, COLOR_TEXT_SECONDARY;
uint16_t COLOR_BUTTON, COLOR_BUTTON_TEXT, COLOR_BUTTON_OUTLINE, COLOR_GRID;
uint16_t COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_ORANGE, COLOR_MAGENTA, COLOR_DARKGREEN;

// --- 설정 변수 ---
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// 설정 임시 저장 변수
float temp_VOLTAGE_THRESHOLD;
float temp_V_MULTIPLIER;
float temp_I_MULTIPLIER;
int temp_setting_step_index;

// --- [복구] 설정 UI 관련 변수 (컴파일 에러 해결) ---
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

// --- 네트워크 및 ThingSpeak 변수 ---
String wifiSSID = "YOUR_WIFI_SSID"; 
String wifiPASS = "YOUR_WIFI_PASSWORD"; 
String apiKey = "YOUR_THINGSPEAK_API_KEY"; 

bool isWifiEnabled = false;     // WiFi 켜짐/꺼짐 설정
bool isWifiConnected = false;   // 실제 연결 상태
unsigned long lastSendTime = 0; // 마지막 전송 시간
const unsigned long SEND_INTERVAL = 20000; // 20초

// 데이터 전송 선택 플래그
bool send_V = true;
bool send_I = true;
bool send_P = true;

// --- 타이머 변수 ---
bool is_timer_active = false;
uint32_t timer_seconds_left = 0;
uint32_t timer_setting_seconds = 0; 
uint32_t temp_timer_setting_seconds = 0; 

bool prev_is_timer_active = false;
uint32_t prev_timer_seconds_left = 0;
uint32_t prev_temp_timer_setting_seconds = 0xFFFFFFFF; 

const uint32_t TIMER_STEP_VALUES[] = {1, 10, 60, 600, 3600, 36000}; 
const char* TIMER_STEP_LABELS[] = {"1 sec", "10 sec", "1 min", "10 min", "1 hour", "10 hour"};
int timer_step_index = 0; 
int prev_timer_step_index = -1;

float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;
bool settingsChanged = false; 

volatile bool warningActive = false;
String warningMessage = "";

#define TS_RAW_X1 370
#define TS_RAW_Y1 450
#define TS_RAW_X2 3760
#define TS_RAW_Y2 3670

#define PLOT_X_START 60 
#define PLOT_X_END 285 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 190 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0; 

int last_frame_y_plot1[PLOT_WIDTH]; 
int last_frame_y_plot2[PLOT_WIDTH]; 
int last_frame_y_plot3[PLOT_WIDTH]; 

const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
const int NUM_P_STEPS = 6;
const float P_AXIS_STEPS[NUM_P_STEPS] = {50.0, 100.0, 250.0, 500.0, 1000.0, 2000.0};

float plot1_axis_max = V_AXIS_STEPS[4]; 
float plot2_axis_max = I_AXIS_STEPS[2]; 
float plot3_axis_max = I_AXIS_STEPS[2]; 

int waveformPlotType = 0; 
const char* WAVEFORM_TYPE_LABELS[] = {"V/I", "P/V", "I/I1/I2"};
int waveformTriggerMode = 0; 
const char* WAVEFORM_MODE_LABELS[] = {"Cont.", "Trig.", "Single"};
volatile bool isWaveformFrozen = false; 
int waveformPeriodIndex = 1; 
const char* WAVEFORM_PERIOD_LABELS[] = {"1.0C", "1.5C", "3.0C"};

#define PHASOR_CX 235
#define PHASOR_CY 115
#define PHASOR_RADIUS 75

float V_rms = 0.0;
float I_rms = 0.0; 
float I_rms_load1 = 0.0;
float I_rms_load2 = 0.0;
float P_real = 0.0; 
float Q_reactive = 0.0; 
float S_apparent = 0.0;
float PF = 0.0; 

float phase_degrees = 0.0; 
String lead_lag_status = "---"; 
float phase_main_deg = 0.0;
float phase_load1_deg = 0.0;
float phase_load2_deg = 0.0;
float thd_v_value = 0.0;
float32_t thd_i_value = 0.0;

// [복구] THD 이전값 변수
float prev_thd_v = 0.0;
float prev_thd_i = 0.0;

float v_harmonics[16]; 
float i_harmonics[16];
int harmonicsSource = 0;   
int harmonicsViewMode = 0; 
bool isHarmonicsFrozen = false;

bool isMainPowerFrozen = false;
bool isPhaseFrozen = false;

bool relay1_state = false;
bool relay2_state = false;
bool prev_r1_state = false; 
bool prev_r2_state = false; 

float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";
float prev_phase_main_deg = 0.0;
float prev_phase_load1_deg = 0.0;
float prev_phase_load2_deg = 0.0;
int prev_v_x = PHASOR_CX, prev_v_y = PHASOR_CY;
int prev_im_x = PHASOR_CX, prev_im_y = PHASOR_CY;
int prev_i1_x = PHASOR_CX, prev_i1_y = PHASOR_CY;
int prev_i2_x = PHASOR_CX, prev_i2_y = PHASOR_CY;

// --- 함수 프로토타입 ---
// View_Static.ino
void setTheme();
void drawWaveformGridAndLabels();
void updateYAxisLabels();
void displayHomeScreenStatic();
void displayMainScreenStatic();
void displayPhaseScreenStatic();
void displayHarmonicsScreenStatic(); 
void displaySettingsScreenStatic();
void displaySettingsNetworkStatic(); 
void displaySettingsCalibStatic();
void displaySettingsProtectStatic();
void displayRelayControlStatic();
void displaySettingsThemeStatic();
void displaySettingsResetStatic();
void displaySettingsAdvancedStatic();
void displaySettingsTimerStatic();
void displayConfirmSaveStatic();
void displayWarningScreenStatic();

// View_Dynamic.ino
void displayMainScreenValues();
void displayPhaseScreenValues();
void runCombinedWaveformLoop();
void displayHarmonicsScreenValues(); 
void runSettingsNetwork(); 
void displaySettingsCalibValues();
void displaySettingsProtectValues();
void runRelayControl();
void runSettingsTheme();
void displaySettingsTimerValues();

// Comm_Input.ino
void checkTouchInput();
void checkSerialInput();

// AT 커맨드 헬퍼 함수
String sendAT(String command, const int timeout, boolean debug) {
  String response = "";
  espSerial.print(command); // 명령 전송
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (espSerial.available()) {
      char c = espSerial.read();
      response += c;
    }
  }
  if (debug) {
    Serial.print(response);
  }
  return response;
}

// WiFi 연결 함수
bool connectWiFi() {
  sendAT("AT+CWMODE=1\r\n", 1000, true); // Station 모드
  String cmd = "AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPASS + "\"\r\n";
  String response = sendAT(cmd, 5000, true); // 연결 시도 (5초 타임아웃)
  if (response.indexOf("OK") != -1 || response.indexOf("WIFI CONNECTED") != -1) {
    return true;
  }
  return false;
}

// ThingSpeak 전송 함수
void sendToThingSpeak() {
  // 1. 연결 시작
  String cmd = "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
  sendAT(cmd, 2000, false);

  // 2. GET 요청 URL 생성
  String getStr = "GET /update?api_key=" + apiKey;
  if (send_V) getStr += "&field1=" + String(V_rms);
  if (send_I) getStr += "&field2=" + String(I_rms);
  if (send_P) getStr += "&field3=" + String(P_real);
  getStr += "\r\n\r\n";

  // 3. 데이터 길이 전송
  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  cmd += "\r\n";
  sendAT(cmd, 1000, false);

  // 4. 데이터 전송
  sendAT(getStr, 2000, false);
  
  // 5. 연결 종료
  sendAT("AT+CIPCLOSE\r\n", 1000, false);
}

void initDummyHarmonics() {
  for(int i=1; i<=15; i++) {
    if (i == 1) { v_harmonics[i] = 100.0; i_harmonics[i] = 100.0; }
    else {
      if (i % 2 != 0) { v_harmonics[i] = 10.0 / (i - 1); i_harmonics[i] = 15.0 / (i - 1); }
      else { v_harmonics[i] = 0.5 / i; i_harmonics[i] = 0.8 / i; }
    }
  }
  thd_v_value = 0.055; thd_i_value = 0.082; 
}

// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  espSerial.begin(9600); // ESP-01 기본 Baud rate
  Serial.println("Controller.ino Booting...");
  
  // ESP 초기화 (Reset)
  sendAT("AT+RST\r\n", 2000, true);

  initDummyHarmonics(); 

  setTheme(); 
  tft.begin();
  tft.setRotation(3); 
  ts.begin(SPI);
  ts.setRotation(3); 
  Serial.println("TFT & Touch OK");

  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. 터치 입력 확인
  checkTouchInput(); 
  
  // 2. 시리얼 입력 확인
  checkSerialInput();

  // 3. WiFi 연결 관리 및 ThingSpeak 전송
  if (isWifiEnabled) {
    if (isWifiConnected) {
      if (millis() - lastSendTime > SEND_INTERVAL) {
        Serial.println("Sending to ThingSpeak...");
        sendToThingSpeak();
        lastSendTime = millis();
      }
    }
  } else {
    isWifiConnected = false;
  }
  
  // 4. 화면 그리기 로직
  if (screenNeedsRedraw) { 
    tft.fillScreen(COLOR_BACKGROUND); 
    
    if (warningActive) {
      currentScreen = SCREEN_WARNING;
      displayWarningScreenStatic(); 
    } else { 
      switch(currentScreen) {
        case SCREEN_HOME: displayHomeScreenStatic(); break;
        case SCREEN_MAIN_POWER: displayMainScreenStatic(); break;
        case SCREEN_PHASE_DIFFERENCE: displayPhaseScreenStatic(); break;
        case SCREEN_COMBINED_WAVEFORM:
          isWaveformFrozen = false; waveformTriggerMode = 0; 
          if (waveformPlotType == 0) { plot1_axis_max = V_AXIS_STEPS[4]; plot2_axis_max = I_AXIS_STEPS[2]; } 
          else if (waveformPlotType == 1) { plot1_axis_max = P_AXIS_STEPS[3]; plot2_axis_max = V_AXIS_STEPS[4]; } 
          else { plot1_axis_max = I_AXIS_STEPS[2]; plot2_axis_max = I_AXIS_STEPS[2]; plot3_axis_max = I_AXIS_STEPS[2]; }
          drawWaveformGridAndLabels(); updateYAxisLabels();      
          for(int i=0; i<PLOT_WIDTH; i++) { last_frame_y_plot1[i] = PLOT_Y_CENTER; last_frame_y_plot2[i] = PLOT_Y_CENTER; last_frame_y_plot3[i] = PLOT_Y_CENTER; }
          break;
        case SCREEN_HARMONICS: displayHarmonicsScreenStatic(); break;
        case SCREEN_SETTINGS: displaySettingsScreenStatic(); break;
        case SCREEN_SETTINGS_NETWORK: displaySettingsNetworkStatic(); break; 
        case SCREEN_SETTINGS_CALIB: displaySettingsCalibStatic(); prev_V_MULTIPLIER = -1.0; prev_I_MULTIPLIER = -1.0; prev_setting_step_index = -1; break;
        case SCREEN_SETTINGS_PROTECT: displaySettingsProtectStatic(); prev_VOLTAGE_THRESHOLD = -1.0; prev_setting_step_index = -1; break;
        case SCREEN_RELAY_CONTROL: displayRelayControlStatic(); break;
        case SCREEN_SETTINGS_THEME: displaySettingsThemeStatic(); break;
        case SCREEN_SETTINGS_RESET: displaySettingsResetStatic(); break;
        case SCREEN_SETTINGS_ADVANCED: displaySettingsAdvancedStatic(); break;
        case SCREEN_SETTINGS_TIMER: 
          displaySettingsTimerStatic(); prev_is_timer_active = !is_timer_active; prev_timer_seconds_left = -1; prev_temp_timer_setting_seconds = 0xFFFFFFFF; prev_timer_step_index = -1; 
          break;
        case SCREEN_CONFIRM_SAVE: displayConfirmSaveStatic(); break;
        case SCREEN_WARNING: break;
      }
    }
    screenNeedsRedraw = false; 
  }
  
  // 5. 동적 데이터 업데이트
  switch(currentScreen) {
    case SCREEN_MAIN_POWER: displayMainScreenValues(); break;
    case SCREEN_PHASE_DIFFERENCE: displayPhaseScreenValues(); break;
    case SCREEN_COMBINED_WAVEFORM: runCombinedWaveformLoop(); break;
    case SCREEN_HARMONICS: displayHarmonicsScreenValues(); break;
    case SCREEN_SETTINGS_NETWORK: runSettingsNetwork(); break; 
    case SCREEN_SETTINGS_CALIB: displaySettingsCalibValues(); break;
    case SCREEN_SETTINGS_PROTECT: displaySettingsProtectValues(); break;
    case SCREEN_RELAY_CONTROL: runRelayControl(); break;
    case SCREEN_SETTINGS_THEME: runSettingsTheme(); break;
    case SCREEN_SETTINGS_TIMER: displaySettingsTimerValues(); break;
    default: break;
  }
}