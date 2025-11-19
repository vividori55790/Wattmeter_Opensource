/*
 * ==============================================================================
 * 파일명: 1. Wattmeter_MCU2_Controller.ino
 * 버전: v209 (Timer Logic Moved to MCU2, Menu Restructured)
 * 설명: 
 * - [Mod] 타이머 카운트 로직을 MCU2 내부 millis() 기반으로 변경
 * - [Mod] 타이머 종료 시 MCU2가 스스로 트립 명령 전송 및 경고 표시
 * - [Mod] Waveform Labels changed (Short/Mid/Long)
 * - 파형, 고조파, 네트워크 등 기존 기능 유지
 * ==============================================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <arm_math.h> 
#include <ArduinoJson.h>
#include <SoftwareSerial.h> // ESP-01 통신용 (일반적으로 ESP-01은 9600 사용)
#include <EEPROM.h>         // 프리셋 저장용

// --- 화면 상태 정의 ---
enum ScreenState {
  SCREEN_HOME,              
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_HARMONICS,          
  SCREEN_SETTINGS,
  SCREEN_SETTINGS_NETWORK,  
  SCREEN_SETTINGS_CALIB_MENU, 
  SCREEN_SETTINGS_CALIB_MANUAL,    
  SCREEN_SETTINGS_CALIB_AUTO,   
  SCREEN_SETTINGS_PROTECT,  
  SCREEN_RELAY_CONTROL,     
  SCREEN_SETTINGS_THEME,    
  SCREEN_SETTINGS_RESET,
  SCREEN_SETTINGS_ADVANCED, 
  SCREEN_SETTINGS_PRESETS, 
  SCREEN_SETTINGS_TIMER,    
  SCREEN_CONFIRM_SAVE,      
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; 
volatile ScreenState previousScreen = SCREEN_HOME; 
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 (TFT & Touch) ---
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7

// --- 아날로그 센서 핀 정의 (MCU2 직접 연결) ---
#define PIN_ADC_V  A3  
#define PIN_ADC_I  A2  
#define PIN_ADC_I1 A4  
#define PIN_ADC_I2 A5  

// ADC 관련 상수
#define ADC_RESOLUTION_BITS 14        
#define ADC_MAX_VAL (1 << ADC_RESOLUTION_BITS) 
#define ADC_MIDPOINT (ADC_MAX_VAL / 2)

// 물리 상수 (캘리브레이션 기본값)
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

// ESP-01 핀 정의
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
StaticJsonDocument<1536> rxJsonDoc; 
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

// --- 설정 UI 관련 변수 ---
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

// --- Auto-Calibration 관련 변수 ---
int auto_calib_step = 0;
float temp_true_v = 220.0;
float temp_true_i = 1.0;
float prev_temp_true_v = 0.0;
float prev_temp_true_i = 0.0;
char calib_status_msg[50] = "";
const int NUM_AUTOCALIB_INPUTS = 3;
float V_ADC_MIDPOINT_CALIB = ADC_MIDPOINT;
float I_ADC_MIDPOINT_CALIB = ADC_MIDPOINT;

// --- EEPROM 프리셋 관련 ---
struct Preset {
  float v_mult;
  float i_mult;
  float v_thresh;
  bool valid;
};
const int EEPROM_BASE_ADDR = 0;
const int PRESET_SIZE = sizeof(Preset);
bool isPresetSaveMode = false; 

// --- 네트워크 및 ThingSpeak 변수 ---
String wifiSSID = "Mathsaves"; 
String wifiPASS = "19886382"; 
String apiKey = "X50GVA1VU213PQ3Z"; 

// 네트워크 상태 관리 (OFF -> WAIT -> ON)
enum WifiState { WIFI_OFF, WIFI_WAIT, WIFI_CONNECTED_STATE };
WifiState wifiState = WIFI_OFF;
unsigned long lastWifiRetryTime = 0;
const unsigned long WIFI_RETRY_INTERVAL = 10000; // 10초마다 재시도

bool isWifiEnabled = false; // 구버전 호환용 플래그 (표시용)
bool isWifiConnected = false; // 실제 연결 여부   
unsigned long lastSendTime = 0; 
const unsigned long SEND_INTERVAL = 20000; 

bool send_V = true;
bool send_I = true;
bool send_P = true;

// --- 타이머 변수 ---
bool is_timer_active = false;
uint32_t timer_seconds_left = 0;
uint32_t timer_setting_seconds = 0; 
uint32_t temp_timer_setting_seconds = 0; 
// 타이머 타겟 릴레이 (1 -> 2 -> 3(Both))
int timer_target_relay = 1; 
int prev_timer_target_relay = -1;

// [Mod] UI 가시성 제어용 전역 변수 (Dynamic View 참조용)
bool prev_is_timer_active = false; 

uint32_t prev_timer_seconds_left = 0;
uint32_t prev_temp_timer_setting_seconds = 0xFFFFFFFF; 

const uint32_t TIMER_STEP_VALUES[] = {1, 10, 60, 600, 3600, 36000}; 
const char* TIMER_STEP_LABELS[] = {"1s", "10s", "1m", "10m", "1h", "10h"};
int timer_step_index = 0; 
int prev_timer_step_index = -1;
unsigned long last_timer_tick = 0; // [Mod] 내부 타이머 카운트용

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

// 파형 그래프 위치
#define PLOT_X_START 30 
#define PLOT_X_END 315 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 190 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0; 

int last_frame_y_plot1[PLOT_WIDTH]; 
int last_frame_y_plot2[PLOT_WIDTH]; 
int last_frame_y_plot3[PLOT_WIDTH]; 

// [Mod] 미리 지정된 범위들 (Stepped Auto-Ranging Requirements)
const int NUM_V_RANGES = 3;
const float V_RANGES[NUM_V_RANGES] = {100.0, 250.0, 500.0};
const int NUM_I_RANGES = 4;
const float I_RANGES[NUM_I_RANGES] = {2.0, 5.0, 10.0, 20.0};
const int NUM_P_RANGES = 4;
const float P_RANGES[NUM_P_RANGES] = {200.0, 500.0, 1000.0, 2000.0};

float plot1_axis_max = V_RANGES[NUM_V_RANGES-1]; 
float plot2_axis_max = I_RANGES[NUM_I_RANGES-1]; 
float plot3_axis_max = I_RANGES[NUM_I_RANGES-1]; 

int waveformPlotType = 0; 
const char* WAVEFORM_TYPE_LABELS[] = {"V/I", "P/Q", "I/I1/I2"};
int waveformTriggerMode = 0; 
const char* WAVEFORM_MODE_LABELS[] = {"Cont.", "Trig.", "Single"};
volatile bool isWaveformFrozen = false; 
int waveformPeriodIndex = 1; 
const char* WAVEFORM_PERIOD_LABELS[] = {"Short", "Mid", "Long"};
const int WAVEFORM_DELAYS_US[] = {50, 100, 200}; 

#define PHASOR_CX 235
#define PHASOR_CY 115
#define PHASOR_RADIUS 75

float prev_V_rms_main = -1.0;
float prev_I_rms_main = -1.0;

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

float prev_thd_v = 0.0;
float prev_thd_i = 0.0;

float v_harmonics[16]; 
float i_harmonics[16];
int harmonicsSource = 0;   // 0: Voltage, 1: Current
int harmonicsViewMode = 0; 
bool isHarmonicsFrozen = false;

// 고조파 버튼 텍스트 상태 변수
String harmonicsSrcLabel = "Src: V";
String harmonicsRunLabel = "RUN";
String prev_harmonicsSrcLabel = "";
String prev_harmonicsRunLabel = "";

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
void setTheme();
void drawWaveformGridAndLabels();
void updateYAxisLabels();
void displayHomeScreenStatic();
void displayMainScreenStatic();
void displayPhaseScreenStatic();
void displayHarmonicsScreenStatic(); 
void displaySettingsScreenStatic();
void displaySettingsNetworkStatic(); 
void displaySettingsCalibMenuStatic();   
void displaySettingsCalibManualStatic(); 
void displayAutoCalibStatic();           
void displaySettingsProtectStatic();
void displayRelayControlStatic();
void displaySettingsThemeStatic();
void displaySettingsResetStatic();
void displaySettingsAdvancedStatic();
void displayPresetScreenStatic();        
void displaySettingsTimerStatic();
void displayConfirmSaveStatic();
void displayWarningScreenStatic();

void displayMainScreenValues();
void displayPhaseScreenValues();
void runCombinedWaveformLoop();
void displayHarmonicsScreenValues(); 
void runSettingsNetwork(); 
void displaySettingsCalibManualValues();
void runAutoCalib(); 
void displaySettingsProtectValues();
void runRelayControl();
void runSettingsTheme();
void displaySettingsTimerValues();
void runPresetScreen(); 

void checkTouchInput();
void checkSerialInput();
void measureOffsets();     
void calculateNewGains(float true_v, float true_i); 

// AT 커맨드 헬퍼 함수 (Blocking)
String sendAT(String command, const int timeout, boolean debug) {
  String response = "";
  espSerial.print(command); 
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
  sendAT("AT+CWMODE=1\r\n", 1000, true); 
  String cmd = "AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPASS + "\"\r\n";
  String response = sendAT(cmd, 6000, true); // 타임아웃 약간 증가
  if (response.indexOf("OK") != -1 || response.indexOf("WIFI CONNECTED") != -1) {
    return true;
  }
  return false;
}

// 백그라운드 WiFi 처리
void handleNetworkLogic() {
  if (wifiState == WIFI_WAIT) {
    if (millis() - lastWifiRetryTime > WIFI_RETRY_INTERVAL) {
      lastWifiRetryTime = millis();
      // 시도 중임을 알리지 않고 백그라운드 시도 (화면 멈춤 최소화 필요하지만 AT명령 특성상 지연 발생)
      if (connectWiFi()) {
        wifiState = WIFI_CONNECTED_STATE;
        isWifiConnected = true;
        isWifiEnabled = true;
      } else {
        // 연결 실패 시 WAIT 유지
      }
    }
  }
}

// ThingSpeak 전송 함수
void sendToThingSpeak() {
  Serial.println("Sending to ThingSpeak..."); 
  String cmd = "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
  sendAT(cmd, 2000, false);

  String getStr = "GET /update?api_key=" + apiKey;
  if (send_V) getStr += "&field1=" + String(V_rms);
  if (send_I) getStr += "&field2=" + String(I_rms);
  if (send_P) getStr += "&field3=" + String(P_real);
  getStr += "\r\n\r\n";

  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  cmd += "\r\n";
  sendAT(cmd, 1000, false);

  sendAT(getStr, 2000, false);
  sendAT("AT+CIPCLOSE\r\n", 1000, false);
}

void initDummyHarmonics() {
  for(int i=0; i<=15; i++) {
    v_harmonics[i] = 0.0;
    i_harmonics[i] = 0.0;
  }
}

// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);  
  Serial1.begin(115200); // MCU1 통신용
  espSerial.begin(9600); // ESP-01

  Serial.println("MCU2 Controller (Direct ADC & Serial1) Booting...");
  
  analogReadResolution(ADC_RESOLUTION_BITS); 
  pinMode(PIN_ADC_V, INPUT);
  pinMode(PIN_ADC_I, INPUT);
  pinMode(PIN_ADC_I1, INPUT);
  pinMode(PIN_ADC_I2, INPUT);
  
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
  checkTouchInput(); 
  checkSerialInput(); 
  handleNetworkLogic(); // 백그라운드 네트워크 재시도

  if (wifiState == WIFI_CONNECTED_STATE) {
    if (millis() - lastSendTime > SEND_INTERVAL) {
      sendToThingSpeak();
      lastSendTime = millis();
    }
  }
  
  // [Mod] MCU2 내부 타이머 로직 (millis 사용)
  // 타이머 활성화 상태일 때 MCU2가 직접 카운트 다운 및 종료 처리
  if (is_timer_active) {
     if (millis() - last_timer_tick >= 1000) {
        last_timer_tick = millis();
        if (timer_seconds_left > 0) {
           timer_seconds_left--;
        }
        
        // 시간이 0이 되면 트립 수행 (MCU2 주도)
        if (timer_seconds_left == 0) {
           is_timer_active = false;
           
           // 1. 릴레이 차단 명령 전송 (MCU1으로 명령만 전송)
           if (timer_target_relay == 1 || timer_target_relay == 3) {
              relay1_state = false;
              txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_RELAY"; txJsonDoc["ID"] = 1; txJsonDoc["STATE"] = 0;
              serializeJson(txJsonDoc, Serial1); Serial1.println();
           }
           if (timer_target_relay == 2 || timer_target_relay == 3) {
              relay2_state = false;
              txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_RELAY"; txJsonDoc["ID"] = 2; txJsonDoc["STATE"] = 0;
              serializeJson(txJsonDoc, Serial1); Serial1.println();
           }
           
           // 2. 화면에 경고(트립) 표시
           warningMessage = "TIMER END"; 
           warningActive = true;
           screenNeedsRedraw = true;
        }
     }
  }

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
          // 초기 범위 설정
          if (waveformPlotType == 0) { plot1_axis_max = V_RANGES[NUM_V_RANGES-1]; plot2_axis_max = I_RANGES[NUM_I_RANGES-1]; } 
          else if (waveformPlotType == 1) { plot1_axis_max = P_RANGES[NUM_P_RANGES-1]; plot2_axis_max = P_RANGES[NUM_P_RANGES-1]; } 
          else { plot1_axis_max = I_RANGES[NUM_I_RANGES-1]; plot2_axis_max = I_RANGES[NUM_I_RANGES-1]; plot3_axis_max = I_RANGES[NUM_I_RANGES-1]; }
          
          drawWaveformGridAndLabels(); updateYAxisLabels();      
          for(int i=0; i<PLOT_WIDTH; i++) { last_frame_y_plot1[i] = PLOT_Y_CENTER; last_frame_y_plot2[i] = PLOT_Y_CENTER; last_frame_y_plot3[i] = PLOT_Y_CENTER; }
          break;
        case SCREEN_HARMONICS: displayHarmonicsScreenStatic(); break;
        case SCREEN_SETTINGS: displaySettingsScreenStatic(); break;
        case SCREEN_SETTINGS_NETWORK: displaySettingsNetworkStatic(); break; 
        case SCREEN_SETTINGS_CALIB_MENU: displaySettingsCalibMenuStatic(); break; 
        case SCREEN_SETTINGS_CALIB_MANUAL: displaySettingsCalibManualStatic(); prev_V_MULTIPLIER = -1.0; prev_I_MULTIPLIER = -1.0; prev_setting_step_index = -1; break;
        case SCREEN_SETTINGS_CALIB_AUTO: displayAutoCalibStatic(); break; 
        case SCREEN_SETTINGS_PROTECT: displaySettingsProtectStatic(); prev_VOLTAGE_THRESHOLD = -1.0; prev_setting_step_index = -1; break;
        case SCREEN_RELAY_CONTROL: displayRelayControlStatic(); break;
        case SCREEN_SETTINGS_THEME: displaySettingsThemeStatic(); break;
        case SCREEN_SETTINGS_RESET: displaySettingsResetStatic(); break;
        case SCREEN_SETTINGS_ADVANCED: displaySettingsAdvancedStatic(); break;
        case SCREEN_SETTINGS_PRESETS: displayPresetScreenStatic(); break; 
        case SCREEN_SETTINGS_TIMER: 
          displaySettingsTimerStatic(); 
          // [Mod] 타이머 화면 진입 시 플래그 초기화 (버튼 가시성 보장)
          prev_is_timer_active = !is_timer_active; 
          prev_timer_seconds_left = -1; 
          prev_temp_timer_setting_seconds = 0xFFFFFFFF; prev_timer_step_index = -1; 
          prev_timer_target_relay = -1;
          break;
        case SCREEN_CONFIRM_SAVE: displayConfirmSaveStatic(); break;
        case SCREEN_WARNING: break;
      }
    }
    screenNeedsRedraw = false; 
  }
  
  switch(currentScreen) {
    case SCREEN_MAIN_POWER: displayMainScreenValues(); break;
    case SCREEN_PHASE_DIFFERENCE: displayPhaseScreenValues(); break;
    case SCREEN_COMBINED_WAVEFORM: runCombinedWaveformLoop(); break;
    case SCREEN_HARMONICS: displayHarmonicsScreenValues(); break;
    case SCREEN_SETTINGS_NETWORK: runSettingsNetwork(); break; 
    case SCREEN_SETTINGS_CALIB_MANUAL: displaySettingsCalibManualValues(); break;
    case SCREEN_SETTINGS_CALIB_AUTO: runAutoCalib(); break; 
    case SCREEN_SETTINGS_PROTECT: displaySettingsProtectValues(); break;
    case SCREEN_RELAY_CONTROL: runRelayControl(); break;
    case SCREEN_SETTINGS_THEME: runSettingsTheme(); break;
    case SCREEN_SETTINGS_TIMER: displaySettingsTimerValues(); break;
    case SCREEN_SETTINGS_PRESETS: runPresetScreen(); break; 
    default: break;
  }
}

void measureOffsets() {
  long v_sum = 0, i_sum = 0;
  int num_samples = 1000;
  for(int i=0; i<num_samples; i++) {
    v_sum += analogRead(PIN_ADC_V);
    i_sum += analogRead(PIN_ADC_I);
    delay(1);
  }
  V_ADC_MIDPOINT_CALIB = (float)v_sum / num_samples;
  I_ADC_MIDPOINT_CALIB = (float)i_sum / num_samples;
}

void calculateNewGains(float true_v, float true_i) {
  unsigned long V_sq_sum = 0, I_sq_sum = 0;
  int samples = 512;
  
  for (int i = 0; i < samples; i++) { 
    int V_raw = analogRead(PIN_ADC_V); 
    int I_raw = analogRead(PIN_ADC_I); 
    int V_ac = V_raw - (int)V_ADC_MIDPOINT_CALIB;
    int I_ac = I_raw - (int)I_ADC_MIDPOINT_CALIB;
    V_sq_sum += (unsigned long)V_ac * V_ac; 
    I_sq_sum += (unsigned long)I_ac * I_ac; 
    delayMicroseconds(130); 
  }
  
  float V_rms_adc = sqrt((float)V_sq_sum / samples);
  float I_rms_adc = sqrt((float)I_sq_sum / samples); 
  
  if (V_rms_adc > 1) {
    V_MULTIPLIER = true_v / (V_rms_adc * BASE_V_CALIB_RMS);
  }
  if (I_rms_adc > 1) {
    I_MULTIPLIER = true_i / (I_rms_adc * BASE_I_CALIB_RMS);
  }
  
  temp_V_MULTIPLIER = V_MULTIPLIER;
  temp_I_MULTIPLIER = I_MULTIPLIER;
  settingsChanged = true; 
}