/*
 * ==============================================================================
 * 파일명: 1. Wattmeter_MCU2_Controller.ino
 * 버전: v213 (Compile Fix)
 * 설명: 
 * - [Fix] Dynamic_View.ino에서 사용하는 prev_harmonicsSrcLabel, prev_harmonicsRunLabel 변수 선언 추가
 * - [Mod] 파형 로직 Wattmeter.ino 원본 스타일(Trigger Mode)로 복원 (MCU2 샘플링)
 * - [Mod] 트립 화면 로직 개선 (사유 표시, 터치 시 릴레이 제어 화면 이동)
 * - [Mod] 고조파 화면 실제 데이터 연동 및 레이아웃 개선
 * ==============================================================================
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <arm_math.h> 
#include <ArduinoJson.h>
#include <SoftwareSerial.h> 
#include <EEPROM.h>         

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

enum WifiState { WIFI_OFF, WIFI_WAIT, WIFI_CONNECTED_STATE };
WifiState wifiState = WIFI_OFF;
unsigned long lastWifiRetryTime = 0;
const unsigned long WIFI_RETRY_INTERVAL = 10000; 

bool isWifiEnabled = false; 
bool isWifiConnected = false;   
unsigned long lastSendTime = 0; 
const unsigned long SEND_INTERVAL = 20000; 

bool send_V = true;
bool send_I = true;
bool send_P = true;

// --- 타이머 변수 (MCU2 Local) ---
bool is_timer_active = false;
uint32_t timer_seconds_left = 0;
uint32_t timer_setting_seconds = 0; 
uint32_t temp_timer_setting_seconds = 0; 
int timer_target_relay = 1; 
int prev_timer_target_relay = -1;
unsigned long lastTimerTick = 0;

bool prev_is_timer_active = false;
uint32_t prev_timer_seconds_left = 0;
uint32_t prev_temp_timer_setting_seconds = 0xFFFFFFFF; 

const uint32_t TIMER_STEP_VALUES[] = {1, 10, 60, 600, 3600, 36000}; 
const char* TIMER_STEP_LABELS[] = {"1s", "10s", "1m", "10m", "1h", "10h"};
int timer_step_index = 0; 
int prev_timer_step_index = -1;

float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;
bool settingsChanged = false; 

// [Mod] Trip Info
volatile bool warningActive = false;
String warningMessage = "";
String tripReason = ""; // [Mod] 구체적 사유
String trippedRelayInfo = "ALL"; // [Mod] 트립된 릴레이 정보

#define TS_RAW_X1 370
#define TS_RAW_Y1 450
#define TS_RAW_X2 3760
#define TS_RAW_Y2 3670

// 파형 그래프 위치 (Wattmeter.ino 레퍼런스 적용)
#define PLOT_X_START 37 
#define PLOT_X_END 285 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0; 

// [Mod] Wattmeter.ino Style Buffers
int last_frame_y_v[PLOT_WIDTH];
int last_frame_y_i[PLOT_WIDTH];
#define WAVEFORM_SAMPLE_PERIOD_US 100 // Wattmeter.ino standard

// [Mod] Y-axis ranges (Wattmeter.ino Style)
const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
float V_axis_max = V_AXIS_STEPS[4]; // 350V
float I_axis_max = I_AXIS_STEPS[2]; // 1.0A

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

// [Mod] 고조파 데이터 (MCU1에서 수신)
float v_harmonics[16]; 
float i_harmonics[16];
int harmonicsSource = 0;   // 0: Voltage, 1: Current
bool isHarmonicsFrozen = false;
String harmonicsSrcLabel = "Src: V";

// [Fix] Missing Variables for Optimization
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
void waitForVoltageZeroCross(); 
float findAxisStep(float peak, const float* steps, int num_steps); 

void checkTouchInput();
void checkSerialInput();
void measureOffsets();     
void calculateNewGains(float true_v, float true_i); 
void sendJsonCommand(String jsonString); 

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

bool connectWiFi() {
  sendAT("AT+CWMODE=1\r\n", 1000, true); 
  String cmd = "AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPASS + "\"\r\n";
  String response = sendAT(cmd, 6000, true); 
  if (response.indexOf("OK") != -1 || response.indexOf("WIFI CONNECTED") != -1) {
    return true;
  }
  return false;
}

void handleNetworkLogic() {
  if (wifiState == WIFI_WAIT) {
    if (millis() - lastWifiRetryTime > WIFI_RETRY_INTERVAL) {
      lastWifiRetryTime = millis();
      if (connectWiFi()) {
        wifiState = WIFI_CONNECTED_STATE;
        isWifiConnected = true;
        isWifiEnabled = true;
      }
    }
  }
}

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

void setup() {
  Serial.begin(115200);  
  Serial1.begin(115200); 
  espSerial.begin(9600); 

  Serial.println("MCU2 Controller (Wattmeter Style Restored) Booting...");
  
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

void loop() {
  checkTouchInput(); 
  checkSerialInput(); 
  handleNetworkLogic(); 

  if (wifiState == WIFI_CONNECTED_STATE) {
    if (millis() - lastSendTime > SEND_INTERVAL) {
      sendToThingSpeak();
      lastSendTime = millis();
    }
  }
  
  // [Mod] 타이머 로직
  if (is_timer_active) {
      if (millis() - lastTimerTick >= 1000) {
          lastTimerTick = millis();
          if (timer_seconds_left > 0) {
              timer_seconds_left--;
          } else {
              is_timer_active = false;
              // 타이머 종료 시 릴레이 끄기 요청
              if (timer_target_relay == 1 || timer_target_relay == 3) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":0}");
              if (timer_target_relay == 2 || timer_target_relay == 3) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":0}");
              
              warningMessage = "TIMER FINISHED";
              tripReason = "Timer Limit Reached";
              trippedRelayInfo = (timer_target_relay == 1) ? "Relay 1" : (timer_target_relay == 2 ? "Relay 2" : "ALL");
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
          // [Mod] Wattmeter.ino 스타일 복원: 버퍼 초기화 및 그리드
          drawWaveformGridAndLabels(); 
          updateYAxisLabels();      
          for(int i=0; i<PLOT_WIDTH; i++) { 
              last_frame_y_v[i] = PLOT_Y_CENTER; 
              last_frame_y_i[i] = PLOT_Y_CENTER; 
          }
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
          prev_is_timer_active = !is_timer_active; prev_timer_seconds_left = -1; 
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
    case SCREEN_COMBINED_WAVEFORM: 
         // [Mod] Wattmeter.ino의 Waveform 로직 사용 (MCU2 샘플링)
         waitForVoltageZeroCross(); 
         if (screenNeedsRedraw || warningActive) break; // ZeroCross 대기 중 화면 전환 대응
         runCombinedWaveformLoop(); 
         break;
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

// [Mod] Zero Crossing Detection (Wattmeter.ino ported)
void waitForVoltageZeroCross() {
  long startTime = micros();
  long timeout = 20000; 
  int V_ac_bits_prev = 0;
  int V_ac_bits = 0; 
  
  while (true) {
    int V_raw = analogRead(PIN_ADC_V);
    V_ac_bits = V_raw - (int)ADC_MIDPOINT; 
    if (V_ac_bits < -50) {
       V_ac_bits_prev = V_ac_bits;
       break;
    }
    if (micros() - startTime > timeout) return; 
  }

  while (true) {
    int V_raw = analogRead(PIN_ADC_V);
    V_ac_bits = V_raw - (int)ADC_MIDPOINT; 
    if (V_ac_bits_prev < 0 && V_ac_bits >= 0) return;
    V_ac_bits_prev = V_ac_bits;
    if (micros() - startTime > timeout * 2) return; 
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