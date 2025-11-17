// [v55] 타이머 기능 추가 및 'p' 변수 참조 오류 수정
/*
 * ==============================================================================
 * [전력계 2-MCU 분리 시스템 - MCU 2: 제어 및 디스플레이]
 * - v55 변경점:
 * - (BUG FIX) 'checkTouchInput_AutoCalib' 함수 내 'p.x', 'p.y' 참조 오류를 'p_x', 'p_y'로 수정.
 * - (NEW) 'SCREEN_SETTINGS_TIMER' 상태 및 관련 UI/로직 함수 추가.
 * - (NEW) 설정 화면에 "TIMER" 버튼 추가.
 * - (NEW) 타이머 설정 화면에서 시간(분) 설정 및 시작/중지 기능 구현.
 * - (NEW) 타이머 만료 시 MCU 1로 릴레이 토글('T') 명령을 전송하여 릴레이 차단.
 * - (UI) 설정 화면 버튼 레이아웃 조정 (새 버튼 공간 확보).
 * ==============================================================================
 */


// --- 라이브러리 포함 (UI, 통신, 파형) ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
// #include <math.h>
#include <avr/pgmspace.h> // [v54] PSTR
#include <arm_math.h>

// [v45] SoftwareSerial 라이브러리 추가 (ESP-01용)
#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>

// --- [v55] 화면 상태 정의 (Timer 추가) ---
enum ScreenState {
  SCREEN_HOME,
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_HARMONICS,
  SCREEN_SETTINGS,
  SCREEN_SETTINGS_CALIB,    // [v54] Manual Multipliers
  SCREEN_SETTINGS_PROTECT,
  SCREEN_AUTO_CALIB,        // [v54] NEW: 자동 보정 화면
  SCREEN_SETTINGS_TIMER,    // [v55] NEW: 타이머 화면
  SCREEN_RELAY_CONTROL,
  SCREEN_SETTINGS_THEME,
  SCREEN_SETTINGS_RESET,
  SCREEN_CONFIRM_SAVE,
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME;
volatile ScreenState previousScreen = SCREEN_HOME;
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7
#define WIFI_TX_PIN 2
#define WIFI_RX_PIN 3

// --- [v38] WiFi 및 MQTT 설정 ---
#define WIFI_SSID "YOUR_WIFI_SSID"     
#define WIFI_PASS "YOUR_WIFI_PASSWORD" 
#define MQTT_BROKER "public.mqtthq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "wattmeter/data" 

SoftwareSerial espSerial(WIFI_RX_PIN, WIFI_TX_PIN); 
WiFiEspClient wifiClient; 
PubSubClient mqttClient(wifiClient);

long lastMqttPublish = 0;
const long mqttPublishInterval = 5000; 
char wifi_status_msg[10] = "INIT"; 


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

// --- [v54] 기본 보정 계수 (Test.ino 기준) ---
#define DEFAULT_V_ADC_MIDPOINT 8192.0
#define DEFAULT_I_ADC_MIDPOINT 8192.0
#define DEFAULT_I1_ADC_MIDPOINT 8192.0
#define DEFAULT_I2_ADC_MIDPOINT 8192.0
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

// --- [v20] 실시간 설정 변수 (Multiplier, Threshold) ---
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// --- [v54] MCU 1로부터 수신/저장할 전체 보정 계수 ---
float mcu_v_midpoint = DEFAULT_V_ADC_MIDPOINT;
float mcu_i_midpoint = DEFAULT_I_ADC_MIDPOINT;
float mcu_i1_midpoint = DEFAULT_I1_ADC_MIDPOINT;
float mcu_i2_midpoint = DEFAULT_I2_ADC_MIDPOINT;
float mcu_v_gain = BASE_V_CALIB_RMS;
float mcu_i_gain = BASE_I_CALIB_RMS;
float mcu_i1_gain = BASE_I_CALIB_RMS;
float mcu_i2_gain = BASE_I_CALIB_RMS;
float mcu_v_offset_adjust = BASE_V_OFFSET_ADJUST;
float mcu_i_offset_adjust = BASE_I_OFFSET_ADJUST;
float mcu_i1_offset_adjust = BASE_I_OFFSET_ADJUST;
float mcu_i2_offset_adjust = BASE_I_OFFSET_ADJUST;


// --- [v25] 설정 임시 저장 변수 ---
float temp_VOLTAGE_THRESHOLD;
float temp_V_MULTIPLIER;
float temp_I_MULTIPLIER;
int temp_setting_step_index;

// --- [v55] 타이머 임시 저장 변수 ---
float temp_timerDurationMinutes = 30; // 타이머 설정 화면용 임시 변수
int timer_selection = 0; // 0 = duration, 1 = step
int prev_timer_selection = -1;
float prev_temp_timerDurationMinutes = -1.0;


// --- [v20] 설정 화면용 이전 값 ---
float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;

bool settingsChanged = false;

volatile bool warningActive = false;
String warningMessage = "";

// --- [v40] FFT 상수 (파형 표시에 필요한 부분만) ---
#define SAMPLING_FREQ_HZ 7680.0f
#define NUM_HARMONICS_TO_RECEIVE 20 

// --- 캘리브레이션 상수 (ADC 오프셋) ---
// [v54] 로컬 파형 표시용 ADC Midpoint (MCU1에서 받은 값으로 갱신됨)
float V_ADC_MIDPOINT_LOCAL = DEFAULT_V_ADC_MIDPOINT; 
float I_ADC_MIDPOINT_LOCAL = DEFAULT_I_ADC_MIDPOINT; 
float I1_ADC_MIDPOINT_LOCAL = DEFAULT_I1_ADC_MIDPOINT;
float I2_ADC_MIDPOINT_LOCAL = DEFAULT_I2_ADC_MIDPOINT;


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

// --- 파형 화면 상수 (유지) ---
#define PLOT_X_START 30 
#define PLOT_X_END 290 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
#define WAVEFORM_SAMPLE_PERIOD_US 100 

volatile bool show_V = true, show_I = true, show_I1 = false, show_I2 = false, show_P = false, show_Q = false;
int last_frame_y_v[PLOT_WIDTH], last_frame_y_i[PLOT_WIDTH], last_frame_y_i1[PLOT_WIDTH], last_frame_y_i2[PLOT_WIDTH], last_frame_y_p[PLOT_WIDTH], last_frame_y_q[PLOT_WIDTH];
const int V_DELAY_SAMPLES = 42; 
float v_delay_buffer[V_DELAY_SAMPLES] = {0.0};
int v_delay_index = 0;
const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};
const int NUM_PQ_STEPS = 6; 
const float PQ_AXIS_STEPS[NUM_PQ_STEPS] = {10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0};
float V_axis_max = V_AXIS_STEPS[4], I_axis_max = I_AXIS_STEPS[2], P_axis_max = PQ_AXIS_STEPS[2], Q_axis_max = PQ_AXIS_STEPS[2];

// --- [v19] 페이저 다이어그램 상수 ---
#define PHASOR_CX 235
#define PHASOR_CY 130
#define PHASOR_RADIUS 75

// --- 전역 변수 (물리량) - MCU 1로부터 수신 ---
float V_rms = 0.0, I_rms = 0.0, I_rms_load1 = 0.0, I_rms_load2 = 0.0;
float P_real = 0.0, Q_reactive = 0.0, S_apparent = 0.0, PF = 0.0; 
float phase_degrees = 0.0;
String lead_lag_status = "---"; 
float phase_main_deg = 0.0, phase_load1_deg = 0.0, phase_load2_deg = 0.0;
float32_t thd_v = 0.0, thd_i = 0.0;
float v_harmonics[NUM_HARMONICS_TO_RECEIVE], i_harmonics[NUM_HARMONICS_TO_RECEIVE];
volatile bool relay1_state_from_mcu1 = true;
volatile bool relay2_state_from_mcu1 = true;

// --- 전역 변수 (이전 값 - 화면 클리어용) ---
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";
float prev_phase_main_deg = 0.0, prev_phase_load1_deg = 0.0, prev_phase_load2_deg = 0.0;
int prev_v_x = PHASOR_CX, prev_v_y = PHASOR_CY;
int prev_im_x = PHASOR_CX, prev_im_y = PHASOR_CY;
int prev_i1_x = PHASOR_CX, prev_i1_y = PHASOR_CY;
int prev_i2_x = PHASOR_CX, prev_i2_y = PHASOR_CY;
float32_t prev_thd_v = 0.0, prev_thd_i = 0.0;
float prev_v_harmonics[NUM_HARMONICS_TO_RECEIVE] = {0};
float prev_i_harmonics[NUM_HARMONICS_TO_RECEIVE] = {0};
int prev_bar_heights[NUM_HARMONICS_TO_RECEIVE] = {0}; 

// [v49] Harmonics 화면 상태 변수
volatile bool showHarmonicsAsList = false;
volatile bool showVoltageHarmonics = true;
int list_scroll_offset = 0; 
int prev_list_scroll_offset = -1;
const int harmonics_per_page = 10;
const int num_harmonic_pages = 2; 

// [v47] 화면별 메뉴 표시 상태 (수정)
volatile bool showMainMenu = false;
volatile bool showPhasorMenu = false;
volatile bool showWaveformMenu = false;
volatile bool showHarmonicsMenu = false;
volatile bool showRelayMenu = false;

// --- [v23] 설정 UI용 전역 변수 (유지) ---
float setting_steps[] = {0.0001, 0.01, 0.1, 1.0, 10.0, 30.0, 60.0}; // [v55] 30, 60분 스텝 추가
const int NUM_SETTING_STEPS = 7;
int setting_step_index = DEFAULT_SETTING_STEP_INDEX;
int prev_setting_step_index = -1;
int calib_selection = 0;
int prev_calib_selection = -1;
const int NUM_CALIB_SETTINGS = 3; 
int protect_selection = 0;
int prev_protect_selection = -1;
const int NUM_PROTECT_SETTINGS = 2;

// --- [v55] 타이머 전역 변수 ---
volatile unsigned long timerStartTime = 0;
volatile unsigned long timerDurationMinutes = 30; // 실제 적용된 타이머 시간 (기본 30분)
volatile bool timerIsRunning = false;
String prev_timer_status = "";
String prev_time_left = "";


// --- [v54] 자동 보정(Auto-Calibration)용 전역 변수 (Test.ino에서 이식) ---
int calib_step = 0;
float temp_true_v = 220.0;
float temp_true_i = 1.0;
float prev_temp_true_v = 0.0;
float prev_temp_true_i = 0.0;
const int NUM_AUTOCALIB_INPUTS = 3; // V_True, I_True, Step
char calib_status_msg[50] = "";


// --- [v54] 통신용 데이터 구조체 (헤더 'D' 추가) ---
struct PowerData {
  char command_char; // 'D'
  float V_rms;
  float I_rms;
  float I_rms_load1;
  float I_rms_load2;
  float P_real;
  float Q_reactive;
  float S_apparent;
  float PF;
  float phase_main_deg;
  float phase_load1_deg;
  float phase_load2_deg;
  float thd_v;
  float thd_i;
  float fuzzy_output_level;
  float voltage_threshold;
  float v_harmonics[NUM_HARMONICS_TO_RECEIVE];
  float i_harmonics[NUM_HARMONICS_TO_RECEIVE];
  bool relay1_trip_command;
  bool relay2_trip_command;
  bool actual_relay1_state;
  bool actual_relay2_state;
};
PowerData receivedData;

// --- [v52] MCU 1로 전송할 제어 명령 구조체 ---
struct ControlRequest {
  char command_char; // 'T' (Toggle), 'R' (Reset)
  char relay_num;    // '1', '2' (Toggle용)
};
ControlRequest controlReq;

// --- [v53] MCU 1과 주고받을 설정/보정 구조체 ---
struct SettingsPacket {
  char command_char; // MUST BE 'S'
  float v_multiplier;
  float i_multiplier;
  float voltage_threshold;
  float v_midpoint;
  float i_midpoint;
  float i1_midpoint;
  float i2_midpoint;
  float v_gain;
  float i_gain;
  float i1_gain;
  float i2_gain;
  float v_offset_adjust;
  float i_offset_adjust;
  float i1_offset_adjust;
  float i2_offset_adjust;
};

// --- [v54] MCU 1로 전송할 보정 명령 구조체 ---
struct CalibrationRequest {
  char command_char; // 'C'
  char calib_step; // '1' (offsets), '2' (gains)
  float true_v;    // for step 2
  float true_i;    // for step 2
};

// [v49] 메뉴 그리기 함수 프로토타입 (수정)
void drawMainMenu();
void drawPhasorMenu();
void drawWaveformMenu();
void drawHarmonicsMenu(); 
void drawRelayMenu();
void drawCheckbox(int x, int y, int w, int h, String text, bool isChecked, uint16_t color);
void displayHarmonicsGraphStatic();
void displayHarmonicsGraphValues();
void displayHarmonicsListStatic();
void displayHarmonicsListValues();
// [v54] 자동 보정 함수 프로토타입
void displayAutoCalibStatic();
void runAutoCalib();
void displayCalibrationResults();
void adjustAutoCalibValue(bool increase);
void sendCalibrationCommand(char step, float v, float i);
void sendCurrentSettingsToMCU1();
// [v55] 타이머 함수 프로토타입
void displaySettingsTimerStatic();
void runSettingsTimer();
void adjustTimerValue(bool increase);


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("MCU 2 (Controller) v55 Booting..."));

  Serial1.begin(115200); 
  
  setTheme(); 
  
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  tft.begin();
  tft.setRotation(3); 
  ts.begin(SPI);
  ts.setRotation(3); 
  Serial.println(F("TFT & Touch OK"));

  setup_wifi();
  
  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop (v54 - 멀티 패킷 리스너)
// ==============================================================================
void loop() {
  checkTouchInput();
  
  // --- [v54] MCU 1로부터 멀티-패킷 수신 ---
  while (Serial1.available() > 0) {
    char command_char = Serial1.peek();
    
    if (command_char == 'D' && Serial1.available() >= sizeof(PowerData)) {
      // --- PowerData Packet ('D') ---
      Serial1.readBytes((uint8_t*)&receivedData, sizeof(PowerData));
      
      if (receivedData.command_char == 'D') {
        // 수신된 데이터로 전역 변수 업데이트
        V_rms = receivedData.V_rms;
        I_rms = receivedData.I_rms;
        I_rms_load1 = receivedData.I_rms_load1;
        I_rms_load2 = receivedData.I_rms_load2;
        P_real = receivedData.P_real;
        Q_reactive = receivedData.Q_reactive;
        S_apparent = receivedData.S_apparent;
        PF = receivedData.PF;
        phase_main_deg = receivedData.phase_main_deg;
        phase_load1_deg = receivedData.phase_load1_deg;
        phase_load2_deg = receivedData.phase_load2_deg;
        thd_v = receivedData.thd_v; 
        thd_i = receivedData.thd_i; 
        
        for(int i=0; i<NUM_HARMONICS_TO_RECEIVE; i++) {
          v_harmonics[i] = receivedData.v_harmonics[i];
          i_harmonics[i] = receivedData.i_harmonics[i];
        }
        
        relay1_state_from_mcu1 = receivedData.actual_relay1_state;
        relay2_state_from_mcu1 = receivedData.actual_relay2_state;

        // 수신된 값으로 로컬 *경고* 로직만 실행
        controlRelays(receivedData.fuzzy_output_level);
        
        if (V_rms > receivedData.voltage_threshold) {
          warningMessage = "OVER VOLTAGE!"; 
          warningActive = true;
          screenNeedsRedraw = true; 
        }
      } else {
         Serial.println(F("MCU2: PowerData header mismatch!"));
         // 간단한 동기화 시도: 'D'가 나올 때까지 버퍼를 비움
         while(Serial1.available() && Serial1.peek() != 'D' && Serial1.peek() != 'S') Serial1.read();
      }
    } 
    else if (command_char == 'S' && Serial1.available() >= sizeof(SettingsPacket)) {
      // --- SettingsPacket Packet ('S') ---
      SettingsPacket newSettings;
      Serial1.readBytes((uint8_t*)&newSettings, sizeof(SettingsPacket));
      
      if (newSettings.command_char == 'S') {
        Serial.println(F("MCU2: Received SettingsPacket from MCU1."));
        // Update settings globals
        V_MULTIPLIER = newSettings.v_multiplier;
        I_MULTIPLIER = newSettings.i_multiplier;
        VOLTAGE_THRESHOLD = newSettings.voltage_threshold;
        
        // Update full calibration globals
        mcu_v_midpoint = newSettings.v_midpoint;
        mcu_i_midpoint = newSettings.i_midpoint;
        mcu_i1_midpoint = newSettings.i1_midpoint;
        mcu_i2_midpoint = newSettings.i2_midpoint;
        mcu_v_gain = newSettings.v_gain;
        mcu_i_gain = newSettings.i_gain;
        mcu_i1_gain = newSettings.i1_gain;
        mcu_i2_gain = newSettings.i2_gain;
        mcu_v_offset_adjust = newSettings.v_offset_adjust;
        mcu_i_offset_adjust = newSettings.i_offset_adjust;
        mcu_i1_offset_adjust = newSettings.i1_offset_adjust;
        mcu_i2_offset_adjust = newSettings.i2_offset_adjust;

        // [v54] 로컬 파형 표시용 Midpoint도 갱신
        V_ADC_MIDPOINT_LOCAL = mcu_v_midpoint;
        I_ADC_MIDPOINT_LOCAL = mcu_i_midpoint;
        I1_ADC_MIDPOINT_LOCAL = mcu_i1_midpoint;
        I2_ADC_MIDPOINT_LOCAL = mcu_i2_midpoint;
        
        // 자동 보정 중이었다면, 이 패킷이 다음 단계로 넘어가는 트리거가 됨
        if (currentScreen == SCREEN_AUTO_CALIB) {
          if (calib_step == 1) { // 1: "Measuring Offsets..."
            calib_step = 2; // 2: "Connect Load"
            screenNeedsRedraw = true;
          } else if (calib_step == 3) { // 3: "Calculating Gains..."
            calib_step = 4; // 4: "Verify"
            screenNeedsRedraw = true;
          }
        }
      } else {
         Serial.println(F("MCU2: SettingsPacket header mismatch!"));
         while(Serial1.available()) Serial1.read(); // Flush
      }
    } 
    else if (Serial1.available() > 0) {
      // 알 수 없는 바이트 (헤더가 D도 S도 아님), 버퍼 비우기
      Serial.print(F("MCU2: Unknown byte in buffer: 0x")); Serial.println(Serial1.read(), HEX);
    }
  } // end while
  
  
  // --- WiFi/MQTT 로직 (기존과 동일) ---
  if (WiFi.status() != WL_CONNECTED) {
    reconnect_mqtt();
  } else if (!mqttClient.connected()) {
    reconnect_mqtt();
  }
  mqttClient.loop();
  
  // --- [v55] 화면 다시 그리기 로직 (Timer 추가) ---
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
          for(int i=0; i<PLOT_WIDTH; i++) { // 파형 버퍼 클리어
              last_frame_y_v[i] = PLOT_Y_CENTER; 
              last_frame_y_i[i] = PLOT_Y_CENTER; 
              last_frame_y_i1[i] = PLOT_Y_CENTER;
              last_frame_y_i2[i] = PLOT_Y_CENTER;
              last_frame_y_p[i] = PLOT_Y_CENTER; 
              last_frame_y_q[i] = PLOT_Y_CENTER;
          }
          break;
        case SCREEN_HARMONICS: 
          if (showHarmonicsAsList) displayHarmonicsListStatic();
          else displayHarmonicsGraphStatic();
          break;
        case SCREEN_SETTINGS:
          displaySettingsScreenStatic();
          break;
        case SCREEN_SETTINGS_CALIB: // Manual Multipliers
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
        case SCREEN_AUTO_CALIB: // [v54] New
          displayAutoCalibStatic(); 
          break;
        case SCREEN_SETTINGS_TIMER: // [v55] New
          displaySettingsTimerStatic();
          prev_temp_timerDurationMinutes = -1.0;
          prev_timer_selection = -1;
          prev_setting_step_index = -1;
          prev_timer_status = "";
          prev_time_left = "";
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
      
      // 메뉴가 활성화된 경우, 화면 위에 덮어쓰기
      if (showMainMenu) drawMainMenu();
      if (showPhasorMenu) drawPhasorMenu();
      if (showWaveformMenu) drawWaveformMenu();
      if (showHarmonicsMenu) drawHarmonicsMenu();
      if (showRelayMenu) drawRelayMenu();
    }
    screenNeedsRedraw = false; 
  }
  
  // --- [v55] 동적 UI 업데이트 (Timer 추가) ---
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      if (screenNeedsRedraw || warningActive) break;
      if (!showMainMenu) { 
        displayMainScreenValues(); 
        if (millis() - lastMqttPublish > mqttPublishInterval && mqttClient.connected()) {
          publishData(); 
        }
      }
      break;
      
    case SCREEN_PHASE_DIFFERENCE:
      if (screenNeedsRedraw || warningActive) break;
      if (!showPhasorMenu) { 
        displayPhaseScreenValues(); 
      }
      break;
      
    case SCREEN_COMBINED_WAVEFORM:
      waitForVoltageZeroCross(); 
      if (screenNeedsRedraw || warningActive) break; 
      if (!showWaveformMenu) { 
        runCombinedWaveformLoop(); 
      }
      break;
      
    case SCREEN_HARMONICS: 
      if (screenNeedsRedraw || warningActive) break; 
      if (!showHarmonicsMenu) { 
        if (showHarmonicsAsList) displayHarmonicsListValues();
        else displayHarmonicsGraphValues(); 
      }
      break;
      
    case SCREEN_SETTINGS_CALIB: // Manual Multipliers
      runSettingsCalib();
      break;
    case SCREEN_SETTINGS_PROTECT:
      runSettingsProtect();
      break;
    case SCREEN_AUTO_CALIB: // [v54] New
      runAutoCalib();
      break;
    case SCREEN_SETTINGS_TIMER: // [v55] New
      runSettingsTimer();
      break;
    case SCREEN_RELAY_CONTROL: 
      if (!showRelayMenu) { 
        runRelayControl();
      }
      break;
      
    case SCREEN_HOME: 
    case SCREEN_SETTINGS:
    case SCREEN_SETTINGS_THEME: 
    case SCREEN_SETTINGS_RESET: 
    case SCREEN_CONFIRM_SAVE: 
    case SCREEN_WARNING: 
      // 정적 화면
      break;
  }
}


// ==============================================================================
// 3. [v55] 터치 입력 확인 함수 (Timer, AutoCalib 수정)
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
    
    // --- [v20] 공통 경고 화면 터치 ---
    if (currentScreen == SCREEN_WARNING) { 
      warningActive = false;
      // MCU 1로 리셋 명령 전송
      controlReq.command_char = 'R'; // Reset
      controlReq.relay_num = '0';
      Serial1.write((uint8_t*)&controlReq, sizeof(ControlRequest));
      
      currentScreen = SCREEN_HOME;
      screenNeedsRedraw = true;
      delay(100); 
      return; 
    }

    // --- [v55] 공통 뒤로 가기 버튼 (<) ---
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
        else if (currentScreen == SCREEN_SETTINGS_THEME || 
                 currentScreen == SCREEN_SETTINGS_RESET || 
                 currentScreen == SCREEN_CONFIRM_SAVE || 
                 currentScreen == SCREEN_AUTO_CALIB ||
                 currentScreen == SCREEN_SETTINGS_TIMER) { // [v55]
          currentScreen = SCREEN_SETTINGS;
        }
        else if (currentScreen == SCREEN_SETTINGS || currentScreen == SCREEN_RELAY_CONTROL) {
          currentScreen = SCREEN_HOME;
        }
        else {
          showMainMenu = false;
          showPhasorMenu = false;
          showWaveformMenu = false;
          showHarmonicsMenu = false;
          showRelayMenu = false;
          currentScreen = SCREEN_HOME;
        }
        
        screenNeedsRedraw = true;
        delay(100); // Debounce
        return;
      }
    }

    // --- [v49] 공통 메뉴 버튼 (...) ---
    if (currentScreen == SCREEN_MAIN_POWER || 
        currentScreen == SCREEN_PHASE_DIFFERENCE || 
        currentScreen == SCREEN_COMBINED_WAVEFORM || 
        currentScreen == SCREEN_HARMONICS || 
        currentScreen == SCREEN_RELAY_CONTROL) { 
      
      if (p.x >= 265 && p.x <= 315 && p.y >= 5 && p.y <= 35) { // 메뉴 버튼 영역
        switch(currentScreen) {
          case SCREEN_MAIN_POWER: showMainMenu = !showMainMenu; break;
          case SCREEN_PHASE_DIFFERENCE: showPhasorMenu = !showPhasorMenu; break;
          case SCREEN_COMBINED_WAVEFORM: showWaveformMenu = !showWaveformMenu; break;
          case SCREEN_HARMONICS: showHarmonicsMenu = !showHarmonicsMenu; break;
          case SCREEN_RELAY_CONTROL: showRelayMenu = !showRelayMenu; break;
          default: break;
        }
        screenNeedsRedraw = true;
        delay(100); 
        return;
      }
    }


    // --- [v55] 화면별 버튼 로직 ---
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
          currentScreen = SCREEN_HARMONICS; screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_RELAY_CONTROL; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS: // [v55] Y 좌표 수정
        if (p.x >= 20 && p.x <= 300 && p.y >= 50 && p.y <= 80) { // Manual Multipliers
          temp_V_MULTIPLIER = V_MULTIPLIER;
          temp_I_MULTIPLIER = I_MULTIPLIER;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_CALIB;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 85 && p.y <= 115) { // Protection
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_PROTECT;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 120 && p.y <= 150) { // Auto Calibration
          calib_step = 0; // 보정 프로세스 0단계부터 시작
          currentScreen = SCREEN_AUTO_CALIB;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 155 && p.y <= 185) { // [v55] TIMER
          temp_timerDurationMinutes = timerDurationMinutes;
          // settingsChanged = false; // 타이머는 저장/취소 로직을 사용하지 않음
          timer_selection = 0;
          setting_step_index = 3; // 1.0 스텝으로 기본값
          currentScreen = SCREEN_SETTINGS_TIMER;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 190 && p.y <= 230) { // Theme
          currentScreen = SCREEN_SETTINGS_THEME;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 190 && p.y <= 230) { // Reset
          currentScreen = SCREEN_SETTINGS_RESET;
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_CALIB: // Manual Multipliers
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) { // UP
          calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) { // DOWN
          calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS;
        }
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) { // -
          adjustCalibValue(false);
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) { // +
          adjustCalibValue(true);
        }
        break;
        
      case SCREEN_SETTINGS_PROTECT:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) { // UP
          protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS;
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) { // DOWN
          protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS;
        }
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) { // -
          adjustProtectValue(false);
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) { // +
          adjustProtectValue(true);
        }
        break;

      // [v54] NEW: 자동 보정 화면 터치 로직
      case SCREEN_AUTO_CALIB:
        checkTouchInput_AutoCalib(p.x, p.y);
        break;
        
      // [v55] NEW: 타이머 설정 화면 터치 로직
      case SCREEN_SETTINGS_TIMER:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) { // UP
          timer_selection = (timer_selection - 1 + 2) % 2; // 2 = 항목 개수 (Duration, Step)
        }
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) { // DOWN
          timer_selection = (timer_selection + 1) % 2;
        }
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) { // -
          adjustTimerValue(false);
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) { // +
          adjustTimerValue(true);
        }
        // START/STOP 버튼 (중앙)
        else if (p.x >= 80 && p.x <= 240 && p.y >= 130 && p.y <= 170) {
          if (timerIsRunning) {
            timerIsRunning = false;
          } else {
            if (temp_timerDurationMinutes > 0) {
              timerDurationMinutes = (unsigned long)temp_timerDurationMinutes;
              timerStartTime = millis();
              timerIsRunning = true;
            }
          }
          screenNeedsRedraw = true; // 버튼 텍스트 및 상태 갱신
        }
        break;
        
      case SCREEN_RELAY_CONTROL:
        if (!showRelayMenu) { 
          if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) { // Relay 1
            controlReq.command_char = 'T'; 
            controlReq.relay_num = '1';
            Serial1.write((uint8_t*)&controlReq, sizeof(controlReq));
            screenNeedsRedraw = true;
          }
          else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) { // Relay 2
            controlReq.command_char = 'T';
            controlReq.relay_num = '2';
            Serial1.write((uint8_t*)&controlReq, sizeof(controlReq));
            screenNeedsRedraw = true; 
          }
        }
        break;

      case SCREEN_SETTINGS_THEME:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) { // Light
          isDarkMode = false; setTheme(); screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) { // Dark
          isDarkMode = true; setTheme(); screenNeedsRedraw = true;
        }
        break;
        
      case SCREEN_SETTINGS_RESET:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) { // RESET
          restoreDefaultSettings();
          sendCurrentSettingsToMCU1(); // [v54] 리셋된 설정을 MCU1로 전송
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) { // CANCEL
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_CONFIRM_SAVE:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) { // DISCARD
          currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) { // SAVE
          if (previousScreen == SCREEN_SETTINGS_CALIB) {
            V_MULTIPLIER = temp_V_MULTIPLIER;
            I_MULTIPLIER = temp_I_MULTIPLIER;
          } else if (previousScreen == SCREEN_SETTINGS_PROTECT) {
            VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD;
          }
          setting_step_index = temp_setting_step_index; 
          
          sendCurrentSettingsToMCU1(); // [v54] 저장된 설정을 MCU 1로 전송
          
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        break;

      // ... (Waveform, Harmonics 등 나머지 화면 로직은 기존과 동일) ...
      
      case SCREEN_COMBINED_WAVEFORM:
        if (showWaveformMenu) { 
          int menuX = 40, menuW = 240;
          int y_start = 40 + 5; 
          int item_h = 30, y_margin = 5;
          if (p.x >= menuX && p.x <= menuX + menuW) {
            if (p.y >= y_start && p.y <= y_start + item_h) {
              show_V = !show_V; drawWaveformMenu(); 
            } else if (p.y >= y_start + (item_h + y_margin) * 1 && p.y <= y_start + (item_h + y_margin) * 1 + item_h) {
              show_I = !show_I; drawWaveformMenu();
            } else if (p.y >= y_start + (item_h + y_margin) * 2 && p.y <= y_start + (item_h + y_margin) * 2 + item_h) {
              show_I1 = !show_I1; drawWaveformMenu();
            } else if (p.y >= y_start + (item_h + y_margin) * 3 && p.y <= y_start + (item_h + y_margin) * 3 + item_h) {
              show_I2 = !show_I2; drawWaveformMenu();
            } else if (p.y >= y_start + (item_h + y_margin) * 4 && p.y <= y_start + (item_h + y_margin) * 4 + item_h) {
              show_P = !show_P; drawWaveformMenu();
            } else if (p.y >= y_start + (item_h + y_margin) * 5 && p.y <= y_start + (item_h + y_margin) * 5 + item_h) {
              show_Q = !show_Q; drawWaveformMenu();
            }
          }
        }
        break;
        
      case SCREEN_HARMONICS: 
        if (showHarmonicsMenu) {
          int menuX = 60, menuW = 200;
          int y_button1 = 50 + 50; 
          int y_button2 = 50 + 100;
          int btn_h = 35;
          if (p.x >= menuX + 10 && p.x <= menuX + menuW - 10 && p.y >= y_button1 && p.y <= y_button1 + btn_h) {
            showHarmonicsAsList = !showHarmonicsAsList;
            drawHarmonicsMenu(); screenNeedsRedraw = true; 
          }
          if (!showHarmonicsAsList && (p.x >= menuX + 10 && p.x <= menuX + menuW - 10 && p.y >= y_button2 && p.y <= y_button2 + btn_h)) {
            showVoltageHarmonics = !showVoltageHarmonics;
            drawHarmonicsMenu(); screenNeedsRedraw = true; 
          }
        } else {
          if (showHarmonicsAsList) {
            if (p.x >= 260 && p.x <= 310 && p.y >= 70 && p.y <= 110) { // Up
              list_scroll_offset = (list_scroll_offset - 1 + num_harmonic_pages) % num_harmonic_pages;
              screenNeedsRedraw = true; 
            }
            else if (p.x >= 260 && p.x <= 310 && p.y >= 130 && p.y <= 170) { // Down
              list_scroll_offset = (list_scroll_offset + 1) % num_harmonic_pages;
              screenNeedsRedraw = true; 
            }
            else if (p.x > 10 && p.x < 250 && p.y > 40 && p.y < 230) { // Tap list
              showHarmonicsAsList = false; screenNeedsRedraw = true;
            }
          } else {
            if (p.x > 10 && p.x < 310 && p.y > 40 && p.y < 230) { // Tap graph
              showHarmonicsAsList = true; list_scroll_offset = 0; screenNeedsRedraw = true;
            }
          }
        }
        break;
        
      case SCREEN_MAIN_POWER:
      case SCREEN_PHASE_DIFFERENCE:
        break;
    }
    
    delay(100); // Debounce
  }
}

// ==============================================================================
// [v55] 자동 보정 화면 전용 터치 로직 (BUG FIX)
// ==============================================================================
void checkTouchInput_AutoCalib(int p_x, int p_y) {
  switch (calib_step) {
    case 0: // "START"
      if (p_x >= 80 && p_x <= 240 && p_y >= 120 && p_y <= 160) {
        sendCalibrationCommand('1', 0, 0); // Calib Step 1 (Measure Offsets)
        calib_step = 1; // "Measuring..." 상태로 변경
        screenNeedsRedraw = true;
      }
      break;
    case 1: // 측정 중... (터치 없음)
      break;
    case 2: // "NEXT"
      // [v55] BUG FIX: p.y -> p_y
      if (p_x >= 80 && p_x <= 240 && p_y >= 130 && p_y <= 170) {
        calib_step = 3; // 값 입력 화면으로
        temp_true_v = 220.0;
        temp_true_i = 1.0;
        prev_temp_true_v = 0.0;
        prev_temp_true_i = 0.0;
        calib_selection = 0; 
        prev_calib_selection = -1;
        setting_step_index = 2; // 0.1 스텝
        prev_setting_step_index = -1; 
        screenNeedsRedraw = true;
      }
      break;
    case 3: // 값 입력 화면 (UP/DOWN/+/-, CALCULATE)
      // [v55] BUG FIX: p.y -> p_y
      if (p_x >= 20 && p_x <= 80 && p_y >= 180 && p_y <= 220) { // UP
        calib_selection = (calib_selection - 1 + NUM_AUTOCALIB_INPUTS) % NUM_AUTOCALIB_INPUTS;
      }
      // [v55] BUG FIX: p.y -> p_y
      else if (p_x >= 90 && p_x <= 150 && p_y >= 180 && p_y <= 220) { // DOWN
        calib_selection = (calib_selection + 1) % NUM_AUTOCALIB_INPUTS;
      }
      // [v55] BUG FIX: p.y -> p_y
      else if (p_x >= 180 && p_x <= 240 && p_y >= 180 && p_y <= 220) { // -
        adjustAutoCalibValue(false); // Decrease
        screenNeedsRedraw = false; // 값만 업데이트
      }
      // [v55] BUG FIX: p.y -> p_y
      else if (p_x >= 250 && p_x <= 310 && p_y >= 180 && p_y <= 220) { // +
        adjustAutoCalibValue(true); // Increase
        screenNeedsRedraw = false; // 값만 업데이트
      }
      // "CALCULATE" (확인) 버튼 (우측 상단)
      // [v55] BUG FIX: p.y -> p_y
      else if (p_x >= 260 && p_x <= 315 && p_y >= 5 && p_y <= 35) {
        // MCU 1로 게인 계산 명령 전송
        sendCalibrationCommand('2', temp_true_v, temp_true_i);
        // calib_step = 3; // UI는 "Calculating..."으로 변경
        strcpy_P(calib_status_msg, PSTR("Calculating gains... Please wait..."));
        runAutoCalib(); // 상태 메시지 즉시 표시
        // MCU 1이 응답('S' 패킷)하면 loop() 리스너가 calib_step을 4로 변경
      }
      break;
    case 4: // "RESULTS" (Test.ino의 'VERIFY' 버튼을 'RESULTS'로 변경)
      // [v55] BUG FIX: p.y -> p_y
      if (p_x >= 80 && p_x <= 240 && p_y >= 130 && p_y <= 170) {
        calib_step = 5; // 최종 결과 화면으로
        screenNeedsRedraw = true;
      }
      break;
      
    case 5: // "RESTART" (결과 화면에서)
      // [v55] BUG FIX: p.x -> p_x, p.y -> p_y
      if (p_x >= 210 && p_x <= 310 && p_y >= 180 && p_y <= 220) { // DONE
        calib_step = 0; // 0단계부터 다시 시작
        currentScreen = SCREEN_SETTINGS; // 설정 메뉴로 복귀
        screenNeedsRedraw = true;
      }
      break;
  }
}

// ==============================================================================
// 4. 헬퍼 함수 (TFT 출력) (유지)
// ==============================================================================

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

void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) {
    return;
  }
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer); tft.print(buffer); tft.print(unit);
  tft.setTextColor(color); tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer); tft.print(buffer); tft.print(unit); 
}

// [v54] F() 매크로용 오버로드
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, const __FlashStringHelper* unit) {
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) {
    return;
  }
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer); tft.print(buffer); tft.print(unit);
  tft.setTextColor(color); tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer); tft.print(buffer); tft.print(unit); 
}

void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (value.equals(prev_value)) return;
  tft.setTextColor(COLOR_BACKGROUND); tft.setCursor(x, y); tft.print(prev_value);
  tft.setTextColor(color); tft.setCursor(x, y); tft.print(value);
}

// [v54] F() 매크로용 오버로드
void printTFTValue(int x, int y, const __FlashStringHelper* value, String prev_value, uint16_t color) {
  String valStr = String(value);
  if (valStr.equals(prev_value)) return;
  tft.setTextColor(COLOR_BACKGROUND); tft.setCursor(x, y); tft.print(prev_value);
  tft.setTextColor(color); tft.setCursor(x, y); tft.print(valStr);
}

// [v54] 보정 결과 표시용
void printTFTResult(int x, int y, const __FlashStringHelper* label, float value, int precision, uint16_t color) {
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(1);
  tft.setCursor(x, y);
  tft.print(label);
  
  char buffer[20];
  dtostrf(value, 4, precision, buffer);
  
  tft.setTextColor(color);
  tft.setCursor(x + 130, y); // 값 정렬
  tft.print(buffer);
}


void displayNetworkStatus() {
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.fillRect(240, 5, 80, 10, COLOR_BACKGROUND);
  tft.setCursor(240, 5);
  tft.print(wifi_status_msg);
}

void drawBackButton() {
  tft.fillRoundRect(5, 5, 50, 30, 8, COLOR_TEXT_SECONDARY);
  tft.setCursor(20, 12);
  tft.setTextColor(COLOR_BACKGROUND);
  tft.setTextSize(2);
  tft.print("<");
}

void drawMenuButton() {
  tft.fillRoundRect(265, 5, 50, 30, 8, COLOR_TEXT_SECONDARY);
  tft.setCursor(280, 12);
  tft.setTextColor(COLOR_BACKGROUND);
  tft.setTextSize(2);
  tft.print("...");
}

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

// [v54] F() 매크로용 오버로드
void drawButton(int x, int y, int w, int h, const __FlashStringHelper* text) {
  String textStr = String(text);
  tft.fillRoundRect(x, y, w, h, 8, COLOR_BUTTON);
  tft.drawRoundRect(x, y, w, h, 8, COLOR_BUTTON_OUTLINE);
  tft.setTextColor(COLOR_BUTTON_TEXT);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(textStr, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(x + (w - w1) / 2, y + (h - h1) / 2);
  tft.print(textStr);
}

// [v55] 버튼 색상 지정 (타이머 START/STOP용)
void drawButton(int x, int y, int w, int h, String text, uint16_t color) {
  tft.fillRoundRect(x, y, w, h, 8, color);
  tft.drawRoundRect(x, y, w, h, 8, COLOR_BUTTON_OUTLINE);
  tft.setTextColor(COLOR_BUTTON_TEXT);
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(x + (w - w1) / 2, y + (h - h1) / 2);
  tft.print(text);
}


void drawCheckbox(int x, int y, int w, int h, String text, bool isChecked, uint16_t color) {
  int boxSize = h - 10;
  int boxX = x + 5;
  int boxY = y + 5;
  tft.fillRect(x, y, w, h, COLOR_BACKGROUND);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(x + boxSize + 15, y + (h - 16) / 2); // 16 = text height 2
  tft.print(text);
  tft.drawRect(boxX, boxY, boxSize, boxSize, color);
  if (isChecked) {
    tft.setTextSize(2);
    tft.setCursor(boxX + 4, boxY + 1); 
    tft.print("X");
  }
}


// ==============================================================================
// 5. 메인 전력 화면 그리기 (유지)
// ==============================================================================
void displayMainScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  drawBackButton();
  drawMenuButton(); 
  tft.setTextSize(2);
  int col1_label_x = 10, col2_label_x = 170;
  int y_row1 = 40, y_row2 = 65, y_row3 = 90, y_row4 = 115, y_row5 = 140; 
  tft.setCursor(col1_label_x, y_row1); tft.setTextColor(COLOR_BLUE); tft.println("V:");
  tft.setCursor(col1_label_x, y_row2); tft.setTextColor(COLOR_ORANGE); tft.println("I-M:");
  tft.setCursor(col1_label_x, y_row3); tft.setTextColor(COLOR_DARKGREEN); tft.println("P:");
  tft.setCursor(col1_label_x, y_row4); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:");
  tft.setCursor(col1_label_x, y_row5); tft.setTextColor(COLOR_ORANGE); tft.println("Q:");
  tft.setCursor(col2_label_x, y_row1); tft.setTextColor(COLOR_RED); tft.println("I-1:");
  tft.setCursor(col2_label_x, y_row2); tft.setTextColor(COLOR_GREEN); tft.println("I-2:");
  tft.setCursor(col2_label_x, y_row3); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("S:");
  tft.setCursor(col2_label_x, y_row4); tft.setTextColor(COLOR_BLUE); tft.println("THD-V:");
  tft.setCursor(col2_label_x, y_row5); tft.setTextColor(COLOR_ORANGE); tft.println("THD-I:");
}

// ==============================================================================
// 6. 메인 전력 화면 "값" 업데이트 (유지)
// ==============================================================================
void displayMainScreenValues() {
  tft.setTextSize(2);
  char buffer[20];
  int col1_value_x = 60, col2_value_x = 220; 
  int col_w_half = 90, col_w_half_wide = 100;
  int y_row1 = 40, y_row2 = 65, y_row3 = 90, y_row4 = 115, y_row5 = 140;

  // --- Column 1 ---
  tft.fillRect(col1_value_x, y_row1, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE); tft.setCursor(col1_value_x, y_row1);
  dtostrf(V_rms, 4, 1, buffer); tft.print(buffer); tft.print(" V");

  tft.fillRect(col1_value_x, y_row2, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE); tft.setCursor(col1_value_x, y_row2);
  if (I_rms < 1.0) { dtostrf(I_rms * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA"); }
  else { dtostrf(I_rms, 4, 2, buffer); tft.print(buffer); tft.print(" A"); }

  tft.fillRect(col1_value_x, y_row3, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_DARKGREEN); tft.setCursor(col1_value_x, y_row3);
  if (P_real >= 1000.0) { dtostrf(P_real / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kW"); }
  else { dtostrf(P_real, 4, 1, buffer); tft.print(buffer); tft.print(" W"); }

  tft.fillRect(col1_value_x, y_row4, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_MAGENTA); tft.setCursor(col1_value_x, y_row4);
  dtostrf(PF, 4, 2, buffer); tft.print(buffer);

  tft.fillRect(col1_value_x, y_row5, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE); tft.setCursor(col1_value_x, y_row5);
  if (Q_reactive >= 1000.0) { dtostrf(Q_reactive / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVAR"); }
  else { dtostrf(Q_reactive, 4, 1, buffer); tft.print(buffer); tft.print(" VAR"); }

  // --- Column 2 ---
  tft.fillRect(col2_value_x, y_row1, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_RED); tft.setCursor(col2_value_x, y_row1); 
  if (I_rms_load1 < 1.0) { dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA"); }
  else { dtostrf(I_rms_load1, 4, 2, buffer); tft.print(buffer); tft.print(" A"); }

  tft.fillRect(col2_value_x, y_row2, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_GREEN); tft.setCursor(col2_value_x, y_row2); 
  if (I_rms_load2 < 1.0) { dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA"); }
  else { dtostrf(I_rms_load2, 4, 2, buffer); tft.print(buffer); tft.print(" A"); }

  tft.fillRect(col2_value_x, y_row3, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setCursor(col2_value_x, y_row3); 
  if (S_apparent >= 1000.0) { dtostrf(S_apparent / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVA"); }
  else { dtostrf(S_apparent, 4, 1, buffer); tft.print(buffer); tft.print(" VA"); }

  tft.fillRect(col2_value_x, y_row4, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE); tft.setCursor(col2_value_x, y_row4);
  dtostrf(thd_v * 100.0, 4, 1, buffer); tft.print(buffer); tft.print(" %");

  tft.fillRect(col2_value_x, y_row5, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE); tft.setCursor(col2_value_x, y_row5);
  dtostrf(thd_i * 100.0, 4, 1, buffer); tft.print(buffer); tft.print(" %");
}


// ==============================================================================
// 7. [v19] 위상차 화면 그리기 (유지)
// ==============================================================================
void displayPhaseScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("PHASOR DIAGRAM"); 
  drawBackButton(); drawMenuButton();
  
  tft.setTextSize(2);
  tft.setCursor(10, 50); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 75); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("Status:");
  tft.setTextSize(1);
  tft.setCursor(10, 110); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("--- Phase (deg) ---");
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
  tft.setCursor(PHASOR_CX + PHASOR_RADIUS + 3, PHASOR_CY - 3); tft.print("0");

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
  
  prev_phase_degrees = 0.0; prev_lead_lag_status = "---"; 
  prev_phase_main_deg = 0.0; prev_phase_load1_deg = 0.0; prev_phase_load2_deg = 0.0;
  prev_v_x = PHASOR_CX; prev_v_y = PHASOR_CY;
  prev_im_x = PHASOR_CX; prev_im_y = PHASOR_CY;
  prev_i1_x = PHASOR_CX; prev_i1_y = PHASOR_CY;
  prev_i2_x = PHASOR_CX; prev_i2_y = PHASOR_CY;
  showPhasorMenu = false; 
}

// ==============================================================================
// 8. [v19] 위상차 화면 "값" 업데이트 함수 (유지)
// ==============================================================================
void displayPhaseScreenValues() {
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

  float max_I_rms = max(I_rms, max(I_rms_load1, I_rms_load2));
  float I_scale_denominator = (max_I_rms < 0.05) ? 0.0 : max_I_rms;
  float v_len = PHASOR_RADIUS, im_len = 0.0, i1_len = 0.0, i2_len = 0.0;

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
  
  prev_phase_degrees = PF; prev_lead_lag_status = lead_lag_status; 
  prev_phase_main_deg = phase_main_deg; prev_phase_load1_deg = phase_load1_deg; prev_phase_load2_deg = phase_load2_deg;
  prev_v_x = v_x; prev_v_y = v_y;
  prev_im_x = im_x; prev_im_y = im_y;
  prev_i1_x = i1_x; prev_i1_y = i1_y;
  prev_i2_x = i2_x; prev_i2_y = i2_y;
}

// ==============================================================================
// 9. 60Hz 0점 통과(Zero-Crossing) 대기 함수 (v54 - 로컬 Midpoint 사용)
// ==============================================================================
void waitForVoltageZeroCross() {
  long startTime = micros();
  long timeout = 20000; 
  int V_ac_bits_prev = 0;
  int V_raw = 0;
  int V_ac_bits = 0; 
  
  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT_LOCAL; // [v54]
    if (V_ac_bits < -50) {
       V_ac_bits_prev = V_ac_bits;
       break;
    }
    if (micros() - startTime > timeout) return; 
  }

  while (true) {
    V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT_LOCAL; // [v54]
    if (V_ac_bits_prev < 0 && V_ac_bits >= 0) return;
    V_ac_bits_prev = V_ac_bits;
    if (micros() - startTime > timeout * 2) return; 
  }
}


// ==============================================================================
// 10. [v48] 공통 파형 화면 (정적 그리드 + 동적 라벨) (유지)
// ==============================================================================
void drawWaveformGridAndLabels() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.print("WAVEFORM (60Hz Sync)"); 
  drawBackButton(); drawMenuButton(); 
  
  int legendX = 200, legendY_start = 28, legendY_step = 12;
  int currentLegendY = legendY_start;
  tft.setTextSize(1);
  tft.fillRect(legendX, legendY_start - 5, 115, 6 * legendY_step, COLOR_BACKGROUND); 

  if (show_V) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_BLUE); 
    tft.setTextColor(COLOR_BLUE); tft.setCursor(legendX + 12, currentLegendY); tft.print("V (Volt)");
    currentLegendY += legendY_step;
  }
  if (show_I) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_ORANGE); 
    tft.setTextColor(COLOR_ORANGE); tft.setCursor(legendX + 12, currentLegendY); tft.print("I-M (Amp)");
    currentLegendY += legendY_step;
  }
  if (show_I1) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_RED); 
    tft.setTextColor(COLOR_RED); tft.setCursor(legendX + 12, currentLegendY); tft.print("I-1 (Amp)");
    currentLegendY += legendY_step;
  }
  if (show_I2) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_GREEN); 
    tft.setTextColor(COLOR_GREEN); tft.setCursor(legendX + 12, currentLegendY); tft.print("I-2 (Amp)");
    currentLegendY += legendY_step;
  }
  if (show_P) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_DARKGREEN); 
    tft.setTextColor(COLOR_DARKGREEN); tft.setCursor(legendX + 12, currentLegendY); tft.print("P (Watt)");
    currentLegendY += legendY_step;
  }
  if (show_Q) {
    tft.drawFastHLine(legendX, currentLegendY + 3, 10, COLOR_MAGENTA); 
    tft.setTextColor(COLOR_MAGENTA); tft.setCursor(legendX + 12, currentLegendY); tft.print("Q (VAR)");
    currentLegendY += legendY_step;
  }

  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), COLOR_GRID);
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY);
  tft.drawFastVLine(PLOT_X_END, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY);

  tft.setCursor(10, SCREEN_HEIGHT - 12);
  tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.print("Sampling Period: "); tft.print(WAVEFORM_SAMPLE_PERIOD_US); tft.print(" us");
  
  for (int i = 0; i < V_DELAY_SAMPLES; i++) v_delay_buffer[i] = 0.0;
  v_delay_index = 0;
  showWaveformMenu = false; 
}

// ==============================================================================
// 11. [v48] 동적 Y축 라벨 업데이트 함수 (유지)
// ==============================================================================
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START, (PLOT_Y_END - PLOT_Y_START), COLOR_BACKGROUND);
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, (PLOT_Y_END - PLOT_Y_START), COLOR_BACKGROUND);

  bool is_I_active = show_I || show_I1 || show_I2, is_V_active = show_V;
  bool is_P_active = show_P, is_Q_active = show_Q;

  // --- 왼쪽 축 (I 또는 P) ---
  if (is_I_active) {
    tft.setTextColor(COLOR_ORANGE);
    if (I_axis_max < 1.0) { dtostrf(I_axis_max * 1000, 3, 0, buffer); }
    else { dtostrf(I_axis_max, 3, 1, buffer); }
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_CENTER - 4); tft.print("0A"); 
  }
  else if (is_P_active) {
    tft.setTextColor(COLOR_DARKGREEN);
    if (P_axis_max >= 1000.0) { dtostrf(P_axis_max / 1000.0, 3, 1, buffer); tft.print("k"); }
    else { dtostrf(P_axis_max, 3, 0, buffer); }
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_CENTER - 4); tft.print("0W"); 
  }

  // --- 오른쪽 축 (V 또는 Q) ---
  if (is_V_active) {
    tft.setTextColor(COLOR_BLUE);
    dtostrf(V_axis_max, 3, 0, buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4); tft.print("0V"); 
  }
  else if (is_Q_active) {
    tft.setTextColor(COLOR_MAGENTA);
    if (Q_axis_max >= 1000.0) { dtostrf(Q_axis_max / 1000.0, 3, 1, buffer); tft.print("k"); }
    else { dtostrf(Q_axis_max, 3, 0, buffer); }
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4); tft.print("0VAR");
  }
}


// ==============================================================================
// 12. 경고 팝업 화면 (유지)
// ==============================================================================
void displayWarningScreenStatic() {
  tft.fillScreen(ILI9341_RED);
  tft.drawRect(5, 5, SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, ILI9341_WHITE);
  tft.drawRect(6, 6, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 12, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3); 
  tft.setCursor(75, 40); tft.print("WARNING!"); 
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2); 
  int16_t x1, y1; uint16_t w1, h1;
  tft.getTextBounds(warningMessage, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 85); tft.print(warningMessage);
  
  tft.setTextSize(3); char buffer[20];
  if (warningMessage.startsWith("OVER VOLTAGE")) { dtostrf(V_rms, 4, 1, buffer); strcat(buffer, " V"); }
  else { dtostrf(I_rms, 4, 2, buffer); strcat(buffer, " A"); }
  tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 125); tft.print(buffer);
  
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2); 
  String resetMsg = "Tap screen to reset";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); tft.print(resetMsg); 
}


// ==============================================================================
// 16. [v54] 파형 그리기 루프 (로컬 Midpoint 사용)
// ==============================================================================
void runCombinedWaveformLoop() {
  bool is_I_active = show_I || show_I1 || show_I2, is_V_active = show_V;
  bool is_P_active = show_P, is_Q_active = show_Q;
  if (is_I_active && is_P_active) is_P_active = false; 
  if (is_V_active && is_Q_active) is_Q_active = false; 

  float volts_to_pixels_scale = (V_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_axis_max);
  float amps_to_pixels_scale = (I_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / I_axis_max);
  float watts_to_pixels_scale = (P_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / P_axis_max);
  float vars_to_pixels_scale = (Q_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / Q_axis_max);

  float new_frame_V_peak = 0.0, new_frame_I_peak = 0.0, new_frame_P_peak = 0.0, new_frame_Q_peak = 0.0;
  int new_y_v[PLOT_WIDTH], new_y_i[PLOT_WIDTH], new_y_i1[PLOT_WIDTH], new_y_i2[PLOT_WIDTH], new_y_p[PLOT_WIDTH], new_y_q[PLOT_WIDTH];

  unsigned long startTime = micros();

  // [v54] 로컬 파형 표시는 MCU 1과 동일한 계산법 사용
  float effective_V_Calib = mcu_v_gain * V_MULTIPLIER;
  float effective_I_Calib = mcu_i_gain * I_MULTIPLIER;
  float effective_I1_Calib = mcu_i1_gain * I_MULTIPLIER;
  float effective_I2_Calib = mcu_i2_gain * I_MULTIPLIER;
  float effective_V_Offset = mcu_v_offset_adjust * V_MULTIPLIER;
  float effective_I_Offset = mcu_i_offset_adjust * I_MULTIPLIER;
  float effective_I1_Offset = mcu_i1_offset_adjust * I_MULTIPLIER;
  float effective_I2_Offset = mcu_i2_offset_adjust * I_MULTIPLIER;

  for (int i = 0; i < PLOT_WIDTH; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int I1_raw = analogRead(CURRENT_PIN_LOAD1);
    int I2_raw = analogRead(CURRENT_PIN_LOAD2);
    
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT_LOCAL; // [v54]
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT_LOCAL; // [v54]
    int I1_ac_bits = I1_raw - (int)I1_ADC_MIDPOINT_LOCAL; // [v54]
    int I2_ac_bits = I2_raw - (int)I2_ADC_MIDPOINT_LOCAL; // [v54]
    
    // [v54] 로컬 파형 계산식 수정 (MCU 1과 동일하게)
    float V_mains_instant = V_ac_bits * effective_V_Calib - effective_V_Offset;
    float I_mains_instant = I_ac_bits * effective_I_Calib - effective_I_Offset;
    float I1_mains_instant = I1_ac_bits * effective_I1_Calib - effective_I1_Offset;
    float I2_mains_instant = I2_ac_bits * effective_I2_Calib - effective_I2_Offset;

    float V_delayed = v_delay_buffer[v_delay_index]; 
    v_delay_buffer[v_delay_index] = V_mains_instant; 
    float P_instant = V_mains_instant * I_mains_instant;
    float Q_instant = V_delayed * I_mains_instant; 
    v_delay_index = (v_delay_index + 1) % V_DELAY_SAMPLES; 
    
    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    float current_I_peak = 0.0;
    if (show_I) current_I_peak = abs(I_mains_instant);
    if (show_I1 && abs(I1_mains_instant) > current_I_peak) current_I_peak = abs(I1_mains_instant);
    if (show_I2 && abs(I2_mains_instant) > current_I_peak) current_I_peak = abs(I2_mains_instant);
    if (current_I_peak > new_frame_I_peak) new_frame_I_peak = current_I_peak;
    if (abs(P_instant) > new_frame_P_peak) new_frame_P_peak = abs(P_instant);
    if (abs(Q_instant) > new_frame_Q_peak) new_frame_Q_peak = abs(Q_instant);

    new_y_v[i] = constrain(PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    new_y_i[i] = constrain(PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    new_y_i1[i] = constrain(PLOT_Y_CENTER - (int)(I1_mains_instant * amps_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    new_y_i2[i] = constrain(PLOT_Y_CENTER - (int)(I2_mains_instant * amps_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    new_y_p[i] = constrain(PLOT_Y_CENTER - (int)(P_instant * watts_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    new_y_q[i] = constrain(PLOT_Y_CENTER - (int)(Q_instant * vars_to_pixels_scale), PLOT_Y_START, PLOT_Y_END);
    
    while(micros() - startTime < (i + 1) * WAVEFORM_SAMPLE_PERIOD_US);
  }

  tft.startWrite(); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  for (int i = 1; i < PLOT_WIDTH; i++) {
      int x_curr = PLOT_X_START + i, x_prev = PLOT_X_START + i - 1;
      if (show_I && (last_frame_y_i[i] != PLOT_Y_CENTER || last_frame_y_i[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_i[i-1], x_curr, last_frame_y_i[i], COLOR_BACKGROUND);
      if (show_I1 && (last_frame_y_i1[i] != PLOT_Y_CENTER || last_frame_y_i1[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_i1[i-1], x_curr, last_frame_y_i1[i], COLOR_BACKGROUND);
      if (show_I2 && (last_frame_y_i2[i] != PLOT_Y_CENTER || last_frame_y_i2[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_i2[i-1], x_curr, last_frame_y_i2[i], COLOR_BACKGROUND);
      if (show_V && (last_frame_y_v[i] != PLOT_Y_CENTER || last_frame_y_v[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_v[i-1], x_curr, last_frame_y_v[i], COLOR_BACKGROUND);
      if (show_P && (last_frame_y_p[i] != PLOT_Y_CENTER || last_frame_y_p[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_p[i-1], x_curr, last_frame_y_p[i], COLOR_BACKGROUND);
      if (show_Q && (last_frame_y_q[i] != PLOT_Y_CENTER || last_frame_y_q[i-1] != PLOT_Y_CENTER)) tft.drawLine(x_prev, last_frame_y_q[i-1], x_curr, last_frame_y_q[i], COLOR_BACKGROUND);
      if (show_I) tft.drawLine(x_prev, new_y_i[i-1], x_curr, new_y_i[i], COLOR_ORANGE);
      if (show_I1) tft.drawLine(x_prev, new_y_i1[i-1], x_curr, new_y_i1[i], COLOR_RED);
      if (show_I2) tft.drawLine(x_prev, new_y_i2[i-1], x_curr, new_y_i2[i], COLOR_GREEN);
      if (show_V) tft.drawLine(x_prev, new_y_v[i-1], x_curr, new_y_v[i], COLOR_BLUE);
      if (show_P) tft.drawLine(x_prev, new_y_p[i-1], x_curr, new_y_p[i], COLOR_DARKGREEN);
      if (show_Q) tft.drawLine(x_prev, new_y_q[i-1], x_curr, new_y_q[i], COLOR_MAGENTA);
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
      last_frame_y_v[i] = show_V ? new_y_v[i] : PLOT_Y_CENTER;
      last_frame_y_i[i] = show_I ? new_y_i[i] : PLOT_Y_CENTER;
      last_frame_y_i1[i] = show_I1 ? new_y_i1[i] : PLOT_Y_CENTER;
      last_frame_y_i2[i] = show_I2 ? new_y_i2[i] : PLOT_Y_CENTER;
      last_frame_y_p[i] = show_P ? new_y_p[i] : PLOT_Y_CENTER;
      last_frame_y_q[i] = show_Q ? new_y_q[i] : PLOT_Y_CENTER;
  }

  float new_V_axis = V_axis_max, new_I_axis = I_axis_max, new_P_axis = P_axis_max, new_Q_axis = Q_axis_max;
  if (is_I_active) new_I_axis = findAxisStep(new_frame_I_peak, I_AXIS_STEPS, NUM_I_STEPS);
  if (is_V_active) new_V_axis = findAxisStep(new_frame_V_peak, V_AXIS_STEPS, NUM_V_STEPS);
  if (is_P_active) new_P_axis = findAxisStep(new_frame_P_peak, PQ_AXIS_STEPS, NUM_PQ_STEPS);
  if (is_Q_active) new_Q_axis = findAxisStep(new_frame_Q_peak, PQ_AXIS_STEPS, NUM_PQ_STEPS);
  
  if (new_V_axis != V_axis_max || new_I_axis != I_axis_max || new_P_axis != P_axis_max || new_Q_axis != Q_axis_max) {
    V_axis_max = new_V_axis; I_axis_max = new_I_axis; P_axis_max = new_P_axis; Q_axis_max = new_Q_axis;
    updateYAxisLabels(); 
  }
}

// ==============================================================================
// 19. [v52] 퍼지 출력 -> *경고* 변환 함수 (유지)
// ==============================================================================
void controlRelays(float level) {
  if (level > 9.0) {
    if (!warningActive) {
      warningMessage = "FUZZY LOGIC TRIP";
      warningActive = true; 
      screenNeedsRedraw = true; 
    }
  } 
}

// ==============================================================================
// 21. [v49] Harmonics 화면 (그래프) 정적 UI 그리기 (유지)
// ==============================================================================
void displayHarmonicsGraphStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("HARMONICS (GRAPH)"); 
  drawBackButton(); drawMenuButton(); 
  
  int graphX = 30, graphY = 60, graphW = 280, graphH = 150;
  tft.drawRect(graphX, graphY, graphW, graphH, COLOR_GRID);
  
  tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(graphX - 25, graphY); tft.print("100%"); 
  tft.setCursor(graphX - 20, graphY + graphH - 10); tft.print("0%");
  tft.setCursor(graphX, graphY + graphH + 5); tft.print("2");
  tft.setCursor(graphX + graphW/2 - 5, graphY + graphH + 5); tft.print("11");
  tft.setCursor(graphX + graphW - 10, graphY + graphH + 5); tft.print("21");
  tft.setCursor(graphX + graphW/2 - 30, graphY + graphH + 15); tft.print("(Harmonic #)");
  
  tft.setTextSize(2);
  String mode_text = showVoltageHarmonics ? "VOLTAGE" : "CURRENT";
  uint16_t mode_color = showVoltageHarmonics ? COLOR_BLUE : COLOR_ORANGE;
  tft.setCursor(10, 220); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Show:");
  tft.setCursor(70, 220); tft.setTextColor(mode_color); tft.print(mode_text);
  tft.setCursor(170, 220); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("THD:");
  
  tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(65, 35); tft.print("Tap graph to view list");
  
  prev_thd_v = -1.0; prev_thd_i = -1.0; 
  for (int i=0; i<NUM_HARMONICS_TO_RECEIVE; i++) prev_bar_heights[i] = 0;
  showHarmonicsMenu = false; 
}

// ==============================================================================
// 22. [v49] Harmonics 화면 (그래프) 동적 값 업데이트 (유지)
// ==============================================================================
void displayHarmonicsGraphValues() {
  float* harmonics_to_draw = showVoltageHarmonics ? v_harmonics : i_harmonics;
  float* prev_harmonics_to_draw = showVoltageHarmonics ? prev_v_harmonics : prev_i_harmonics;
  float thd_to_show = showVoltageHarmonics ? thd_v : thd_i;
  float prev_thd_to_show = showVoltageHarmonics ? prev_thd_v : prev_thd_i;
  uint16_t bar_color = showVoltageHarmonics ? COLOR_BLUE : COLOR_ORANGE;

  tft.setTextSize(2);
  printTFTValue(220, 220, thd_to_show * 100.0, prev_thd_to_show * 100.0, 1, bar_color, "%");
  if (showVoltageHarmonics) prev_thd_v = thd_v;
  else prev_thd_i = thd_i;

  int graphX = 30, graphY = 60, graphW = 280, graphH = 150;
  int bar_width_full = (graphW / NUM_HARMONICS_TO_RECEIVE); // 14
  int bar_width = max(1, bar_width_full - 2); // 12
  
  float max_percent = 0.1; 
  for (int i = 0; i < NUM_HARMONICS_TO_RECEIVE; i++) {
    if (harmonics_to_draw[i] > max_percent) max_percent = harmonics_to_draw[i];
  }
  max_percent = min(1.0, max_percent * 1.2); 
  
  tft.setTextSize(1);
  tft.fillRect(graphX - 28, graphY, 28, 10, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(graphX - 25, graphY); 
  char buffer[10];
  dtostrf(max_percent * 100.0, 3, 0, buffer);
  tft.print(buffer); tft.print("%");

  tft.startWrite();
  for (int i = 0; i < NUM_HARMONICS_TO_RECEIVE; i++) {
    int x_pos = graphX + (i * bar_width_full) + (bar_width_full - bar_width)/2;
    int new_height = (int)( (harmonics_to_draw[i] / max_percent) * (float)graphH );
    new_height = constrain(new_height, 0, graphH);
    int y_pos = graphY + graphH - new_height;
    int prev_height = prev_bar_heights[i];
    int prev_y_pos = graphY + graphH - prev_height;
    
    if (new_height != prev_height) {
      if (new_height > prev_height) tft.fillRect(x_pos, y_pos, bar_width, (prev_y_pos - y_pos), bar_color);
      else tft.fillRect(x_pos, prev_y_pos, bar_width, (y_pos - prev_y_pos), COLOR_BACKGROUND);
      prev_bar_heights[i] = new_height;
    }
  }
  tft.endWrite();
}

// ==============================================================================
// 23. [v49] Harmonics 화면 (리스트) 정적 UI 그리기 (유지)
// ==============================================================================
void displayHarmonicsListStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("HARMONICS (LIST)"); 
  drawBackButton(); drawMenuButton();
  
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setCursor(10, 45); tft.print("H#");
  tft.setCursor(80, 45); tft.setTextColor(COLOR_BLUE); tft.print("V (%)");
  tft.setCursor(170, 45); tft.setTextColor(COLOR_ORANGE); tft.print("I (%)");
  tft.drawFastHLine(10, 65, 240, COLOR_GRID);

  drawButton(260, 70, 50, 40, "UP");
  drawButton(260, 130, 50, 40, "DOWN");
  
  tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(265, 180);
  tft.print("Page "); tft.print(list_scroll_offset + 1); tft.print("/"); tft.print(num_harmonic_pages);
  tft.setCursor(65, 35); tft.print("Tap list to view graph");
  
  prev_list_scroll_offset = -1; 
  for(int i=0; i<NUM_HARMONICS_TO_RECEIVE; i++) {
    prev_v_harmonics[i] = -1.0; prev_i_harmonics[i] = -1.0;
  }
}

// ==============================================================================
// 24. [v49] Harmonics 화면 (리스트) 동적 값 업데이트 (유지)
// ==============================================================================
void displayHarmonicsListValues() {
  int y_start = 75, y_step = 16; 
  if (prev_list_scroll_offset != list_scroll_offset) {
     tft.fillRect(0, y_start, 250, 160, COLOR_BACKGROUND); 
     for(int i=0; i<NUM_HARMONICS_TO_RECEIVE; i++) {
        prev_v_harmonics[i] = -1.0; prev_i_harmonics[i] = -1.0;
     }
     prev_list_scroll_offset = list_scroll_offset;
     
     tft.fillRect(265, 180, 50, 10, COLOR_BACKGROUND);
     tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
     tft.setCursor(265, 180);
     tft.print("Page "); tft.print(list_scroll_offset + 1); tft.print("/"); tft.print(num_harmonic_pages);
  }

  int start_index = list_scroll_offset * harmonics_per_page;
  tft.setTextSize(2);
  for (int i = 0; i < harmonics_per_page; i++) {
    int harmonic_index = start_index + i;
    if (harmonic_index >= NUM_HARMONICS_TO_RECEIVE) break;
    int y_pos = y_start + (i * y_step);
    int harmonic_num = harmonic_index + 2; 
    
    tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setCursor(10, y_pos); tft.print(harmonic_num);
    printTFTValue(80, y_pos, v_harmonics[harmonic_index] * 100.0, prev_v_harmonics[harmonic_index] * 100.0, 1, COLOR_BLUE, "");
    prev_v_harmonics[harmonic_index] = v_harmonics[harmonic_index];
    printTFTValue(170, y_pos, i_harmonics[harmonic_index] * 100.0, prev_i_harmonics[harmonic_index] * 100.0, 1, COLOR_ORANGE, "");
    prev_i_harmonics[harmonic_index] = i_harmonics[harmonic_index];
  }
}


// ==============================================================================
// 25. [v20] 홈 화면 그리기 (v49)
// ==============================================================================
void displayHomeScreenStatic() {
  tft.setCursor(50, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(3);
  tft.print("KW WATT-METER");
  drawButton(20, 50, 130, 40, "MAIN POWER");
  drawButton(170, 50, 130, 40, "PHASOR");
  drawButton(20, 110, 130, 40, "WAVEFORM");
  drawButton(170, 110, 130, 40, "HARMONICS"); 
  drawButton(20, 170, 130, 40, "SETTINGS");
  drawButton(170, 170, 130, 40, "RELAY CTRL");
}

// ==============================================================================
// 26. [v55] 설정 메인 화면 그리기 (Timer 버튼 추가 및 레이아웃 조정)
// ==============================================================================
void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("SETTINGS");
  displayNetworkStatus();
  drawBackButton();
  
  // [v55] 버튼 레이아웃 조정
  drawButton(20, 50, 280, 30, "Manual Multipliers"); 
  drawButton(20, 85, 280, 30, "PROTECTION");
  drawButton(20, 120, 280, 30, "Auto Calibration"); 
  drawButton(20, 155, 280, 30, "TIMER"); // [v55] NEW
  
  drawButton(20, 190, 130, 40, "THEME");  
  drawButton(170, 190, 130, 40, "RESET"); 
}

// ==============================================================================
// 27. [v20] 설정 - 수동 보정 화면 그리기 (유지)
// ==============================================================================
void displaySettingsCalibStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("MANUAL MULTIPLIERS"); // [v54] Renamed
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
  prev_V_MULTIPLIER = -1.0; prev_I_MULTIPLIER = -1.0;
  prev_setting_step_index = -1;
}

void runSettingsCalib() {
  int y_positions[] = {50, 85, 120};
  if (prev_calib_selection != calib_selection) {
    if (prev_calib_selection != -1) tft.fillRect(10, y_positions[prev_calib_selection], 15, 18, COLOR_BACKGROUND); 
    tft.setTextColor(COLOR_RED); tft.setTextSize(2);
    tft.setCursor(10, y_positions[calib_selection]); tft.print(">");
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
// 28. [v20] 설정 - 보호 화면 그리기 (유지)
// ==============================================================================
void displaySettingsProtectStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
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
  int y_positions[] = {70, 110};
  if (prev_protect_selection != protect_selection) {
    if (prev_protect_selection != -1) tft.fillRect(10, y_positions[prev_protect_selection], 15, 18, COLOR_BACKGROUND); 
    tft.setTextColor(COLOR_RED); tft.setTextSize(2);
    tft.setCursor(10, y_positions[protect_selection]); tft.print(">");
    prev_protect_selection = protect_selection;
  }
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 
  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  prev_setting_step_index = setting_step_index;
}

// ==============================================================================
// 29. [v22] 릴레이 제어 화면 그리기 (v52 - 상태 수신)
// ==============================================================================
void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton(); drawMenuButton(); 

  String r1_status = relay1_state_from_mcu1 ? "ON" : "OFF (TRIPPED)";
  String r2_status = relay2_state_from_mcu1 ? "ON" : "OFF (TRIPPED)";
  drawButton(20, 70, 280, 40, "Relay 1: " + r1_status);
  drawButton(20, 130, 280, 40, "Relay 2: " + r2_status);
}

void runRelayControl() {
  // 터치 시에만 반응
}

// ==============================================================================
// 30. [v23] 설정 값 변경 헬퍼 함수 (유지)
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
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) setting_step_index = new_index;
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
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) setting_step_index = new_index;
      break;
  }
}

// ==============================================================================
// 31. [v24] 파형 Y축 범위 계산 헬퍼 (유지)
// ==============================================================================
float findAxisStep(float peak, const float* steps, int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    if (peak <= steps[i]) return steps[i];
  }
  return steps[num_steps - 1];
}

// ==============================================================================
// 32. [v25] 신규 화면 그리기 (유지)
// ==============================================================================
void displaySettingsThemeStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("THEME SETTINGS");
  drawBackButton();
  drawButton(20, 70, 280, 40, "LIGHT MODE");
  drawButton(20, 130, 280, 40, "DARK MODE");
  tft.setTextColor(COLOR_RED); tft.setTextSize(2);
  if (isDarkMode) tft.setCursor(5, 140);
  else tft.setCursor(5, 80);
  tft.print(">");
}
void runSettingsTheme() { }

void displaySettingsResetStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("RESET SETTINGS");
  drawBackButton();
  tft.setCursor(20, 60); tft.print("Reset all values to");
  tft.setCursor(20, 80); tft.print("factory defaults?");
  drawButton(20, 100, 130, 40, "RESET");
  drawButton(170, 100, 130, 40, "CANCEL");
}

void displayConfirmSaveStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("SAVE CHANGES?");
  drawBackButton();
  tft.setCursor(20, 70); tft.print("Save the new settings?");
  drawButton(20, 100, 130, 40, "SAVE");
  drawButton(170, 100, 130, 40, "DISCARD");
}

void restoreDefaultSettings() {
  VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
  V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
  I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
  setting_step_index = DEFAULT_SETTING_STEP_INDEX;
  
  // [v54] 전체 보정 계수도 기본값으로 리셋
  mcu_v_midpoint = DEFAULT_V_ADC_MIDPOINT;
  mcu_i_midpoint = DEFAULT_I_ADC_MIDPOINT;
  mcu_i1_midpoint = DEFAULT_I1_ADC_MIDPOINT;
  mcu_i2_midpoint = DEFAULT_I2_ADC_MIDPOINT;
  mcu_v_gain = BASE_V_CALIB_RMS;
  mcu_i_gain = BASE_I_CALIB_RMS;
  mcu_i1_gain = BASE_I_CALIB_RMS;
  mcu_i2_gain = BASE_I_CALIB_RMS;
  mcu_v_offset_adjust = BASE_V_OFFSET_ADJUST;
  mcu_i_offset_adjust = BASE_I_OFFSET_ADJUST;
  mcu_i1_offset_adjust = BASE_I_OFFSET_ADJUST;
  mcu_i2_offset_adjust = BASE_I_OFFSET_ADJUST;
}

// ==============================================================================
// 33. [v38] WiFi/MQTT 헬퍼 함수들 (유지)
// ==============================================================================
void setup_wifi() {
  espSerial.begin(9600); 
  Serial.println("Initializing ESP-01...");
  strcpy(wifi_status_msg, "ESP...");
  WiFi.init(&espSerial);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("ESP-01 module not found.");
    strcpy(wifi_status_msg, "ESP_ERR");
    screenNeedsRedraw = true; return;
  }
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  strcpy(wifi_status_msg, "CONN..."); screenNeedsRedraw = true;
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    WiFi.begin(WIFI_SSID, WIFI_PASS); delay(500); Serial.print("."); retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected."); Serial.print("IP: "); Serial.println(WiFi.localIP());
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    strcpy(wifi_status_msg, "MQTT...");
  } else {
    Serial.println("\nWiFi FAILED"); strcpy(wifi_status_msg, "WIFI_ERR");
  }
  screenNeedsRedraw = true;
}

void reconnect_mqtt() {
  if (WiFi.status() == WL_NO_SHIELD) {
     strcpy(wifi_status_msg, "ESP_ERR"); screenNeedsRedraw = true; setup_wifi(); return;
  }
  if (WiFi.status() != WL_CONNECTED) {
     strcpy(wifi_status_msg, "WIFI_ERR"); screenNeedsRedraw = true; setup_wifi(); return;
  }
  if (mqttClient.connected()) return;
  Serial.print("MQTT connection..."); strcpy(wifi_status_msg, "MQTT..."); screenNeedsRedraw = true;
  String clientId = "wattmeter-"; clientId += String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected"); strcpy(wifi_status_msg, "NET: OK");
    lastMqttPublish = millis();
  } else {
    Serial.print("failed, rc="); Serial.print(mqttClient.state()); Serial.println(" try again");
    strcpy(wifi_status_msg, "MQTT_ERR");
  }
  screenNeedsRedraw = true;
}

void publishData() {
  if (!mqttClient.connected()) { reconnect_mqtt(); return; }
  char payload[300];
  snprintf(payload, 300, 
    "{\"V_rms\":%.2f, \"I_rms\":%.3f, \"I_1\":%.3f, \"I_2\":%.3f, \"P_real\":%.2f, \"S_app\":%.2f, \"Q_react\":%.2f, \"PF\":%.2f, \"THD_V\":%.2f, \"THD_I\":%.2f}",
    V_rms, I_rms, I_rms_load1, I_rms_load2, P_real, S_apparent, Q_reactive, PF, (thd_v * 100.0), (thd_i * 100.0)
  );
  if (mqttClient.publish(MQTT_TOPIC, payload)) { Serial.println("Publish OK"); }
  else { Serial.println("Publish FAILED"); }
  lastMqttPublish = millis();
}


// ==============================================================================
// 34. [v49] 화면별 메뉴 그리기 함수 (유지)
// ==============================================================================
void drawMainMenu() {
  int menuX = 60, menuY = 50, menuW = 200, menuH = 140;
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, isDarkMode ? 0x2104 : 0xDEFB);
  tft.fillRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_BACKGROUND);
  tft.drawRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_TEXT_PRIMARY);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.setCursor(menuX + 10, menuY + 10); tft.print("Main Menu");
  tft.drawFastHLine(menuX + 10, menuY + 35, menuW - 20, COLOR_GRID);
  tft.setTextSize(1);
  tft.setCursor(menuX + 10, menuY + 50); tft.print("Placeholder for Main functions.");
  tft.setCursor(menuX + 10, menuY + menuH - 20); tft.print("Tap '...' to close.");
}
void drawPhasorMenu() {
  int menuX = 60, menuY = 50, menuW = 200, menuH = 140;
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, isDarkMode ? 0x2104 : 0xDEFB);
  tft.fillRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_BACKGROUND);
  tft.drawRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_TEXT_PRIMARY);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.setCursor(menuX + 10, menuY + 10); tft.print("Phasor Menu");
  tft.drawFastHLine(menuX + 10, menuY + 35, menuW - 20, COLOR_GRID);
  tft.setTextSize(1);
  tft.setCursor(menuX + 10, menuY + 50); tft.print("Placeholder for Phasor functions.");
  tft.setCursor(menuX + 10, menuY + menuH - 20); tft.print("Tap '...' to close.");
}
void drawWaveformMenu() {
  int menuX = 40, menuY = 40, menuW = 240;
  int item_h = 30, y_margin = 5, num_items = 6;
  int menuH = (item_h + y_margin) * num_items + 10; 
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, isDarkMode ? 0x2104 : 0xDEFB);
  tft.fillRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_BACKGROUND);
  tft.drawRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_TEXT_PRIMARY);
  int y_curr = menuY + 5;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "V (Volt)", show_V, COLOR_BLUE); y_curr += item_h + y_margin;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "I-M (Amp)", show_I, COLOR_ORANGE); y_curr += item_h + y_margin;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "I-1 (Amp)", show_I1, COLOR_RED); y_curr += item_h + y_margin;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "I-2 (Amp)", show_I2, COLOR_GREEN); y_curr += item_h + y_margin;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "P (Watt)", show_P, COLOR_DARKGREEN); y_curr += item_h + y_margin;
  drawCheckbox(menuX + 5, y_curr, menuW - 10, item_h, "Q (VAR)", show_Q, COLOR_MAGENTA);
}
void drawHarmonicsMenu() {
  int menuX = 60, menuY = 50, menuW = 200, menuH = 150;
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, isDarkMode ? 0x2104 : 0xDEFB);
  tft.fillRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_BACKGROUND);
  tft.drawRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_TEXT_PRIMARY);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.setCursor(menuX + 10, menuY + 10); tft.print("Harmonics Menu");
  tft.drawFastHLine(menuX + 10, menuY + 35, menuW - 20, COLOR_GRID);
  String view_text = showHarmonicsAsList ? "View: LIST" : "View: GRAPH";
  drawButton(menuX + 10, menuY + 50, menuW - 20, 35, view_text);
  if (!showHarmonicsAsList) {
    String show_text = showVoltageHarmonics ? "Show: VOLTAGE" : "Show: CURRENT";
    drawButton(menuX + 10, menuY + 100, menuW - 20, 35, show_text);
  }
}
void drawRelayMenu() {
  int menuX = 60, menuY = 50, menuW = 200, menuH = 140;
  tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, isDarkMode ? 0x2104 : 0xDEFB);
  tft.fillRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_BACKGROUND);
  tft.drawRoundRect(menuX, menuY, menuW, menuH, 8, COLOR_TEXT_PRIMARY);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.setCursor(menuX + 10, menuY + 10); tft.print("Relay Menu");
  tft.drawFastHLine(menuX + 10, menuY + 35, menuW - 20, COLOR_GRID);
  tft.setTextSize(1);
  tft.setCursor(menuX + 10, menuY + 50); tft.print("Placeholder for Relay functions.");
  tft.setCursor(menuX + 10, menuY + menuH - 20); tft.print("Tap '...' to close.");
}


// ==============================================================================
// 35. [v54] 자동 보정 화면 (Test.ino에서 이식)
// ==============================================================================

// [v54] 자동 보정 화면 (정적)
void displayAutoCalibStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println(F("AUTO CALIBRATION")); 
  drawBackButton();

  tft.setTextSize(2);
  
  switch(calib_step) {
    case 0: // Step 0: 시작
      tft.setCursor(20, 80); tft.print(F("Disconnect all 220V power")); 
      tft.setCursor(20, 100); tft.print(F("and any loads.")); 
      drawButton(80, 120, 160, 40, F("START")); 
      break;
      
    case 1: // Step 1: 오프셋 측정 중
      tft.setCursor(20, 80); tft.print(F("Measuring offsets...")); 
      tft.setCursor(20, 100); tft.print(F("Please wait (1 sec)...")); 
      strcpy_P(calib_status_msg, PSTR("MCU1 is measuring..."));
      break;
      
    case 2: // Step 2: 전원 연결 요청
      tft.setCursor(20, 60); tft.print(F("Offsets measured.")); 
      tft.setCursor(20, 80); tft.print(F("Connect 220V power and")); 
      tft.setCursor(20, 100); tft.print(F("a known MAIN load (I-M).")); 
      drawButton(80, 130, 160, 40, F("NEXT")); 
      strcpy_P(calib_status_msg, PSTR("Offsets received from MCU1."));
      break;
      
    case 3: // Step 3: 실효값 입력
      tft.setCursor(30, 50); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print(F("True V (RMS):")); 
      tft.setCursor(30, 85); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print(F("True I (RMS):")); 
      tft.setCursor(30, 120); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print(F("Step:")); 
      drawButton(20, 180, 60, 40, F("UP")); 
      drawButton(90, 180, 60, 40, F("DOWN")); 
      drawButton(180, 180, 60, 40, F("-")); 
      drawButton(250, 180, 60, 40, F("+")); 
      tft.fillRoundRect(260, 5, 55, 30, 8, COLOR_GREEN);
      tft.setCursor(268, 12); tft.setTextColor(COLOR_BACKGROUND); tft.setTextSize(2);
      tft.print(F("CALC")); // Calculate
      strcpy_P(calib_status_msg, PSTR("Enter known True V/I values."));
      break;
      
    case 4: // Step 4: 게인 계산 완료, 결과 확인 요청
      tft.setCursor(20, 60); tft.print(F("Gains calculated.")); 
      tft.setCursor(20, 80); tft.print(F("Calibration complete.")); 
      tft.setCursor(20, 100); tft.print(F("Press to view results.")); 
      drawButton(80, 130, 160, 40, F("RESULTS")); 
      strcpy_P(calib_status_msg, PSTR("Gains received from MCU1."));
      break;
      
    case 5: // Step 5: 최종 결과 화면
      displayCalibrationResults(); // 별도 함수 호출
      break;
  }
  
  // 상태 메시지 출력 (화면 하단)
  tft.fillRect(0, 225, SCREEN_WIDTH, 15, COLOR_BACKGROUND); // [v54] Y pos
  tft.setCursor(20, 225);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.print(calib_status_msg);
  calib_status_msg[0] = '\0'; // 메시지 초기화
}

// [v54] 자동 보정 결과 화면
void displayCalibrationResults() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println(F("CALIBRATION RESULTS")); 
  drawBackButton(); // 설정 메뉴로 돌아가기

  // --- Offsets (ADC Midpoints) ---
  tft.setCursor(10, 50);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.print(F("Offsets (ADC Mid):"));
  
  printTFTResult(10, 80,  F("V_ADC_MIDPOINT:"), mcu_v_midpoint, 2, COLOR_BLUE);
  printTFTResult(10, 100, F("I_ADC_MIDPOINT:"), mcu_i_midpoint, 2, COLOR_ORANGE);
  printTFTResult(10, 120, F("I1_ADC_MIDPOINT:"), mcu_i1_midpoint, 2, COLOR_DARKGREEN);
  printTFTResult(10, 140, F("I2_ADC_MIDPOINT:"), mcu_i2_midpoint, 2, COLOR_DARKGREEN);
  
  // --- Gains (Calibration Factors) ---
  tft.setCursor(10, 170);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.print(F("Gains (Calib):"));
  
  printTFTResult(10, 200, F("calib_v_rms:"), mcu_v_gain, 7, COLOR_BLUE); 
  printTFTResult(10, 220, F("calib_i_rms (all):"), mcu_i_gain, 7, COLOR_ORANGE); 
  
  // --- Restart Button ---
  drawButton(210, 180, 100, 40, F("DONE")); // [v54] "DONE" (뒤로 가기 버튼과 동일)
}


// [v54] 자동 보정 화면 (동적)
void runAutoCalib() {
  switch(calib_step) {
    case 3: { // 실효값 입력
      int y_positions[] = {50, 85, 120};
      
      if (prev_calib_selection != calib_selection) {
        if (prev_calib_selection != -1) {
          tft.fillRect(10, y_positions[prev_calib_selection], 15, 18, COLOR_BACKGROUND); 
        }
        tft.setTextColor(COLOR_RED); tft.setTextSize(2);
        tft.setCursor(10, y_positions[calib_selection]);
        tft.print(F(">")); 
        prev_calib_selection = calib_selection;
      }

      printTFTValue(200, 50, temp_true_v, prev_temp_true_v, 1, COLOR_BLUE, F(" V")); 
      printTFTValue(200, 85, temp_true_i, prev_temp_true_i, 3, COLOR_ORANGE, F(" A")); 
      printTFTValue(200, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, F("")); 

      prev_temp_true_v = temp_true_v;
      prev_temp_true_i = temp_true_i;
      prev_setting_step_index = setting_step_index;
      break;
    }
    
    case 0: // 대기
    case 1: // 측정 중...
    case 2: // 대기
    case 4: // 대기
    case 5: // 결과 표시 (정적)
      break;
  }
}

// [v54] 자동 보정 값 입력용
void adjustAutoCalibValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  switch (calib_selection) {
    case 0: // True V
      temp_true_v += (increase ? step_to_apply : -step_to_apply);
      if (temp_true_v < 0) temp_true_v = 0;
      break;
    case 1: // True I
      temp_true_i += (increase ? step_to_apply : -step_to_apply);
      if (temp_true_i < 0) temp_true_i = 0;
      break;
    case 2: // Step
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) setting_step_index = new_index;
      break;
  }
}

// ==============================================================================
// 36. [v54] MCU 1로 명령 전송 헬퍼
// ==============================================================================

// 자동 보정 명령 전송
void sendCalibrationCommand(char step, float v, float i) {
  CalibrationRequest req;
  req.command_char = 'C';
  req.calib_step = step;
  req.true_v = v;
  req.true_i = i;
  Serial.print(F("MCU2: Sending Calib Command (Step ")); Serial.print(step); Serial.println(F(")"));
  Serial1.write((uint8_t*)&req, sizeof(req));
}

// 현재 설정/보정값을 MCU 1로 전송 (동기화)
void sendCurrentSettingsToMCU1() {
  SettingsPacket packetToSend;
  packetToSend.command_char = 'S';
  
  // 현재 설정값
  packetToSend.v_multiplier = V_MULTIPLIER;
  packetToSend.i_multiplier = I_MULTIPLIER;
  packetToSend.voltage_threshold = VOLTAGE_THRESHOLD;
  
  // 현재 보정 계수 (Midpoints)
  packetToSend.v_midpoint = mcu_v_midpoint;
  packetToSend.i_midpoint = mcu_i_midpoint;
  packetToSend.i1_midpoint = mcu_i1_midpoint;
  packetToSend.i2_midpoint = mcu_i2_midpoint;
  
  // 현재 보정 계수 (Gains)
  packetToSend.v_gain = mcu_v_gain;
  packetToSend.i_gain = mcu_i_gain;
  packetToSend.i1_gain = mcu_i1_gain;
  packetToSend.i2_gain = mcu_i2_gain;
  
  // 현재 보정 계수 (Offset Adjusts)
  packetToSend.v_offset_adjust = mcu_v_offset_adjust;
  packetToSend.i_offset_adjust = mcu_i_offset_adjust;
  packetToSend.i1_offset_adjust = mcu_i1_offset_adjust;
  packetToSend.i2_offset_adjust = mcu_i2_offset_adjust;
  
  Serial.println(F("MCU2: Sending updated SettingsPacket to MCU1."));
  Serial1.write((uint8_t*)&packetToSend, sizeof(SettingsPacket));
}

// ==============================================================================
// 37. [v55] 타이머 화면 함수
// ==============================================================================

// [v55] 타이머 설정 화면 (정적)
void displaySettingsTimerStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("TIMER SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(30, 50); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Status:");
  tft.setCursor(30, 70); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Time Left:");
  
  tft.drawFastHLine(10, 90, 300, COLOR_GRID);

  tft.setCursor(30, 100); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Duration:"); 
  tft.setCursor(30, 120); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Step:"); 

  // START/STOP 버튼
  String btn_text = timerIsRunning ? "STOP" : "START";
  uint16_t btn_color = timerIsRunning ? COLOR_RED : COLOR_GREEN;
  drawButton(80, 130, 160, 40, btn_text, btn_color);
  
  // UP/DOWN/+/ - 버튼
  drawButton(20, 180, 60, 40, "UP");
  drawButton(90, 180, 60, 40, "DOWN");
  drawButton(180, 180, 60, 40, "-");
  drawButton(250, 180, 60, 40, "+");
}

// [v55] 타이머 설정 화면 (동적)
void runSettingsTimer() {
  int y_positions[] = {100, 120};
  
  // UP/DOWN 선택 하이라이트 (타이머가 꺼져있을 때만)
  if (!timerIsRunning) {
    if (prev_timer_selection != timer_selection) {
      if (prev_timer_selection != -1) tft.fillRect(10, y_positions[prev_timer_selection], 15, 18, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_RED); tft.setTextSize(2);
      tft.setCursor(10, y_positions[timer_selection]); tft.print(">");
      prev_timer_selection = timer_selection;
    }
  } else {
    // 타이머가 실행 중이면 하이라이트 숨김
    if (prev_timer_selection != -1) {
      tft.fillRect(10, y_positions[prev_timer_selection], 15, 18, COLOR_BACKGROUND); 
      prev_timer_selection = -1;
    }
  }

  // 값 표시 (타이머가 꺼져있을 때만 업데이트)
  if (!timerIsRunning) {
    printTFTValue(190, 100, temp_timerDurationMinutes, prev_temp_timerDurationMinutes, 1, COLOR_BLUE, " min"); 
    printTFTValue(190, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 
    prev_temp_timerDurationMinutes = temp_timerDurationMinutes;
    prev_setting_step_index = setting_step_index;
  }

  // 타이머 상태 및 남은 시간 표시
  String status_str;
  String time_left_str;
  
  if (timerIsRunning) {
    unsigned long elapsed = millis() - timerStartTime;
    unsigned long totalDurationMillis = timerDurationMinutes * 60000;
    
    if (elapsed >= totalDurationMillis) {
      // 타이머 만료!
      timerIsRunning = false;
      status_str = "OFF (Expired)";
      time_left_str = "00:00:00";
      
      // MCU 1로 릴레이 차단(Toggle) 명령 전송
      Serial.println(F("MCU2: Timer expired! Sending Trip command."));
      controlReq.command_char = 'T'; 
      controlReq.relay_num = '1'; // Relay 1
      Serial1.write((uint8_t*)&controlReq, sizeof(controlReq));
      delay(10); // 약간의 딜레이
      controlReq.relay_num = '2'; // Relay 2
      Serial1.write((uint8_t*)&controlReq, sizeof(controlReq));
      
      screenNeedsRedraw = true; // 버튼 텍스트를 "START"로 변경하기 위해
      
    } else {
      // 타이머 작동 중
      status_str = "ON";
      unsigned long remainingMillis = totalDurationMillis - elapsed;
      unsigned long remainingSecondsTotal = remainingMillis / 1000;
      
      int hours = remainingSecondsTotal / 3600;
      int minutes = (remainingSecondsTotal % 3600) / 60;
      int seconds = remainingSecondsTotal % 60;
      
      char buffer[10];
      sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
      time_left_str = buffer;
    }
  } else {
    // 타이머 꺼짐
    status_str = "OFF";
    time_left_str = "--:--:--";
  }
  
  printTFTValue(150, 50, status_str, prev_timer_status, timerIsRunning ? COLOR_GREEN : COLOR_RED);
  printTFTValue(150, 70, time_left_str, prev_time_left, COLOR_TEXT_PRIMARY);
  prev_timer_status = status_str;
  prev_time_left = time_left_str;
}

// [v55] 타이머 값 변경 헬퍼
void adjustTimerValue(bool increase) {
  // 타이머가 실행 중일 때는 값 변경 불가
  if (timerIsRunning) return; 

  float step_to_apply = setting_steps[setting_step_index];
  
  switch (timer_selection) {
    case 0: // Duration
      temp_timerDurationMinutes += (increase ? step_to_apply : -step_to_apply);
      if (temp_timerDurationMinutes < 0) temp_timerDurationMinutes = 0;
      if (temp_timerDurationMinutes > 1440) temp_timerDurationMinutes = 1440; // 24시간 제한
      break;
    case 1: // Step
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) setting_step_index = new_index;
      break;
  }
}