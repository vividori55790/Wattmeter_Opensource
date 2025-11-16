/*
 * ==============================================================================
 * [전력계 2-MCU 분리 시스템 - MCU 2: 제어 및 디스플레이]
 * - 담당: TFT/터치스크린, UI 로직, Wi-Fi/MQTT 전송, 릴레이 제어,
 * 'Waveform' 화면 전용 로컬 샘플링.
 * - 제외: 전력 계산(RMS, P, Q, S, PF), FFT/THD, 퍼지 로직 연산.
 * - 통신: Serial1(D0, RX)을 통해 MCU 1로부터 계산된 데이터(struct PowerData) 수신.
 * - 중요: 센서(A2~A5)는 MCU 1과 이 보드에 병렬 연결되어야 함.
 * ==============================================================================
 */


// --- 라이브러리 포함 (UI, 통신, 파형) ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <math.h>
// #include <Fuzzy.h> // MCU 1이 담당
// #include <FuzzySet.h> // MCU 1이 담당
#include <arm_math.h> // MCU 1이 담당

// [v45] SoftwareSerial 라이브러리 추가 (ESP-01용)
#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>

// --- [v20] 화면 상태 정의 (홈 화면 기반) ---
enum ScreenState {
  SCREEN_HOME,              // [v20] 홈 메뉴
  SCREEN_MAIN_POWER,
  SCREEN_PHASE_DIFFERENCE,
  SCREEN_COMBINED_WAVEFORM, 
  SCREEN_THD,
  SCREEN_SETTINGS,          // [v20] 설정 메뉴
  SCREEN_SETTINGS_CALIB,    // [v20] 2차 깊이: 보정
  SCREEN_SETTINGS_PROTECT,  // [v20] 2차 깊이: 보호
  SCREEN_RELAY_CONTROL,     // [v22] 릴레이 제어 화면
  SCREEN_SETTINGS_THEME,    // [v25] 테마 설정
  SCREEN_SETTINGS_RESET,    // [v25] 초기화
  SCREEN_CONFIRM_SAVE,      // [v25] 저장 확인
  SCREEN_WARNING
};
volatile ScreenState currentScreen = SCREEN_HOME; // [v20] 부팅 시 홈 화면
volatile ScreenState previousScreen = SCREEN_HOME; // [v25] 저장/취소 시 돌아갈 화면
volatile bool screenNeedsRedraw = true;

// --- 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
// 아날로그 핀은 파형 표시(로컬 샘플링)를 위해 유지
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TOUCH_CS   7
// [v39] ESP-01 (SoftwareSerial) 핀 정의
#define WIFI_TX_PIN 2
#define WIFI_RX_PIN 3

// --- [v38] WiFi 및 MQTT 설정 ---
#define WIFI_SSID "YOUR_WIFI_SSID"     // <-- 여기에 실제 Wi-Fi SSID를 입력하세요.
#define WIFI_PASS "YOUR_WIFI_PASSWORD" // <-- 여기에 실제 Wi-Fi 비밀번호를 입력하세요.
#define MQTT_BROKER "public.mqtthq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "wattmeter/data" // 데이터를 발행할 토픽

SoftwareSerial espSerial(WIFI_RX_PIN, WIFI_TX_PIN); // (RX = 3, TX = 2)
WiFiEspClient wifiClient; 
PubSubClient mqttClient(wifiClient);

long lastMqttPublish = 0;
const long mqttPublishInterval = 5000; // 5초마다 MQTT로 데이터 전송
char wifi_status_msg[10] = "INIT"; 


// --- [v25] 다크 모드 및 테마 ---
bool isDarkMode = false;
uint16_t COLOR_BACKGROUND;
uint16_t COLOR_TEXT_PRIMARY;
// ... (모든 색상 정의 유지) ...
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

// --- [v20] 실시간 설정 변수 ---
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// --- [v25] 설정 임시 저장 변수 ---
float temp_VOLTAGE_THRESHOLD;
float temp_V_MULTIPLIER;
float temp_I_MULTIPLIER;
int temp_setting_step_index;

// --- [v20] 설정 화면용 이전 값 ---
float prev_VOLTAGE_THRESHOLD = -1.0;
float prev_V_MULTIPLIER = -1.0;
float prev_I_MULTIPLIER = -1.0;

bool settingsChanged = false;

volatile bool warningActive = false;
String warningMessage = "";

// --- [v40] FFT 상수 (파형 표시에 필요한 부분만) ---
// #define FFT_N 512 // MCU 1이 담당
#define SAMPLING_FREQ_HZ 7680.0f
// ...

// --- 캘리브레이션 상수 (ADC 오프셋) ---
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

// --- 파형 화면 상수 (유지) ---
#define PLOT_X_START 30 
#define PLOT_X_END 290 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define PLOT_Y_START 50 
#define PLOT_Y_END 210 
#define PLOT_Y_CENTER (PLOT_Y_START + ((PLOT_Y_END - PLOT_Y_START) / 2)) 
const float PLOT_HEIGHT_HALF = (PLOT_Y_END - PLOT_Y_START) / 2.0;
#define WAVEFORM_SAMPLE_PERIOD_US 100 // [v40] 파형 표시 전용 샘플링 주기 (100us)
int last_frame_y_v[PLOT_WIDTH];
int last_frame_y_i[PLOT_WIDTH];
int last_frame_y_i1[PLOT_WIDTH];
int last_frame_y_i2[PLOT_WIDTH];

const int NUM_V_STEPS = 6;
const float V_AXIS_STEPS[NUM_V_STEPS] = {50.0, 100.0, 150.0, 250.0, 350.0, 500.0};
const int NUM_I_STEPS = 7;
const float I_AXIS_STEPS[NUM_I_STEPS] = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0};

float V_axis_max = V_AXIS_STEPS[4]; // 350V
float I_axis_max = I_AXIS_STEPS[2]; // 1.0A

// --- [v19] 페이저 다이어그램 상수 ---
#define PHASOR_CX 235
#define PHASOR_CY 130
#define PHASOR_RADIUS 75

// --- 전역 변수 (물리량) - MCU 1로부터 수신 ---
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
float32_t thd_v_value = 0.0;
float32_t thd_i_value = 0.0;

// --- 전역 변수 (이전 값 - 화면 클리어용) ---
float prev_phase_degrees = 0.0;
String prev_lead_lag_status = "---";
// ... (모든 prev_ 변수들 유지) ...
float prev_phase_main_deg = 0.0;
float prev_phase_load1_deg = 0.0;
float prev_phase_load2_deg = 0.0;
int prev_v_x = PHASOR_CX, prev_v_y = PHASOR_CY;
int prev_im_x = PHASOR_CX, prev_im_y = PHASOR_CY;
int prev_i1_x = PHASOR_CX, prev_i1_y = PHASOR_CY;
int prev_i2_x = PHASOR_CX, prev_i2_y = PHASOR_CY;
float32_t prev_thd_v = 0.0;
float32_t prev_thd_i = 0.0;

// --- FFT 버퍼 (제거) ---
// --- 퍼지 로직 변수 (제거) ---

// --- [v23] 설정 UI용 전역 변수 (유지) ---
float setting_steps[] = {0.0001, 0.01, 0.1, 1.0, 10.0};
// ... (모든 설정 변수 유지) ...
const int NUM_SETTING_STEPS = 5;
int setting_step_index = DEFAULT_SETTING_STEP_INDEX;
int prev_setting_step_index = -1;
int calib_selection = 0;
int prev_calib_selection = -1;
const int NUM_CALIB_SETTINGS = 3; 
int protect_selection = 0;
int prev_protect_selection = -1;
const int NUM_PROTECT_SETTINGS = 2;

// --- [MCU-Split] 통신용 데이터 구조체 ---
struct PowerData {
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
  float thd_v_value;
  float thd_i_value;
  float fuzzy_output_level;
  float voltage_threshold;
};
PowerData receivedData;


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("MCU 2 (Controller) Booting...");

  // MCU 1(연산부)과 통신하기 위한 하드웨어 직렬 포트
  Serial1.begin(115200); 
  
  setTheme(); 
  
  analogReadResolution(14); 
  // 아날로그 핀은 파형 샘플링을 위해 유지
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  // 릴레이 핀 초기화
  pinMode(RELAY_1_PIN, OUTPUT); 
  pinMode(RELAY_2_PIN, OUTPUT); 
  digitalWrite(RELAY_1_PIN, HIGH); 
  digitalWrite(RELAY_2_PIN, HIGH); 

  // TFT 및 터치스크린 초기화
  tft.begin();
  tft.setRotation(3); 
  ts.begin(SPI);
  ts.setRotation(3); 
  Serial.println("TFT & Touch OK");

  // WiFi 설정
  setup_wifi();

  // 퍼지 로직 및 FFT 초기화 (제거)
  
  screenNeedsRedraw = true; 
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  checkTouchInput();
  
  // --- [MCU-Split] MCU 1로부터 데이터 수신 ---
  if (Serial1.available() >= sizeof(receivedData)) {
    Serial1.readBytes((uint8_t*)&receivedData, sizeof(receivedData));

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
    thd_v_value = receivedData.thd_v_value;
    thd_i_value = receivedData.thd_i_value;
    
    // 수신된 퍼지 레벨로 로컬 릴레이 제어
    controlRelays(receivedData.fuzzy_output_level);
    
    // 수신된 과전압 임계값으로 과전압 보호 로직 실행
    if (V_rms > receivedData.voltage_threshold) {
      digitalWrite(RELAY_1_PIN, LOW); // [v45] 릴레이 활성화 (차단)
      digitalWrite(RELAY_2_PIN, LOW); // [v45] 릴레이 활성화 (차단)
      warningMessage = "OVER VOLTAGE!"; 
      warningActive = true;
      screenNeedsRedraw = true; 
    }
  }
  
  // --- WiFi/MQTT 로직 유지 ---
  if (WiFi.status() != WL_CONNECTED) {
    reconnect_mqtt();
  } else if (!mqttClient.connected()) {
    reconnect_mqtt();
  }
  mqttClient.loop();
  
  // --- 화면 다시 그리기 로직 유지 ---
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
              last_frame_y_i1[i] = PLOT_Y_CENTER;
              last_frame_y_i2[i] = PLOT_Y_CENTER;
          }
          break;
        case SCREEN_THD:
          displayTHDScreenStatic(); 
          break;
        // ... (모든 설정/UI 화면 케이스 유지) ...
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
  
  // --- [v40] 동적 UI 업데이트 (분리된 로직) ---
  switch(currentScreen) {
    case SCREEN_MAIN_POWER:
      // perform_unified_analysis(); // 제거됨
      if (screenNeedsRedraw || warningActive) break;
      displayMainScreenValues(); // 수신된 전역 변수 사용
      
      if (millis() - lastMqttPublish > mqttPublishInterval && mqttClient.connected()) {
        publishData(); // 수신된 전역 변수 사용
      }
      break;
      
    case SCREEN_PHASE_DIFFERENCE:
      // perform_unified_analysis(); // 제거됨
      if (screenNeedsRedraw || warningActive) break;
      displayPhaseScreenValues(); // 수신된 전역 변수 사용
      break;
      
    case SCREEN_COMBINED_WAVEFORM:
      // [MCU-Split] 이 화면은 로컬에서 직접 샘플링 수행
      waitForVoltageZeroCross(); 
      if (screenNeedsRedraw || warningActive) break; 
      runCombinedWaveformLoop(); // 로컬 함수 실행
      break;
      
    case SCREEN_THD:
      // perform_unified_analysis(); // 제거됨
      if (screenNeedsRedraw || warningActive) break; 
      displayTHDScreenValues(); // 수신된 전역 변수 사용
      break;
      
    // ... (모든 설정/UI 실행 함수 유지) ...
    case SCREEN_SETTINGS_CALIB:
      runSettingsCalib();
      break;
    case SCREEN_SETTINGS_PROTECT:
      runSettingsProtect();
      break;
    case SCREEN_RELAY_CONTROL: 
      runRelayControl();
      break;
    case SCREEN_SETTINGS_THEME: 
      runSettingsTheme();
      break;
    case SCREEN_HOME: 
    case SCREEN_SETTINGS:
    case SCREEN_SETTINGS_RESET: 
    case SCREEN_CONFIRM_SAVE: 
    case SCREEN_WARNING: 
      // 정적 화면
      break;
  }
}


// ==============================================================================
// 3. [v20] 터치 입력 확인 함수 (유지)
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
      digitalWrite(RELAY_1_PIN, HIGH); // [v45] 릴레이 비활성화 (HIGH)
      digitalWrite(RELAY_2_PIN, HIGH); // [v45] 릴레이 비활성화 (HIGH)
      currentScreen = SCREEN_HOME;
      screenNeedsRedraw = true;
      delay(100); // Debounce
      return; 
    }

    // --- [v20] 공통 뒤로 가기 버튼 (<) ---
    // (기존과 동일하게 모두 유지)
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

    // --- [v20] 화면별 버튼 로직 ---
    // (기존과 동일하게 모두 유지)
    switch (currentScreen) {
      case SCREEN_HOME:
        if (p.x >= 20 && p.x <= 150 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_MAIN_POWER;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_PHASE_DIFFERENCE;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_COMBINED_WAVEFORM;
          screenNeedsRedraw = true;
        }
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
        if (p.x >= 20 && p.x <= 300 && p.y >= 60 && p.y <= 95) {
          temp_V_MULTIPLIER = V_MULTIPLIER;
          temp_I_MULTIPLIER = I_MULTIPLIER;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_CALIB;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 105 && p.y <= 140) {
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
          temp_setting_step_index = setting_step_index;
          settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_PROTECT;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 160 && p.y <= 200) {
          currentScreen = SCREEN_SETTINGS_THEME;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 160 && p.y <= 200) {
          currentScreen = SCREEN_SETTINGS_RESET;
          screenNeedsRedraw = true;
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
          adjustCalibValue(false);
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustCalibValue(true);
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
          adjustProtectValue(false);
        }
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) {
          adjustProtectValue(true);
        }
        break;
        
      case SCREEN_RELAY_CONTROL:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) {
          digitalWrite(RELAY_1_PIN, !digitalRead(RELAY_1_PIN)); // 토글
          screenNeedsRedraw = true; 
        }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) {
          digitalWrite(RELAY_2_PIN, !digitalRead(RELAY_2_PIN)); // 토글
          screenNeedsRedraw = true; 
        }
        break;

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
        
      case SCREEN_SETTINGS_RESET:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          restoreDefaultSettings();
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_CONFIRM_SAVE:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          if (previousScreen == SCREEN_SETTINGS_CALIB) {
            V_MULTIPLIER = temp_V_MULTIPLIER;
            I_MULTIPLIER = temp_I_MULTIPLIER;
          } else if (previousScreen == SCREEN_SETTINGS_PROTECT) {
            VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD;
          }
          setting_step_index = temp_setting_step_index; 
          
          currentScreen = SCREEN_SETTINGS;
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
  tft.setTextColor(COLOR_BACKGROUND);
  tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit); 
}

void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (value.equals(prev_value)) return;
  
  tft.setTextColor(COLOR_BACKGROUND);
  tft.setCursor(x, y);
  tft.print(prev_value);
  tft.setTextColor(color);
  tft.setCursor(x, y); 
  tft.print(value);
}

// ... (calculatePhase 함수는 MCU 1로 이동했으므로 제거) ...

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
// 5. 메인 전력 화면 그리기 (유지)
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

  // Column 1 (Rows 1-5)
  tft.setCursor(col1_label_x, y_row1); tft.setTextColor(COLOR_BLUE); tft.println("V:");
  tft.setCursor(col1_label_x, y_row2); tft.setTextColor(COLOR_ORANGE); tft.println("I-M:");
  tft.setCursor(col1_label_x, y_row3); tft.setTextColor(COLOR_DARKGREEN); tft.println("P:");
  tft.setCursor(col1_label_x, y_row4); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:");
  tft.setCursor(col1_label_x, y_row5); tft.setTextColor(COLOR_ORANGE); tft.println("Q:");

  // Column 2 (Rows 1-5)
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
  
  int col1_value_x = 60;
  int col2_value_x = 220; 
  int col_w_half = 90; 
  int col_w_half_wide = 100;
  
  int y_row1 = 40;
  int y_row2 = 65;
  int y_row3 = 90;
  int y_row4 = 115;
  int y_row5 = 140;

  // --- Column 1 (Rows 1-5) ---
  tft.fillRect(col1_value_x, y_row1, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE); 
  tft.setCursor(col1_value_x, y_row1);
  dtostrf(V_rms, 4, 1, buffer);
  tft.print(buffer); tft.print(" V");

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

  tft.fillRect(col1_value_x, y_row4, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_MAGENTA);
  tft.setCursor(col1_value_x, y_row4);
  dtostrf(PF, 4, 2, buffer);
  tft.print(buffer);

  tft.fillRect(col1_value_x, y_row5, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, y_row5);
  if (Q_reactive >= 1000.0) { 
    dtostrf(Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVAR");
  } else { 
    dtostrf(Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" VAR");
  }

  // --- Column 2 (Rows 1-5) ---
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

  tft.fillRect(col2_value_x, y_row4, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_BLUE);
  tft.setCursor(col2_value_x, y_row4);
  dtostrf(thd_v_value * 100.0, 4, 1, buffer);
  tft.print(buffer); tft.print(" %");

  tft.fillRect(col2_value_x, y_row5, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col2_value_x, y_row5);
  dtostrf(thd_i_value * 100.0, 4, 1, buffer);
  tft.print(buffer); tft.print(" %");
}


// ==============================================================================
// 7. [v19] 위상차 화면 그리기 (유지)
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
// 9. 60Hz 0점 통과(Zero-Crossing) 대기 함수 (파형 화면 전용 - 유지)
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
// 10. [v19] 공통 파형 화면 (정적 그리드 + Vpk/Ipk 라벨) (유지)
// ==============================================================================
void drawWaveformGridAndLabels() {
  
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("V/I WAVEFORM (60Hz Sync)");
  drawBackButton();
  
  int legendX = 200;
  int legendY = 28;
  tft.setTextSize(1);
  tft.drawFastHLine(legendX, legendY, 10, COLOR_BLUE); tft.setTextColor(COLOR_BLUE); tft.setCursor(legendX + 12, legendY - 3); tft.print("V");
  tft.drawFastHLine(legendX + 30, legendY, 10, COLOR_ORANGE); tft.setTextColor(COLOR_ORANGE); tft.setCursor(legendX + 42, legendY - 3); tft.print("I-M");
  tft.drawFastHLine(legendX, legendY + 12, 10, COLOR_RED); tft.setTextColor(COLOR_RED); tft.setCursor(legendX + 12, legendY + 9); tft.print("I-1");
  tft.drawFastHLine(legendX + 30, legendY + 12, 10, COLOR_GREEN); tft.setTextColor(COLOR_GREEN); tft.setCursor(legendX + 42, legendY + 9); tft.print("I-2");

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
// 11. 동적 Y축 라벨 업데이트 함수 (유지)
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
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
  } else { 
    dtostrf(I_axis_max, 3, 1, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
  }

  tft.setTextColor(COLOR_BLUE);
  dtostrf(V_axis_max, 3, 0, buffer); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); 
}


// ==============================================================================
// 12. 경고 팝업 화면 (유지)
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
  
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2); 
  String resetMsg = "Tap screen to reset";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); 
  tft.print(resetMsg); 
}


// ==============================================================================
// [v40] 통합 샘플링 및 분석 함수 (제거)
// ==============================================================================
// void perform_unified_analysis() { ... } // MCU 1이 담당


// ==============================================================================
// 16. [v19] 파형 그리기 루프 (유지 - 로컬 샘플링)
// ==============================================================================
void runCombinedWaveformLoop() {
  float volts_to_pixels_scale = (V_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_axis_max);
  float amps_to_pixels_scale = (I_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / I_axis_max);

  float new_frame_V_peak = 0.0;
  float new_frame_I_peak = 0.0;
  
  int new_y_v[PLOT_WIDTH];
  int new_y_i[PLOT_WIDTH];
  int new_y_i1[PLOT_WIDTH];
  int new_y_i2[PLOT_WIDTH];

  unsigned long startTime = micros();

  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  for (int i = 0; i < PLOT_WIDTH; i++) {
    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int I1_raw = analogRead(CURRENT_PIN_LOAD1);
    int I2_raw = analogRead(CURRENT_PIN_LOAD2);
    
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    int I1_ac_bits = I1_raw - (int)I_ADC_MIDPOINT;
    int I2_ac_bits = I2_raw - (int)I_ADC_MIDPOINT;
    
    float V_mains_instant = V_ac_bits * effective_V_Calib + effective_V_Offset;
    float I_mains_instant = I_ac_bits * effective_I_Calib - effective_I_Offset;
    float I1_mains_instant = I1_ac_bits * effective_I_Calib - effective_I_Offset;
    float I2_mains_instant = I2_ac_bits * effective_I_Calib - effective_I_Offset;

    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    
    if (abs(I_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I_mains_instant);
    if (abs(I1_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I1_mains_instant);
    if (abs(I2_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I2_mains_instant);

    int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
    new_y_v[i] = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
    new_y_i[i] = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i1 = PLOT_Y_CENTER - (int)(I1_mains_instant * amps_to_pixels_scale);
    new_y_i1[i] = constrain(y_pos_i1, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i2 = PLOT_Y_CENTER - (int)(I2_mains_instant * amps_to_pixels_scale);
    new_y_i2[i] = constrain(y_pos_i2, PLOT_Y_START, PLOT_Y_END);
    
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
      if(last_frame_y_i1[i] != PLOT_Y_CENTER || last_frame_y_i1[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_i1[i-1], x_curr, last_frame_y_i1[i], COLOR_BACKGROUND);
      }
      if(last_frame_y_i2[i] != PLOT_Y_CENTER || last_frame_y_i2[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_i2[i-1], x_curr, last_frame_y_i2[i], COLOR_BACKGROUND);
      }

      tft.drawLine(x_prev, new_y_v[i-1], x_curr, new_y_v[i], COLOR_BLUE);
      tft.drawLine(x_prev, new_y_i[i-1], x_curr, new_y_i[i], COLOR_ORANGE);
      tft.drawLine(x_prev, new_y_i1[i-1], x_curr, new_y_i1[i], COLOR_RED);
      tft.drawLine(x_prev, new_y_i2[i-1], x_curr, new_y_i2[i], COLOR_GREEN);
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
      last_frame_y_v[i] = new_y_v[i];
      last_frame_y_i[i] = new_y_i[i];
      last_frame_y_i1[i] = new_y_i1[i];
      last_frame_y_i2[i] = new_y_i2[i];
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
// 17. 퍼지 로직 시스템 빌드 함수 (제거)
// ==============================================================================
// void buildFuzzySystem() { ... } // MCU 1이 담당

// ==============================================================================
// 18. 퍼지 로직 실행 헬퍼 함수 (제거)
// ==============================================================================
// void runFuzzyLogic() { ... } // MCU 1이 담당 (결과만 수신)

// ==============================================================================
// 19. 퍼지 출력 -> 릴레이 제어 변환 함수 (유지)
// ==============================================================================
void controlRelays(float level) {
  // [v45] 릴레이 로직 수정: LOW = 활성화(차단), HIGH = 비활성화(공급)
  // 레벨 9 이상 (Critical): 릴레이 1, 2 모두 활성화 (전원 차단)
  if (level > 9.0) {
    digitalWrite(RELAY_1_PIN, LOW); // 활성화 (차단)
    digitalWrite(RELAY_2_PIN, LOW); // 활성화 (차단)
    if (!warningActive) {
      warningMessage = "FUZZY LOGIC TRIP";
      warningActive = true; 
      screenNeedsRedraw = true; 
    }
  } 
  // 레벨 5~9 (Danger): 릴레이 1, 2 모두 활성화 (전원 차단)
  else if (level > 5.0) {
    digitalWrite(RELAY_1_PIN, LOW); // 활성화 (차단)
    digitalWrite(RELAY_2_PIN, LOW); // 활성화 (차단)
  } 
  // 레벨 2~5 (Warning): 릴레이 2만 활성화 (부하 2만 차단)
  else if (level > 2.0) {
    digitalWrite(RELAY_1_PIN, HIGH); // 비활성화 (전원 유지)
    digitalWrite(RELAY_2_PIN, LOW);  // 활성화 (부하 2 차단)
  } 
  // 레벨 0~2 (Safe): 릴레이 1, 2 모두 비활성화 (전원 공급)
  else { 
    if (!warningActive) {
      digitalWrite(RELAY_1_PIN, HIGH); // 비활성화 (공급)
      digitalWrite(RELAY_2_PIN, HIGH); // 비활성화 (공급)
    }
  }
}

// ==============================================================================
// 21. THD 화면 정적 UI 그리기 (유지)
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

// ==============================================================================
// 22. THD 화면 동적 값 업데이트 (유지)
// ==============================================================================
void displayTHDScreenValues() {
  tft.setTextSize(4);
  printTFTValue(30, 110, thd_v_value * 100.0, prev_thd_v * 100.0, 1, COLOR_BLUE, " %"); 
  printTFTValue(30, 190, thd_i_value * 100.0, prev_thd_i * 100.0, 1, COLOR_ORANGE, " %"); 
  prev_thd_v = thd_v_value; 
  prev_thd_i = thd_i_value; 
}


// ==============================================================================
// 23. [v40] CMSIS-DSP FFT 및 THD 계산 헬퍼 함수 (제거)
// ==============================================================================
// float32_t calculateTHD(float32_t* mags, int fundamentalBin) { ... } // MCU 1이 담당


// ==============================================================================
// 25. [v20] 홈 화면 그리기 (유지)
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
// 26. [v20] 설정 메인 화면 그리기 (유지)
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
// 27. [v20] 설정 - 보정 화면 그리기 (유지)
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
  
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  prev_setting_step_index = setting_step_index;
}

// ==============================================================================
// 29. [v22] 릴레이 제어 화면 그리기 (유지)
// ==============================================================================
void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton();

  // [v45] 릴레이 로직 수정: HIGH(비활성화)일 때 "ON" (전원 공급), LOW(활성화)일 때 "OFF" (전원 차단)
  String r1_status = digitalRead(RELAY_1_PIN) ? "ON" : "OFF (TRIPPED)";
  String r2_status = digitalRead(RELAY_2_PIN) ? "ON" : "OFF (TRIPPED)";
  
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
// 31. [v24] 파형 Y축 범위 계산 헬퍼 (유지)
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
// 32. [v25] 신규 화면 그리기 (유지)
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

// --- 초기화 헬퍼 ---
void restoreDefaultSettings() {
  VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
  V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
  I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
  setting_step_index = DEFAULT_SETTING_STEP_INDEX;
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
    Serial.println("ESP-01 module not found or not responding.");
    strcpy(wifi_status_msg, "ESP_ERR");
    screenNeedsRedraw = true;
    return;
  }

  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  strcpy(wifi_status_msg, "CONN...");
  screenNeedsRedraw = true;
  
  int retries = 0;
  
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    WiFi.begin(WIFI_SSID, WIFI_PASS); 
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    strcpy(wifi_status_msg, "MQTT...");
  } else {
    Serial.println("\nWiFi connection FAILED");
    strcpy(wifi_status_msg, "WIFI_ERR");
  }
  screenNeedsRedraw = true;
}

void reconnect_mqtt() {
  if (WiFi.status() == WL_NO_SHIELD) {
     strcpy(wifi_status_msg, "ESP_ERR");
     screenNeedsRedraw = true;
     setup_wifi();
     return;
  }
  if (WiFi.status() != WL_CONNECTED) {
     strcpy(wifi_status_msg, "WIFI_ERR");
     screenNeedsRedraw = true;
     setup_wifi();
     return;
  }
  
  if (mqttClient.connected()) {
    return;
  }

  Serial.print("Attempting MQTT connection...");
  strcpy(wifi_status_msg, "MQTT...");
  screenNeedsRedraw = true;
  
  String clientId = "arduinoWattmeterClient-";
  clientId += String(random(0xffff), HEX);
  
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected");
    strcpy(wifi_status_msg, "NET: OK");
    lastMqttPublish = millis();
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 5 seconds");
    strcpy(wifi_status_msg, "MQTT_ERR");
  }
  screenNeedsRedraw = true;
}

void publishData() {
  if (!mqttClient.connected()) {
    reconnect_mqtt();
    return;
  }
  
  char payload[300];
  // 전역 변수(MCU 1에서 수신)를 사용하여 MQTT 페이로드 생성
  snprintf(payload, 300, 
    "{\"V_rms\":%.2f, \"I_rms\":%.3f, \"I_1\":%.3f, \"I_2\":%.3f, \"P_real\":%.2f, \"S_app\":%.2f, \"Q_react\":%.2f, \"PF\":%.2f, \"THD_V\":%.2f, \"THD_I\":%.2f}",
    V_rms, I_rms, I_rms_load1, I_rms_load2, P_real, S_apparent, Q_reactive, PF, (thd_v_value * 100.0), (thd_i_value * 100.0)
  );

  Serial.print("Publishing message: ");
  Serial.println(payload);
  
  if (mqttClient.publish(MQTT_TOPIC, payload)) {
     Serial.println("Publish OK");
  } else {
     Serial.println("Publish FAILED");
  }
  
  lastMqttPublish = millis();
}