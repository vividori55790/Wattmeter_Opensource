/*
 * ==============================================================================
 * 0. 라이브러리 포함 및 상수/변수 정의 (Arduino Nano R4 - 핀 수정 및 기능 통합)
 * ==============================================================================
 * [Ver 5.0 - Wi-Fi 모듈 데이터 전송 (JSON)]
 * 1. MCU: Arduino Nano R4 (Renesas RA4M1)
 * 2. Display: Adafruit ILI9341 (TFT) [D10, D9, D8]
 * 3. Touch: XPT2046 (Touch Panel) [D7, D6]
 * 4. SPI: D11(MOSI), D12(MISO), D13(SCK) - TFT/Touch 공유
 * 5. Sensors: A3(Voltage), A4(Current 1 - Ch1), A5(Current 2 - Ch2)
 * 6. Relays: A2(Main), D4(Relay 1 - Ch1), D5(Relay 2 - Ch2)
 * 7. 기능: 다중 화면(터치), 파형, 퍼지 경고, 우선순위 부하 차단
 * 8. [NEW] Wi-Fi: SoftwareSerial (D2/D3)을 통해 JSON 데이터 1초마다 전송
 * ==============================================================================
 */

// --- 필수 라이브러리 ---
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h> 
#include <FspTimer.h>
#include <arduinoFFT.h>
#include <math.h>
#include <SoftwareSerial.h> // [NEW] Wi-Fi 모듈 통신용

// --- 화면 상태 정의 ---
enum ScreenState {
  SCREEN_MAIN_POWER,
  SCREEN_WAVEFORM,
  SCREEN_FAULT
};
volatile ScreenState currentScreen = SCREEN_MAIN_POWER;

// --- 핀 정의 (dropship_text (1).txt 기반) ---
// SPI 핀 (공유)
#define SPI_MOSI 11 // D11 (MOSI / T_DIN)
#define SPI_MISO 12 // D12 (MISO / T_DO)
#define SPI_SCK  13 // D13 (SCK / T_CLK)

// TFT 핀
#define TFT_CS 10     // D10 (TFT CS)
#define TFT_DC 9      // D9 (TFT DC)
#define TFT_RST 8     // D8 (TFT RST)

// Touch 핀
#define TOUCH_CS 7      // D7 (T_CS)
#define TOUCH_IRQ 6     // D6 (T_IRQ)

// 릴레이 핀 (3개)
#define RELAY_PIN_MAIN A2  // A2 (릴레이 0 신호 - 메인)
#define RELAY_PIN_CH1  D4  // D4 (릴레이 1 신호 - Ch1 고우선순위)
#define RELAY_PIN_CH2  D5  // D5 (릴레이 2 신호 - Ch2 저우선순위)

// 센서 핀
#define VOLTAGE_PIN A3 // A3 (전압센서 1)
#define CURRENT_PIN_1 A4 // A4 (전류센서 1 - Ch1)
#define CURRENT_PIN_2 A5 // A5 (전류센서 2 - Ch2)

// [NEW] Wi-Fi 모듈용 SoftwareSerial 핀
// (D2가 TX, D3가 RX라고 명시됨)
#define WIFI_RX_PIN 3  // Arduino RX(D3) <-> Wi-Fi TX
#define WIFI_TX_PIN 2  // Arduino TX(D2) <-> Wi-Fi RX

// --- 채널 및 샘플링 설정 ---
#define NUM_CHANNELS 2       
#define SAMPLE_PERIOD_MS 1   
#define SAMPLES_PER_CALC 167 
#define FFT_SAMPLES 256      

// --- 퍼지 논리(경고) 및 부하 차단 임계값 (1500W 정격 기준) ---
#define POWER_RECOVERY_W 1200.0  // 1200W 미만 시 저우선순위 부하 복구
#define POWER_WARNING_W 1300.0   // 1300W 초과 시 '주의'
#define POWER_CRITICAL_W 1500.0  // 1500W 초과 시 '위험' (정격) / 부하 차단 시작
#define OVERLOAD_TRIP_CYCLES 3 

// --- 교정 및 오프셋 계수 (Nano R4 14-bit ADC 기준) ---
// !!! [중요] 이 값은 실제 회로 연결 후 반드시 재교정(Calibration)해야 합니다.
const int V_OFFSET_ADC = 8192; 
const int I_OFFSET_ADC_1 = 8192; 
const int I_OFFSET_ADC_2 = 8192; 
const float K_V_REAL = 0.0431; 
const float K_I_REAL_1 = 0.0305; 
const float K_I_REAL_2 = 0.0305; 

// --- 객체 선언 ---
FspTimer gpt_timer;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS); 
ArduinoFFT<double> FFT;
SoftwareSerial wifiSerial(WIFI_RX_PIN, WIFI_TX_PIN); // [NEW] (RX, TX)

// SPI 설정 (TFT/Touch 공유용)
SPISettings spiSettingsTFT(8000000, MSBFIRST, SPI_MODE0); 
SPISettings spiSettingsTouch(2000000, MSBFIRST, SPI_MODE0); 

// --- 다채널 누적 변수 (ISR 접근) ---
volatile int sampleCount = 0;
volatile float V_sq_sum = 0.0;
volatile float I_sq_sum[NUM_CHANNELS] = {0.0, 0.0};
volatile float P_sum[NUM_CHANNELS] = {0.0, 0.0};
volatile bool calculationReady = false;

// --- FFT 버퍼 ---
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
int fft_count = 0;

// --- 부하 차단(Shedding) 상태 변수 ---
volatile int overload_counter = 0;
volatile bool protectionTripped_Critical = false; 
volatile bool lowPriorityShed = false;          

// --- 터치 좌표 변수 ---
int touch_x, touch_y;

// --- [NEW] Wi-Fi 전송 타이머 ---
unsigned long lastWifiSend = 0;
const long wifiSendInterval = 1000; // 1초 (1000ms)
char jsonBuffer[256]; // JSON 문자열 버퍼

// ==============================================================================
// 1. 함수 프로토타입 선언
// ==============================================================================
void setup_timer_interrupt();
void timer_ISR_routine();
void setup_display_and_touch();
void checkTouchInput(); 
void manageLoadPriorities(float P_total, float P_ch1); 
void checkFuzzyWarnings(float totalPower);
float calculate_THD();
void sendDataToWifi(float V_rms, float P1, float I1, float P2, float I2, float P_total, float THD_val); // [NEW]

// 화면별 출력 함수
void displayMainScreen(float V_rms, float I_rms[], float P_real[], float Q_react[], float PF[], float P_total, float Q_total, float S_total, float PF_total, float THD);
void displayWaveformScreen();
void displayCriticalFaultScreen(const char* message); 
void drawWaveform(int x, int y, int w, int h, double* data, int dataSize, uint16_t color);
void drawTouchButtons();

// ==============================================================================
// 2. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200); // PC 시리얼 모니터
  wifiSerial.begin(115200); // [NEW] Wi-Fi 모듈 통신
  
  // --- ADC 해상도 설정 (Nano R4: 14-bit) ---
  analogReadResolution(14);

  // --- 릴레이 핀 3개 초기화 (핀맵 기반) ---
  pinMode(RELAY_PIN_MAIN, OUTPUT); 
  pinMode(RELAY_PIN_CH1, OUTPUT); 
  pinMode(RELAY_PIN_CH2, OUTPUT); 
  
  digitalWrite(RELAY_PIN_MAIN, HIGH); 
  digitalWrite(RELAY_PIN_CH1, HIGH);
  digitalWrite(RELAY_PIN_CH2, HIGH);
  
  // --- 디스플레이 및 터치 설정 ---
  setup_display_and_touch();

  // --- 타이머 인터럽트 설정 (Nano R4) ---
  setup_timer_interrupt();

  Serial.println("--- 다채널 전력계 초기화 완료 (v5.0 - WiFi 전송) ---");
  tft.println("시스템 초기화 (v5.0)");
  tft.println("Wi-Fi 전송 활성화 (D2,D3)");
  delay(2000);
}

// ==============================================================================
// 3. Main Loop
// ==============================================================================
void loop() {
  
  // 치명적 오류(수동 리셋 필요) 상태인 경우
  if (protectionTripped_Critical) {
    displayCriticalFaultScreen("치명적 오류! 수동 리셋 필요");
    checkTouchInput(); 
    delay(500);
    return;
  }
  
  // 터치 입력 처리
  checkTouchInput();

  // 계산 준비 플래그 확인
  if (calculationReady) {
    calculationReady = false;

    // 1. 변수 안전 복사 및 초기화
    noInterrupts();
    float temp_V_sq_sum = V_sq_sum;
    int temp_sampleCount = sampleCount;
    float temp_I_sq_sum[NUM_CHANNELS];
    float temp_P_sum[NUM_CHANNELS];
    for(int i=0; i<NUM_CHANNELS; i++) {
        temp_I_sq_sum[i] = I_sq_sum[i];
        temp_P_sum[i] = P_sum[i];
        I_sq_sum[i] = 0.0; P_sum[i] = 0.0;
    }
    V_sq_sum = 0.0; sampleCount = 0;
    interrupts();

    // 2. RMS 계산 (전압은 공통)
    float V_rms_adc = sqrt(temp_V_sq_sum / temp_sampleCount);
    float V_rms_real = V_rms_adc * K_V_REAL;
    if (V_rms_real < 50) V_rms_real = 0.0; // 노이즈 플로어 처리

    // 3. 채널별 RMS, P, S, Q, PF 계산
    float I_rms_real[NUM_CHANNELS];
    float P_avg_real[NUM_CHANNELS];
    float S_apparent[NUM_CHANNELS];
    float PF[NUM_CHANNELS];
    float Q_reactive[NUM_CHANNELS];
    float P_total = 0.0;
    float Q_total = 0.0;

    // Ch 1 (A4)
    float I_rms_adc_1 = sqrt(temp_I_sq_sum[0] / temp_sampleCount);
    I_rms_real[0] = I_rms_adc_1 * K_I_REAL_1;
    if (I_rms_real[0] < 0.03) I_rms_real[0] = 0.0; // 노이즈 플로어
    P_avg_real[0] = (temp_P_sum[0] / temp_sampleCount) * K_V_REAL * K_I_REAL_1;
    S_apparent[0] = V_rms_real * I_rms_real[0];
    if (S_apparent[0] < 0.1) S_apparent[0] = 0.0; // 노이즈 플로어
    if (P_avg_real[0] > S_apparent[0] || S_apparent[0] == 0.0) P_avg_real[0] = S_apparent[0];
    PF[0] = (S_apparent[0] != 0.0) ? (P_avg_real[0] / S_apparent[0]) : 1.0;
    if (PF[0] > 1.0) PF[0] = 1.0; if (PF[0] < -1.0) PF[0] = -1.0;
    Q_reactive[0] = (S_apparent[0] * S_apparent[0]) - (P_avg_real[0] * P_avg_real[0]);
    Q_reactive[0] = (Q_reactive[0] > 0) ? sqrt(Q_reactive[0]) : 0;
    if (I_rms_real[0] > 0.03 && P_avg_real[0] < 0.1) Q_reactive[0] = S_apparent[0]; // 순수 무효전력

    // Ch 2 (A5)
    float I_rms_adc_2 = sqrt(temp_I_sq_sum[1] / temp_sampleCount);
    I_rms_real[1] = I_rms_adc_2 * K_I_REAL_2;
    if (I_rms_real[1] < 0.03) I_rms_real[1] = 0.0; // 노이즈 플로어
    P_avg_real[1] = (temp_P_sum[1] / temp_sampleCount) * K_V_REAL * K_I_REAL_2;
    S_apparent[1] = V_rms_real * I_rms_real[1];
    if (S_apparent[1] < 0.1) S_apparent[1] = 0.0; // 노이즈 플로어
    if (P_avg_real[1] > S_apparent[1] || S_apparent[1] == 0.0) P_avg_real[1] = S_apparent[1];
    PF[1] = (S_apparent[1] != 0.0) ? (P_avg_real[1] / S_apparent[1]) : 1.0;
    if (PF[1] > 1.0) PF[1] = 1.0; if (PF[1] < -1.0) PF[1] = -1.0;
    Q_reactive[1] = (S_apparent[1] * S_apparent[1]) - (P_avg_real[1] * P_avg_real[1]);
    Q_reactive[1] = (Q_reactive[1] > 0) ? sqrt(Q_reactive[1]) : 0;
    if (I_rms_real[1] > 0.03 && P_avg_real[1] < 0.1) Q_reactive[1] = S_apparent[1];

    // 4. 총계 계산
    P_total = P_avg_real[0] + P_avg_real[1];
    Q_total = Q_reactive[0] + Q_reactive[1];
    float S_total_vector = sqrt(P_total * P_total + Q_total * Q_total);
    float PF_total = (S_total_vector != 0.0) ? (P_total / S_total_vector) : 1.0;

    // 5. THD 계산
    float THD_value = calculate_THD();

    // 6. 우선순위 부하 차단/관리 로직 실행
    manageLoadPriorities(P_total, P_avg_real[0]);

    // 7. 현재 화면 상태에 따라 다른 내용 출력
    tft.fillScreen(ILI9341_BLACK);
    
    switch(currentScreen) {
      case SCREEN_MAIN_POWER:
        displayMainScreen(V_rms_real, I_rms_real, P_avg_real, Q_reactive, PF, P_total, Q_total, S_total_vector, PF_total, THD_value);
        break;
      case SCREEN_WAVEFORM:
        displayWaveformScreen();
        break;
      case SCREEN_FAULT:
        displayCriticalFaultScreen("치명적 오류 발생");
        break;
    }
    
    // 8. 퍼지 경고 확인 (데이터 출력 후 상단에 덮어쓰기)
    checkFuzzyWarnings(P_total); 
    drawTouchButtons(); // 공통 버튼 그리기

    // 9. [NEW] 1초마다 Wi-Fi 모듈로 JSON 데이터 전송
    unsigned long now = millis();
    if (now - lastWifiSend > wifiSendInterval) {
      lastWifiSend = now;
      sendDataToWifi(V_rms_real, 
                     P_avg_real[0], I_rms_real[0], 
                     P_avg_real[1], I_rms_real[1], 
                     P_total, THD_value);
    }

    // 10. FFT 카운터 초기화
    fft_count = 0;
  }
}

// ==============================================================================
// 5. ISR 및 보조 함수 (기존 v4와 대부분 동일)
// ==============================================================================

/**
 * @brief Nano R4용 FspTimer 설정
 */
void setup_timer_interrupt() {
  gpt_timer.begin(TIMER_MODE_PERIODIC, GPT_TIMER_CH0, (SAMPLE_PERIOD_MS * 1000), MICROSECONDS);
  gpt_timer.setCallback(timer_ISR_routine);
  gpt_timer.start();
}

/**
 * @brief 1ms 타이머 ISR (다채널 샘플링)
 */
void timer_ISR_routine() {
  if (calculationReady) return;
  int V_raw = analogRead(VOLTAGE_PIN);   
  int I_raw_1 = analogRead(CURRENT_PIN_1); 
  int I_raw_2 = analogRead(CURRENT_PIN_2); 
  float V_ac = (float)(V_raw - V_OFFSET_ADC);
  float I_ac_1 = (float)(I_raw_1 - I_OFFSET_ADC_1);
  float I_ac_2 = (float)(I_raw_2 - I_OFFSET_ADC_2);
  V_sq_sum += V_ac * V_ac;
  I_sq_sum[0] += I_ac_1 * I_ac_1;
  P_sum[0]    += V_ac * I_ac_1;
  I_sq_sum[1] += I_ac_2 * I_ac_2;
  P_sum[1]    += V_ac * I_ac_2;
  if (fft_count < FFT_SAMPLES) {
    vReal[fft_count] = (double)V_ac;
    vImag[fft_count] = 0;
  }
  fft_count++;
  sampleCount++;
  if (sampleCount >= SAMPLES_PER_CALC) {
    calculationReady = true;
  }
}

/**
 * @brief ILI9341 디스플레이 및 XPT2046 터치 초기 설정
 */
void setup_display_and_touch() {
  tft.begin();
  tft.setRotation(3); // 가로 모드 (320x240)
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.println("TFT OK");

  ts.begin(spiSettingsTouch); 
  ts.setRotation(3); 
  tft.println("Touch OK");
  delay(1000);
}

/**
 * @brief 터치 입력 확인 및 화면 상태 변경
 */
void checkTouchInput() {
  if (protectionTripped_Critical) {
    currentScreen = SCREEN_FAULT;
    return;
  }

  SPI.beginTransaction(spiSettingsTouch); 
  bool touched = ts.touched();
  SPI.endTransaction();

  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    // 터치 좌표 보정 (TFT.setRotation(3) - 320x240 기준)
    // 이 값은 실제 하드웨어에 맞게 보정(calibrate)이 필요합니다.
    touch_x = map(p.y, 200, 3800, 0, 320); 
    touch_y = map(p.x, 300, 3800, 0, 240); 
    touch_y = 240 - touch_y; 

    touch_x = constrain(touch_x, 0, 320);
    touch_y = constrain(touch_y, 0, 240);

    Serial.print("터치: X="); Serial.print(touch_x); Serial.print(", Y="); Serial.println(touch_y);

    if (touch_y > 200) { 
      if (touch_x > 0 && touch_x < 160) {
        if (currentScreen != SCREEN_MAIN_POWER) {
          currentScreen = SCREEN_MAIN_POWER;
          Serial.println("화면 -> 메인");
        }
      } else if (touch_x > 160 && touch_x < 320) {
         if (currentScreen != SCREEN_WAVEFORM) {
          currentScreen = SCREEN_WAVEFORM;
          Serial.println("화면 -> 파형");
         }
      }
    }
    delay(100); // 디바운싱
  }
}

/**
 * @brief [NEW] Wi-Fi 모듈로 JSON 데이터 전송
 */
void sendDataToWifi(float V_rms, float P1, float I1, float P2, float I2, float P_total, float THD_val) {
  // JSON 문자열 생성
  sprintf(jsonBuffer, 
    "{\"v\":%.1f,\"p1\":%.1f,\"i1\":%.2f,\"p2\":%.1f,\"i2\":%.2f,\"pt\":%.1f,\"thd\":%.1f,\"shed\":%s,\"fault\":%s}",
    V_rms,
    P1, I1,
    P2, I2,
    P_total,
    THD_val,
    lowPriorityShed ? "true" : "false",
    protectionTripped_Critical ? "true" : "false"
  );
  
  // SoftwareSerial (D2/D3)을 통해 전송
  wifiSerial.println(jsonBuffer);
  
  // PC 모니터에도 동일하게 출력
  Serial.print("WIFI_TX (D2): ");
  Serial.println(jsonBuffer);
}


/**
 * @brief 화면 하단에 공통 버튼 그리기
 */
void drawTouchButtons() {
  if (currentScreen == SCREEN_FAULT) return;
  tft.setTextSize(2);
  if (currentScreen == SCREEN_MAIN_POWER) {
    tft.fillRect(10, 210, 140, 30, ILI9341_GREEN);
    tft.setTextColor(ILI9341_BLACK);
  } else {
    tft.drawRect(10, 210, 140, 30, ILI9341_GREEN);
    tft.setTextColor(ILI9341_GREEN);
  }
  tft.setCursor(50, 217); tft.print("전력");

  if (currentScreen == SCREEN_WAVEFORM) {
    tft.fillRect(170, 210, 140, 30, ILI9341_CYAN);
    tft.setTextColor(ILI9341_BLACK);
  } else {
    tft.drawRect(170, 210, 140, 30, ILI9341_CYAN);
    tft.setTextColor(ILI9341_CYAN);
  }
  tft.setCursor(210, 217); tft.print("파형");
}


/**
 * @brief 메인 전력 값 화면 표시
 */
void displayMainScreen(float V_rms, float I_rms[], float P_real[], float Q_react[], float PF[], 
                         float P_total, float Q_total, float S_total, float PF_total, float THD) {
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  // 1. 공통 전압
  tft.setCursor(10, 30); tft.print("전압:");
  tft.setTextColor(ILI9341_CYAN); tft.print(V_rms, 2); tft.println(" V");

  // 2. 총계
  tft.setCursor(160, 30); tft.setTextColor(ILI9341_WHITE); tft.print("총전력:");
  tft.setTextColor(ILI9341_GREEN); tft.print(P_total, 2); tft.println(" W");
  tft.setCursor(160, 55); tft.setTextColor(ILI9341_WHITE); tft.print("총피상:");
  tft.print(S_total, 2); tft.println(" VA");
  tft.setCursor(10, 55); tft.setTextColor(ILI9341_WHITE); tft.print("총역률:");
  tft.setTextColor(ILI9341_RED); tft.println(PF_total, 3);
  tft.setCursor(10, 80); tft.setTextColor(ILI9341_WHITE); tft.print("THD:");
  tft.setTextColor(ILI9341_ORANGE); tft.print(THD, 2); tft.println(" %");

  // 3. 채널 1 (A4) - 고우선순위
  tft.drawFastHLine(10, 110, 300, ILI9341_DARKGREY);
  tft.setCursor(10, 120); tft.setTextColor(ILI9341_WHITE); tft.print("Ch1(고) I:");
  tft.setTextColor(ILI9341_YELLOW); tft.print(I_rms[0], 3); tft.println(" A");
  tft.setCursor(160, 120); tft.setTextColor(ILI9341_WHITE); tft.print("P:");
  tft.setTextColor(ILI9341_YELLOW); tft.print(P_real[0], 2); tft.println(" W");

  // 4. 채널 2 (A5) - 저우선순위
  tft.drawFastHLine(10, 150, 300, ILI9341_DARKGREY);
  tft.setCursor(10, 160); tft.setTextColor(ILI9341_WHITE); tft.print("Ch2(저) I:");
  
  if(lowPriorityShed) { // 저우선순위 부하가 차단되었다면
    tft.setTextColor(ILI9341_DARKGREY); 
    tft.print("--- (차단됨) ---");
  } else {
    tft.setTextColor(ILI9341_MAGENTA);
    tft.print(I_rms[1], 3); tft.println(" A");
    tft.setCursor(160, 160); tft.setTextColor(ILI9341_WHITE); tft.print("P:");
    tft.setTextColor(ILI9341_MAGENTA); tft.print(P_real[1], 2); tft.println(" W");
  }
}

/**
 * @brief 전압 파형 표시 화면
 */
void displayWaveformScreen() {
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(10, 10);
  tft.println("전압 파형 (FFT 버퍼)");
  drawWaveform(10, 40, 300, 160, vReal, FFT_SAMPLES, ILI9341_CYAN);
}

/**
 * @brief 치명적 오류(Fault) 화면 표시
 */
void displayCriticalFaultScreen(const char* message) {
  tft.fillScreen(ILI9341_RED);
  tft.setCursor(40, 100);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.println(message);
  tft.setTextSize(2);
  tft.setCursor(80, 140);
  tft.println("모든 릴레이 차단");
}


/**
 * @brief TFT에 파형을 그리는 헬퍼 함수
 */
void drawWaveform(int x, int y, int w, int h, double* data, int dataSize, uint16_t color) {
  tft.drawRect(x, y, w, h, ILI9341_WHITE); 
  double maxVal = -99999.0, minVal = 99999.0;
  for (int i = 0; i < dataSize; i++) {
    if (data[i] > maxVal) maxVal = data[i];
    if (data[i] < minVal) minVal = data[i];
  }
  if (maxVal < 1000) maxVal = 8192;
  if (minVal > -1000) minVal = -8192;
  double yRange = maxVal - minVal;
  if (yRange == 0) yRange = 1; 
  int lastX = -1, lastY = -1;
  for (int i = 0; i < dataSize; i++) {
    int px = x + (int)((float)i * (float)w / (float)dataSize);
    int py = y + (int)(h * (maxVal - data[i]) / yRange);
    if (lastX != -1) tft.drawLine(lastX, lastY, px, py, color);
    lastX = px; lastY = py;
  }
}


/**
 * @brief 퍼지 논리(단순 임계값) 기반 경고
 */
void checkFuzzyWarnings(float totalPower) {
  if (protectionTripped_Critical) return; // 치명적 오류 시 상단바 표시 안함 (전체 화면 붉음)
  
  if (totalPower > POWER_CRITICAL_W) {
    tft.fillRect(0, 0, 320, 25, ILI9341_RED); 
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(40, 5);
    tft.println("!! 위험 전력 초과 !!");
    Serial.println("!! 위험 전력 초과 !!");
  } else if (totalPower > POWER_WARNING_W) {
    tft.fillRect(0, 0, 320, 25, ILI9341_ORANGE); 
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(40, 5);
    tft.println("! 경고: 높은 전력 !");
    Serial.println("! 경고: 높은 전력 !");
  }
}


/**
 * @brief 우선순위 기반 부하 차단(Shedding) 및 관리
 */
void manageLoadPriorities(float P_total, float P_ch1) {
  
  if (protectionTripped_Critical) return;

  // --- 시나리오 2: 치명적 오류 (고우선순위 부하 단독 초과) ---
  if (P_ch1 > POWER_CRITICAL_W) {
    overload_counter++;
    if (overload_counter >= OVERLOAD_TRIP_CYCLES) {
      protectionTripped_Critical = true; // 되돌릴 수 없는 차단
      digitalWrite(RELAY_PIN_MAIN, LOW); 
      digitalWrite(RELAY_PIN_CH1, LOW);  
      digitalWrite(RELAY_PIN_CH2, LOW);  
      Serial.println("!! 치명적 오류: Ch 1 (고우선순위) 부하가 정격을 초과했습니다. 모든 릴레이 차단.");
      currentScreen = SCREEN_FAULT; 
    }
  } 
  // --- 시나리오 1: 경미한 오류 (총 부하 초과) ---
  else if (P_total > POWER_CRITICAL_W) {
    overload_counter++;
    if (overload_counter >= OVERLOAD_TRIP_CYCLES) {
      if (!lowPriorityShed) { 
        digitalWrite(RELAY_PIN_CH2, LOW); // 저우선순위(Ch2)만 차단
        lowPriorityShed = true;
        Serial.println("!! 부하 차단: 총 전력 초과. Ch 2 (저우선순위)를 차단합니다.");
      }
      overload_counter = 0; 
    }
  } 
  // --- 시나리오 3: 복구 (안전 임계값 미만) ---
  else if (P_total < POWER_RECOVERY_W) { // 1200W 미만 (Hysteresis)
    overload_counter = 0;
    if (lowPriorityShed) { 
      lowPriorityShed = false;
      digitalWrite(RELAY_PIN_CH2, HIGH); // 저우선순위(Ch2) 복구
      Serial.println("...전력 안전. Ch 2 (저우선순위)를 복구합니다.");
    }
  } 
  // --- 경고 상태 (WARNING ~ CRITICAL 사이) ---
  else {
    overload_counter = 0; 
  }
}


/**
 * @brief THD 계산 함수
 */
float calculate_THD() {
  if (fft_count < FFT_SAMPLES) return 0.0;
  
  SPI.beginTransaction(spiSettingsTFT);
  FFT.windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLES);
  SPI.endTransaction();
  
  int fundamental_idx = (int)(60.0 / (1000.0 / FFT_SAMPLES) + 0.5); 
  float fundamental_mag = vReal[fundamental_idx];
  float harmonic_sq_sum = 0.0;

  for (int i = 2; i < FFT_SAMPLES / 2; i++) { 
    if (i != fundamental_idx) {
      harmonic_sq_sum += vReal[i] * vReal[i];
    }
  }

  float THD = 0.0;
  if (fundamental_mag > 0.0) {
      THD = (sqrt(harmonic_sq_sum) / fundamental_mag) * 100.0;
  }
  return THD;
}