/*
 * ==============================================================================
 * Processor.ino (헤드리스 센서 및 로직 코어)
 *
 * 이 스케치는 Wattmeter 프로젝트의 '두뇌' 역할을 합니다.
 * 모든 센서(전압, 전류) 입력, FFT/THD 계산, 퍼지 로직, 릴레이 제어를 담당합니다.
 * 이 스케치에는 디스플레이(TFT) 또는 터치스크린 관련 코드가 없습니다.
 * * 통신:
 * - TX: 메인 데이터 패킷 (JSON) 또는 파형 데이터 스트림 (Text)을 Controller로 전송합니다.
 * - RX: Controller로부터 설정 변경, 릴레이 제어 등의 명령 (JSON)을 수신합니다.
 * ==============================================================================
 */
/*
 * ==============================================================================
 * v60 변경 사항 (2025-11-18):
 * 1. 릴레이 자동 차단 타이머 기능 추가.
 * - checkSerialCommands(): Controller로부터 "SET_TIMER" (JSON) 명령 수신 .
 * - loop(): relay_shutdown_time > 0 일 때 millis()를 확인하여 릴레이 자동 차단 .
 * - sendMainDataPacket(): "T_ACTIVE", "T_LEFT_S", "T_MIN" 타이머 상태를 Controller로 전송 .
 * 2. 안전 로직 추가:
 * - checkSerialCommands(): "RESET_WARNING" 수신 시, 경고뿐만 아니라 타이머(relay_shutdown_time)도 함께 리셋 .
 * ==============================================================================
 */
 
// --- 라이브러리 포함 ---
#include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>
#include <ArduinoJson.h> // 시리얼 통신용

// --- 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5
// #define WIFI_TX_PIN 2 // (향후 확장용)
// #define WIFI_RX_PIN 3 // (향후 확장용)

// --- 통신 상태 ---
volatile bool isStreamingWaveform = false;
unsigned long lastDataSendTime = 0;
const unsigned long MAIN_DATA_INTERVAL = 333; // 약 3Hz

// --- JSON 문서 (버퍼) ---
StaticJsonDocument<1024> txJsonDoc; // 전송용 (데이터가 많음)
StaticJsonDocument<256> rxJsonDoc;  // 수신용 (명령은 간단함)

// --- [v25] 기본값 정의 (초기화용)
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

// --- [v27] 설정 변수 (Processor가 마스터 사본을 가짐) ---
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
#define DEFAULT_SETTING_STEP_INDEX 3 // 1.0

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
int setting_step_index = DEFAULT_SETTING_STEP_INDEX; // [v23]

volatile bool warningActive = false;
String warningMessage = "";

// --- [신규] 타이머 관련 전역 변수 ---
unsigned long relay_shutdown_time = 0; // 0 = 비활성. 타이머가 만료되는 millis() 시간
uint16_t timer_setting_minutes_master = 0; // Controller와 동기화할 마스터 설정값 (분)

// --- FFT 상수 ---
#define FFT_N 512
#define SAMPLING_FREQ_HZ 7680.0f
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ)
#define FUNDALMENTAL_BIN 4
#define MAX_HARMONIC 40

// --- 캘리브레이션 상수 (ADC 오프셋) ---
const float V_ADC_MIDPOINT = 8192.0; 
const float I_ADC_MIDPOINT = 8192.0; 

#define SAMPLES_PER_CALC 334 
#define SAMPLE_PERIOD_US 1000
const float FREQUENCY = 60.0;

// --- 파형 화면 상수 (Controller와 동일해야 함) ---
#define PLOT_X_START 37 
#define PLOT_X_END 285 
#define PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define WAVEFORM_SAMPLE_PERIOD_US 100

// --- 전역 변수 (물리량) ---
float V_rms = 0.0;
float I_rms = 0.0; 
float I_rms_load1 = 0.0;
float I_rms_load2 = 0.0;
float P_real = 0.0; 
float Q_reactive = 0.0; 
float S_apparent = 0.0;
float PF = 0.0; 

// --- 전역 변수 (위상) ---
String lead_lag_status = "---"; 
float phase_main_deg = 0.0;
float phase_load1_deg = 0.0;
float phase_load2_deg = 0.0;

// --- FFT 버퍼 ---
float32_t v_samples[FFT_N];
float32_t i_samples[FFT_N];
float32_t v_fft_output[FFT_N];
float32_t i_fft_output[FFT_N];
float32_t v_mags[FFT_N / 2];
float32_t i_mags[FFT_N / 2];
float32_t thd_v_value = 0.0;
float32_t thd_i_value = 0.0;
arm_rfft_fast_instance_f32 fft_inst_v;
arm_rfft_fast_instance_f32 fft_inst_i;

// --- 퍼지 로직 변수 ---
Fuzzy *fuzzy; 
FuzzyInput *totalCurrent; 
FuzzyInput *currentChangeRate; 
FuzzyOutput *shutdownLevel; 
FuzzySet* safeCurrent;
FuzzySet* warningCurrent; 
FuzzySet* dangerCurrent; 
FuzzySet* criticalCurrent; 
FuzzySet* stableChange;
FuzzySet* slowIncrease;
FuzzySet* suddenSurge;
FuzzySet* level0;
FuzzySet* level3;
FuzzySet* level6;
FuzzySet* level10; 
float last_I_rms = 0.0;
unsigned long lastFuzzyTime = 0; 

// --- 함수 프로토타입 ---
void buildFuzzySystem();
void runFuzzyLogic(); 
void controlRelays(float level);
void waitForVoltageZeroCross();
float calculatePhase(long time_diff, float period_us);
void restoreDefaultSettings(); 
void performFFT_and_CalcTHD();
void calculatePowerMetrics();
void checkSerialCommands();
void sendMainDataPacket();
void runModifiedWaveformLoop();
float32_t calculateTHD(float32_t* mags, int fundamentalBin);


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Processor.ino Booting...");
  
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  pinMode(RELAY_1_PIN, OUTPUT); 
  pinMode(RELAY_2_PIN, OUTPUT); 
  digitalWrite(RELAY_1_PIN, LOW); 
  digitalWrite(RELAY_2_PIN, LOW); 

  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1); 
  currentChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  
  buildFuzzySystem(); 
  Serial.println("Fuzzy Logic System Built"); 
  lastFuzzyTime = millis(); 

  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N); 
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N); 
  Serial.println("CMSIS-DSP FFT Initialized.");
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  
  if (isStreamingWaveform) {
    // 파형 스트리밍 모드:
    // 이 함수는 Controller에서 중지 명령을 받을 때까지 
    // 내부에서 시리얼을 확인하며 파형 데이터를 전송합니다.
    runModifiedWaveformLoop();
  } else {
    // 일반 데이터 모드:

    // --- [신규] 릴레이 타이머 만료 확인 (요청 사항) ---
    // (경고 상태가 아닐 때만 타이머 검사)
    if (relay_shutdown_time > 0 && !warningActive && millis() >= relay_shutdown_time) {
      digitalWrite(RELAY_1_PIN, LOW);
      digitalWrite(RELAY_2_PIN, LOW);
      relay_shutdown_time = 0; // 타이머 비활성화
      timer_setting_minutes_master = 0; // 설정값 리셋
      
      // 사용자에게 알림 (선택 사항 -> 요청 사항이므로 구현)
      warningMessage = "TIMER SHUTDOWN";
      warningActive = true;
    }
    
    // 1. 비동기적으로 Controller의 명령을 확인합니다.
    checkSerialCommands();

    // 2. 주기적으로 (MAIN_DATA_INTERVAL) 계산을 수행하고 메인 데이터 패킷을 전송합니다.
    if (millis() - lastDataSendTime > MAIN_DATA_INTERVAL) {
      // 모든 계산 수행
      performFFT_and_CalcTHD();
      calculatePowerMetrics(); 
      // runFuzzyLogic()은 calculatePowerMetrics() 내부에서 호출됩니다.

      // 계산된 데이터를 JSON으로 전송
      sendMainDataPacket();
      lastDataSendTime = millis();
    }
  }
}

// ==============================================================================
// 3. 시리얼 명령 수신 및 처리 (Controller -> Processor)
// ==============================================================================
void checkSerialCommands() {
  if (Serial.available() == 0) return;

  String line = Serial.readStringUntil('\n');
  if (line.length() == 0) return;

  DeserializationError error = deserializeJson(rxJsonDoc, line);

  if (error) {
    Serial.print("P: deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  const char* cmd = rxJsonDoc["CMD"];
  if (cmd == NULL) return;

  // --- 명령 분기 ---
  if (strcmp(cmd, "SET_CALIB") == 0) {
    // V_MULTIPLIER 또는 I_MULTIPLIER가 JSON에 있는지 확인
    if (rxJsonDoc.containsKey("V_MULT")) {
      V_MULTIPLIER = rxJsonDoc["V_MULT"];
      if (V_MULTIPLIER < 0) V_MULTIPLIER = 0;
    }
    if (rxJsonDoc.containsKey("I_MULT")) {
      I_MULTIPLIER = rxJsonDoc["I_MULT"];
      if (I_MULTIPLIER < 0) I_MULTIPLIER = 0;
    }
  } 
  else if (strcmp(cmd, "SET_PROTECT") == 0) {
    if (rxJsonDoc.containsKey("V_THR")) {
      VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"];
      if (VOLTAGE_THRESHOLD < 0) VOLTAGE_THRESHOLD = 0;
    }
  }
  else if (strcmp(cmd, "SET_STEP") == 0) {
    if (rxJsonDoc.containsKey("IDX")) {
       setting_step_index = rxJsonDoc["IDX"];
    }
  }
  else if (strcmp(cmd, "SET_RELAY") == 0) {
    int id = rxJsonDoc["ID"];
    int state = rxJsonDoc["STATE"]; // 0:OFF, 1:ON, 2:TOGGLE
    int pin = (id == 1) ? RELAY_1_PIN : RELAY_2_PIN;

    if (state == 2) { // TOGGLE
      digitalWrite(pin, !digitalRead(pin));
    } else {
      digitalWrite(pin, state);
    }
  }
  else if (strcmp(cmd, "RESET_SETTINGS") == 0) {
    restoreDefaultSettings();
  }
  else if (strcmp(cmd, "REQ_WAVEFORM") == 0) {
    isStreamingWaveform = (rxJsonDoc["MODE"] == 1);
  }
  else if (strcmp(cmd, "RESET_WARNING") == 0) {
    warningActive = false;
    // 경고 해제 시 릴레이도 즉시 차단 (안전)
    digitalWrite(RELAY_1_PIN, LOW);
    digitalWrite(RELAY_2_PIN, LOW);
    // [신규] 경고 해제 시 타이머도 함께 리셋 (안전)
    relay_shutdown_time = 0;
    timer_setting_minutes_master = 0;
  }
  // --- [신규] 타이머 설정 명령 ---
  else if (strcmp(cmd, "SET_TIMER") == 0) {
    timer_setting_minutes_master = rxJsonDoc["MIN"] | 0; // 값이 없으면 0
    if (timer_setting_minutes_master > 0) {
      // 새 타이머 설정 (millis() + 분 * 60 * 1000)
      // 60000UL: unsigned long으로 강제 형변환하여 오버플로우 방지
      relay_shutdown_time = millis() + (timer_setting_minutes_master * 60000UL);
    } else {
      // 0분이거나 명령이 0이면 타이머 비활성화
      relay_shutdown_time = 0;
    }
  }
}

// ==============================================================================
// 4. 메인 데이터 패킷 전송 (Processor -> Controller)
// ==============================================================================
void sendMainDataPacket() {
  txJsonDoc.clear();
  txJsonDoc["TYPE"] = "DATA";
  txJsonDoc["V"] = V_rms;
  txJsonDoc["I"] = I_rms;
  txJsonDoc["I1"] = I_rms_load1;
  txJsonDoc["I2"] = I_rms_load2;
  txJsonDoc["P"] = P_real;
  txJsonDoc["Q"] = Q_reactive;
  txJsonDoc["S"] = S_apparent;
  txJsonDoc["PF"] = PF;
  txJsonDoc["PH_M"] = phase_main_deg;
  txJsonDoc["PH_1"] = phase_load1_deg;
  txJsonDoc["PH_2"] = phase_load2_deg;
  txJsonDoc["LL"] = lead_lag_status;
  txJsonDoc["THD_V"] = thd_v_value;
  txJsonDoc["THD_I"] = thd_i_value;
  
  // 현재 설정값 동기화
  txJsonDoc["V_MULT"] = V_MULTIPLIER;
  txJsonDoc["I_MULT"] = I_MULTIPLIER;
  txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD;
  txJsonDoc["STEP_IDX"] = setting_step_index;

  // 릴레이 및 경고 상태
  txJsonDoc["R1"] = digitalRead(RELAY_1_PIN);
  txJsonDoc["R2"] = digitalRead(RELAY_2_PIN);
  txJsonDoc["WARN"] = warningActive;
  if (warningActive) {
    txJsonDoc["MSG"] = warningMessage;
  }

  // --- [신규] 타이머 상태 전송 ---
  bool timer_is_active = (relay_shutdown_time > 0);
  txJsonDoc["T_ACTIVE"] = timer_is_active;
  // 타이머가 활성 상태이고, 경고가 아닐 때만 남은 시간 계산
  txJsonDoc["T_LEFT_S"] = (timer_is_active && !warningActive) ? ((relay_shutdown_time - millis()) / 1000) : 0;
  txJsonDoc["T_MIN"] = timer_setting_minutes_master; // 현재 설정된 마스터 값

  serializeJson(txJsonDoc, Serial);
  Serial.println(); // JSON 패킷의 끝을 알림
}

// ==============================================================================
// 5. 파형 스트리밍 루프 (Processor -> Controller)
// ==============================================================================
void runModifiedWaveformLoop() {
  // 파형 전송 시작 전 0점 동기화
  waitForVoltageZeroCross();
  
  // 원본 runCombinedWaveformLoop의 샘플링 로직 사용
  unsigned long startTime = micros();

  // [v27] 보정 계수와 배수를 곱하여 최종 계산 (V/I 배수 통합)
  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  for (int i = 0; i < PLOT_WIDTH; i++) {
    // --- 중요 ---
    // 고속 샘플링 루프 *내부*에서 시리얼 명령(중지)을 확인합니다.
    checkSerialCommands();
    if (!isStreamingWaveform) {
      return; // 중지 명령 수신 시 즉시 탈출
    }
    // ---

    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    
    // [v27] * effective 값 사용
    float V_mains_instant = V_ac_bits * effective_V_Calib + effective_V_Offset;
    float I_mains_instant = I_ac_bits * effective_I_Calib - effective_I_Offset;

    // JSON이 아닌 텍스트 스트림으로 고속 전송
    Serial.print(V_mains_instant);
    Serial.print(",");
    Serial.println(I_mains_instant);
    
    while(micros() - startTime < (i + 1) * WAVEFORM_SAMPLE_PERIOD_US);
  }
  // 한 프레임 전송 완료. loop()가 이 함수를 다시 호출할 것임.
}


// ==============================================================================
// 6. 60Hz 0점 통과(Zero-Crossing) 대기 함수 
// (원본 코드와 동일)
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
// 7. 실시간 전력 계산 (모든 위상차 계산 포함)
// (원본 코드와 동일, UI 관련 코드 제거)
// ==============================================================================
void calculatePowerMetrics() {
  
  unsigned long V_sq_sum = 0; 
  unsigned long I_sq_sum = 0; 
  unsigned long I_sq_sum_load1 = 0;
  unsigned long I_sq_sum_load2 = 0;
  long P_sum = 0; 

  int V_ac_bits_prev = 0; 
  int I_ac_bits_prev = 0; 
  int I1_ac_bits_prev = 0;
  int I2_ac_bits_prev = 0;
  long time_V_cross = -1, time_I_cross = -1, time_I1_cross = -1, time_I2_cross = -1;
  bool found_V_cross = false, found_I_cross = false, found_I1_cross = false, found_I2_cross = false; 

  unsigned long startTimePass = micros();
  
  // 퍼지 로직 실행 (원본과 동일)
  runFuzzyLogic(); 

  for (int i = 0; i < SAMPLES_PER_CALC; i++) { 
    int V_raw = analogRead(VOLTAGE_PIN); 
    int I_raw = analogRead(CURRENT_PIN); 
    int I_raw_load1 = analogRead(CURRENT_PIN_LOAD1);
    int I_raw_load2 = analogRead(CURRENT_PIN_LOAD2);
    
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    int I_ac_bits_load1 = I_raw_load1 - (int)I_ADC_MIDPOINT;
    int I_ac_bits_load2 = I_raw_load2 - (int)I_ADC_MIDPOINT;
    
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits; 
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits; 
    I_sq_sum_load1 += (unsigned long)I_ac_bits_load1 * I_ac_bits_load1;
    I_sq_sum_load2 += (unsigned long)I_ac_bits_load2 * I_ac_bits_load2;
    P_sum += (long)V_ac_bits * I_ac_bits; 
    
    if (i < (SAMPLE_PERIOD_US * 1000 / FREQUENCY / SAMPLE_PERIOD_US)) { 
      unsigned long crossTime = micros();
      if (!found_V_cross && V_ac_bits_prev < 0 && V_ac_bits >= 0) {
        time_V_cross = crossTime;
        found_V_cross = true; 
      }
      if (!found_I_cross && I_ac_bits_prev < 0 && I_ac_bits >= 0) {
        time_I_cross = crossTime;
        found_I_cross = true; 
      }
      if (!found_I1_cross && I1_ac_bits_prev < 0 && I_ac_bits_load1 >= 0) {
        time_I1_cross = crossTime;
        found_I1_cross = true; 
      }
      if (!found_I2_cross && I2_ac_bits_prev < 0 && I_ac_bits_load2 >= 0) {
        time_I2_cross = crossTime;
        found_I2_cross = true; 
      }
    }
    V_ac_bits_prev = V_ac_bits; 
    I_ac_bits_prev = I_ac_bits; 
    I1_ac_bits_prev = I_ac_bits_load1;
    I2_ac_bits_prev = I_ac_bits_load2;
    
    while(micros() - startTimePass < (i + 1) * SAMPLE_PERIOD_US); 
  }

  float V_rms_adc = sqrt((float)V_sq_sum / SAMPLES_PER_CALC);
  float I_rms_adc = sqrt((float)I_sq_sum / SAMPLES_PER_CALC); 
  float I_rms_adc_load1 = sqrt((float)I_sq_sum_load1 / SAMPLES_PER_CALC);
  float I_rms_adc_load2 = sqrt((float)I_sq_sum_load2 / SAMPLES_PER_CALC);
  float P_avg_adc = (float)P_sum / SAMPLES_PER_CALC;

  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  V_rms = (V_rms_adc * effective_V_Calib) - effective_V_Offset;
  I_rms = (I_rms_adc * effective_I_Calib) - effective_I_Offset; 
  I_rms_load1 = (I_rms_adc_load1 * effective_I_Calib) - effective_I_Offset;
  I_rms_load2 = (I_rms_adc_load2 * effective_I_Calib) - effective_I_Offset;
  P_real = P_avg_adc * effective_V_Calib * effective_I_Calib; 
  
  if (V_rms < 0) V_rms = 0; 
  if (I_rms < 0) I_rms = 0; 
  if (I_rms_load1 < 0) I_rms_load1 = 0;
  if (I_rms_load2 < 0) I_rms_load2 = 0;

  S_apparent = V_rms * I_rms; 
  
  if (S_apparent < 0.01) { 
    PF = 0.0; P_real = 0.0; Q_reactive = 0.0; 
  } else {
    PF = P_real / S_apparent; 
    if (PF > 1.0) PF = 1.0; 
    if (PF < -1.0) PF = -1.0; 
    Q_reactive = sqrt(max(0.0, S_apparent * S_apparent - P_real * P_real)); 
  }

  float period_us = 1000000.0 / FREQUENCY; 
  phase_main_deg = 0.0;
  phase_load1_deg = 0.0;
  phase_load2_deg = 0.0;
  
  if (found_V_cross) {
    if (found_I_cross) phase_main_deg = calculatePhase(time_I_cross - time_V_cross, period_us);
    if (found_I1_cross) phase_load1_deg = calculatePhase(time_I1_cross - time_V_cross, period_us);
    if (found_I2_cross) phase_load2_deg = calculatePhase(time_I2_cross - time_V_cross, period_us);
  }

  // phase_degrees는 Controller에서 PF로 계산하므로 제거
  if (phase_main_deg < -2.0) lead_lag_status = "Lead";
  else if (phase_main_deg > 2.0) lead_lag_status = "Lag";
  else lead_lag_status = "---";

  if (I_rms < 0.05) {
     phase_main_deg = 0.0;
     lead_lag_status = "---"; 
     PF = (V_rms > 10.0) ? 1.0 : 0.0;
     P_real = 0.0; 
     Q_reactive = 0.0; 
  }
  if (I_rms_load1 < 0.05) { I_rms_load1 = 0.0; phase_load1_deg = 0.0; }
  if (I_rms_load2 < 0.05) { I_rms_load2 = 0.0; phase_load2_deg = 0.0; }

  // 과전압 보호 로직 (원본과 동일)
  if (V_rms > VOLTAGE_THRESHOLD && !warningActive) { // 경고가 활성화되지 않았을 때만 트리거
    digitalWrite(RELAY_1_PIN, HIGH);
    digitalWrite(RELAY_2_PIN, HIGH); 
    warningMessage = "OVER VOLTAGE!"; 
    warningActive = true;
    // screenNeedsRedraw는 Controller의 영역이므로 제거
  }
}

// ==============================================================================
// 8. [v19] 위상 계산 헬퍼 함수
// (원본 코드와 동일)
// ==============================================================================
float calculatePhase(long time_diff, float period_us) {
  float phase = fmod(((float)time_diff / period_us) * 360.0, 360.0);
  if (phase > 180.0) phase -= 360.0;
  else if (phase < -180.0) phase += 360.0;
  return phase;
}

// ==============================================================================
// 9. 퍼지 로직 시스템 빌드 함수
// (원본 코드와 동일)
// ==============================================================================
void buildFuzzySystem() {
  safeCurrent = new FuzzySet(0, 0, 5, 6); 
  warningCurrent = new FuzzySet(5, 6, 6, 7); 
  dangerCurrent = new FuzzySet(6.5, 7.25, 7.25, 8); 
  criticalCurrent = new FuzzySet(7.5, 8, 10, 10); 
  totalCurrent->addFuzzySet(safeCurrent);
  totalCurrent->addFuzzySet(warningCurrent); 
  totalCurrent->addFuzzySet(dangerCurrent);
  totalCurrent->addFuzzySet(criticalCurrent);
  fuzzy->addFuzzyInput(totalCurrent); 

  stableChange = new FuzzySet(-2, -1, 1, 2);
  slowIncrease = new FuzzySet(1, 3, 3, 5);
  suddenSurge = new FuzzySet(4, 6, 10, 10); 
  currentChangeRate->addFuzzySet(stableChange);
  currentChangeRate->addFuzzySet(slowIncrease);
  currentChangeRate->addFuzzySet(suddenSurge);
  fuzzy->addFuzzyInput(currentChangeRate); 

  level0 = new FuzzySet(0, 0, 0, 1);
  level3 = new FuzzySet(2, 3, 3, 4);
  level6 = new FuzzySet(5, 6, 6, 7);
  level10 = new FuzzySet(9, 10, 10, 10);
  shutdownLevel->addFuzzySet(level0);
  shutdownLevel->addFuzzySet(level3);
  shutdownLevel->addFuzzySet(level6);
  shutdownLevel->addFuzzySet(level10);
  fuzzy->addFuzzyOutput(shutdownLevel); 

  FuzzyRuleAntecedent* if_Safe_and_Stable = new FuzzyRuleAntecedent();
  if_Safe_and_Stable->joinWithAND(safeCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level0 = new FuzzyRuleConsequent();
  then_Level0->addOutput(level0);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_Safe_and_Stable, then_Level0)); 

  FuzzyRuleAntecedent* if_Warning_and_Stable = new FuzzyRuleAntecedent();
  if_Warning_and_Stable->joinWithAND(warningCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level3 = new FuzzyRuleConsequent();
  then_Level3->addOutput(level3); 
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_Warning_and_Stable, then_Level3)); 

  FuzzyRuleAntecedent* if_Danger_and_Stable = new FuzzyRuleAntecedent();
  if_Danger_and_Stable->joinWithAND(dangerCurrent, stableChange); 
  FuzzyRuleConsequent* then_Level6 = new FuzzyRuleConsequent();
  then_Level6->addOutput(level6); 
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_Danger_and_Stable, then_Level6)); 

  FuzzyRuleAntecedent* if_Warning_and_Slow = new FuzzyRuleAntecedent(); 
  if_Warning_and_Slow->joinWithAND(warningCurrent, slowIncrease); 
  fuzzy->addFuzzyRule(new FuzzyRule(4, if_Warning_and_Slow, then_Level6)); 
  
  FuzzyRuleAntecedent* if_Critical = new FuzzyRuleAntecedent();
  if_Critical->joinSingle(criticalCurrent);
  FuzzyRuleConsequent* then_Level10 = new FuzzyRuleConsequent();
  then_Level10->addOutput(level10); 
  fuzzy->addFuzzyRule(new FuzzyRule(5, if_Critical, then_Level10)); 

  FuzzyRuleAntecedent* if_Surge = new FuzzyRuleAntecedent();
  if_Surge->joinSingle(suddenSurge);
  fuzzy->addFuzzyRule(new FuzzyRule(6, if_Surge, then_Level10));
}


// ==============================================================================
// 10. 퍼지 로직 실행 헬퍼 함수
// (원본 코드와 동일)
// ==============================================================================
void runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  if (deltaTime < 0.1) return; 

  float dI_dt = (I_rms - last_I_rms) / deltaTime;
  last_I_rms = I_rms;
  lastFuzzyTime = currentTime; 

  fuzzy->setInput(1, I_rms); 
  fuzzy->setInput(2, dI_dt); 
  fuzzy->fuzzify(); 
  float outputLevel = fuzzy->defuzzify(1); 
  controlRelays(outputLevel); 
}


// ==============================================================================
// 11. 퍼지 출력 -> 릴레이 제어 변환 함수
// (원본 코드와 동일, UI 관련 코드 제거)
// ==============================================================================
void controlRelays(float level) {
  if (level > 9.0) {
    digitalWrite(RELAY_2_PIN, HIGH);
    digitalWrite(RELAY_1_PIN, HIGH);
    if (!warningActive) {
      warningMessage = "FUZZY LOGIC TRIP";
      warningActive = true; 
      // screenNeedsRedraw = true; // 제거
    }
  } 
  else if (level > 5.0) {
    digitalWrite(RELAY_2_PIN, HIGH);
    digitalWrite(RELAY_1_PIN, HIGH);
  } 
  else if (level > 2.0) {
    digitalWrite(RELAY_2_PIN, LOW);
    digitalWrite(RELAY_1_PIN, HIGH);
  } 
  else { 
    if (!warningActive) { // 경고 상태가 아닐 때만 LOW로 복귀
      digitalWrite(RELAY_2_PIN, LOW);
      digitalWrite(RELAY_1_PIN, LOW);
    }
  }
}

// ==============================================================================
// 12. CMSIS-DSP FFT 및 THD 계산 헬퍼 함수
// (원본 코드와 동일)
// ==============================================================================
float32_t calculateTHD(float32_t* mags, int fundamentalBin) {
  float32_t fundamental_power = mags[fundamentalBin] * mags[fundamentalBin];
  float32_t harmonics_power_sum = 0.0; 
  if (fundamental_power < 1e-10) return 0.0;

  for (int n = 2; n <= MAX_HARMONIC; n++) {
    int binIndex = fundamentalBin * n;
    if (binIndex >= (FFT_N / 2)) break;
    harmonics_power_sum += mags[binIndex] * mags[binIndex]; 
  }
  return sqrt(harmonics_power_sum / fundamental_power); 
}


// ==============================================================================
// 13. 메인 FFT/THD 연산 함수
// (원본 코드와 동일, 터치 확인 제거)
// ==============================================================================
void performFFT_and_CalcTHD() {
  unsigned long startTime = micros(); 
  
  for (int i = 0; i < FFT_N; i++) {
    v_samples[i] = (float32_t)(analogRead(VOLTAGE_PIN)) - V_ADC_MIDPOINT; 
    i_samples[i] = (float32_t)(analogRead(CURRENT_PIN)) - I_ADC_MIDPOINT;
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US); 
  }
  
  for (int i = 0; i < FFT_N; i++) {
    float32_t window_factor = 0.5f - 0.5f * arm_cos_f32(2.0f * PI * i / (FFT_N - 1));
    v_samples[i] = v_samples[i] * window_factor; 
    i_samples[i] = i_samples[i] * window_factor; 
  }
  
  arm_rfft_fast_f32(&fft_inst_v, v_samples, v_fft_output, 0); 
  arm_cmplx_mag_f32(v_fft_output, v_mags, FFT_N / 2); 
  arm_rfft_fast_f32(&fft_inst_i, i_samples, i_fft_output, 0); 
  arm_cmplx_mag_f32(i_fft_output, i_mags, FFT_N / 2); 
  
  thd_v_value = calculateTHD(v_mags, FUNDALMENTAL_BIN); 
  thd_i_value = calculateTHD(i_mags, FUNDALMENTAL_BIN); 
}

// ==============================================================================
// 14. 설정 초기화 헬퍼
// (원본 코드와 동일)
// ==============================================================================
void restoreDefaultSettings() {
  VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
  V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
  I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
  setting_step_index = DEFAULT_SETTING_STEP_INDEX;
}