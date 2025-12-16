/*
 * ==============================================================================
 * File: Wattmeter_MCU1_Processor.ino
 * Version: v108_THD_Fixed (Hybrid Logic: Code B base + Code A FFT Algo)
 * Target: Arduino R4 (Renesas)
 * Note: Requires 'arduinoFFT' library (v2.x)
 * ==============================================================================
 */

#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arduinoFFT.h> // Standard FFT Library
#include <ArduinoJson.h>

// --- Pin Definitions ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5

// --- Physics & Calibration ---
const float V_ADC_MIDPOINT = 8192.0;
const float I_ADC_MIDPOINT = 8192.0; 

#define FFT_N 256 
#define SAMPLING_FREQ_HZ 3840.0f 
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ)
#define FUNDALMENTAL_BIN 4 
#define MAX_HARMONIC 15 
#define NUM_HARMONICS_TO_SEND 15 

// [수정] 짝수 고조파 억제 계수 (Code A에서 가져옴)
#define EVEN_HARM_FACTOR 0.4f

const float FREQUENCY = 60.0;
const float CURRENT_CUTOFF_THRES = 0.07;

// Calibration Factors
float BASE_V_CALIB_RMS = 0.19277;
float BASE_I_CALIB_RMS = 0.003190;
float BASE_I_CALIB_RMS1 = 0.003150; 
float BASE_I_CALIB_RMS2 = 0.003117;

float BASE_V_OFFSET_ADJUST = 0.0; 
float BASE_I_OFFSET_ADJUST = 0.0;
float V_MULTIPLIER = 1.0;
float I_MULTIPLIER = 1.0;
float VOLTAGE_THRESHOLD = 240.0;

// --- Global Buffers ---
uint16_t raw_v_buf[FFT_N];
uint16_t raw_i_buf[FFT_N];
uint16_t raw_i1_buf[FFT_N];
uint16_t raw_i2_buf[FFT_N];

// --- Global Variables ---
double PF = 0.0, arccos=0.0;
float V_rms = 0.0, I_rms = 0.0, I_rms_load1 = 0.0, I_rms_load2 = 0.0;
float P_real = 0.0, Q_reactive = 0.0, S_apparent = 0.0, S_Physic=0.0;
float phase_main_deg = 0.0, phase_load1_deg = 0.0, phase_load2_deg = 0.0;
String lead_lag_status = "---";
float thd_v_value = 0.0, thd_i_value = 0.0;
bool warningActive = false;
String warningMessage = "";
String tripReason = "";
String tripRelay = "";  

// Relay & Timer
bool relay1_state = false;
bool relay2_state = false;
bool is_timer_active = false;
uint32_t timer_seconds_left = 0;
uint32_t timer_setting_seconds = 0; 
unsigned long lastTimerUpdate = 0;
unsigned long load1StartTime = 0; // Load 1 가동 시작 시간 기록용
unsigned long load2StartTime = 0; // Load 2 가동 시작 시간 기록용
double V_fund_accurate = 0.0;
double I_fund_accurate = 0.0;

// --- FFT Arrays (Double required for v2.x) ---
double vReal[FFT_N];
double vImag[FFT_N];
double iReal[FFT_N];
double iImag[FFT_N];

// FFT Objects
ArduinoFFT<double> FFT_V = ArduinoFFT<double>(vReal, vImag, FFT_N, SAMPLING_FREQ_HZ);
ArduinoFFT<double> FFT_I = ArduinoFFT<double>(iReal, iImag, FFT_N, SAMPLING_FREQ_HZ);

// --- Fuzzy Logic ---
Fuzzy *fuzzy;
FuzzyInput *realPowerInput, *powerChangeRate;
FuzzyOutput *shutdownLevel;

FuzzySet *safePower, *warningPower, *dangerPower, *criticalPower;
FuzzySet *stablePowerChange, *slowPowerIncrease, *suddenPowerSurge;
FuzzySet *level0, *level3, *level6, *level10;

float last_P_real = 0.0;
unsigned long lastFuzzyTime = 0;
float fuzzy_output_level = 0.0;
int fuzzyTripCounter = 0;
const int TRIP_DELAY_COUNT = 8;
unsigned long loadStartTime = 0; // [추가] 부하 가동 시작 시간 기록용

// --- Waveform Streaming ---
bool isWaveformStreaming = false;
int waveformType = 0; 
int waveformSyncMode = 0;
unsigned long lastDataSendTime = 0;

const unsigned long DATA_SEND_INTERVAL = 500; 

// --- Auto-Calibration ---
bool isAutoCalibrating = false;
float calibTargetV = 0.0;
float calibTargetI = 0.0;
double calibSumV = 0.0;
double calibSumI = 0.0;
int calibSampleCount = 0;
unsigned long calibStartTime = 0;

// JSON Docs
StaticJsonDocument<256> rxJsonDoc;
StaticJsonDocument<1024> txJsonDoc;

// --- Function Prototypes ---
void buildFuzzySystem();
float runFuzzyLogic();
void perform_unified_analysis();
void calculateNewGains(float true_v, float true_i);
void sendMainData();
void checkSerialCommand();
void runWaveformStreaming();
void waitForVoltageZeroCross();
// [삭제] calculateTHD_FFT 함수 제거됨 (perform_unified_analysis 내부로 통합)

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial1.begin(115200);
  analogReadResolution(14);
  
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);
  pinMode(RELAY_1_PIN, OUTPUT); 
  pinMode(RELAY_2_PIN, OUTPUT); 
  digitalWrite(RELAY_1_PIN, LOW); 
  digitalWrite(RELAY_2_PIN, LOW);

  // Initialize Fuzzy Logic
  fuzzy = new Fuzzy();
  realPowerInput = new FuzzyInput(1); 
  powerChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  buildFuzzySystem();
  
  lastFuzzyTime = millis();
}

// ================================================================
// LOOP (수정됨: 레벨별 차등 차단 속도 적용)
// ================================================================
void loop() {
  checkSerialCommand();

  // -----------------------------------------------------------
  // 1. Timer Logic
  // -----------------------------------------------------------
  if (is_timer_active) {
    if (millis() - lastTimerUpdate >= 1000) {
      lastTimerUpdate = millis();
      if (timer_seconds_left > 0) {
        timer_seconds_left--;
      } else {
        is_timer_active = false;
        relay1_state = false;
        relay2_state = false;
        digitalWrite(RELAY_1_PIN, HIGH);
        digitalWrite(RELAY_2_PIN, HIGH);
        warningMessage = "TIMER FINISHED";
        warningActive = true;
      }
    }
  }

  // -----------------------------------------------------------
  // 2. Main Operation
  // -----------------------------------------------------------
  if (isWaveformStreaming) {
    runWaveformStreaming();
  } else {
    // 0.5초(500ms)마다 데이터 전송 및 보호 로직 수행
    if (millis() - lastDataSendTime >= 500) {
       lastDataSendTime = millis();
       perform_unified_analysis();
       
       // 퍼지 로직 실행 (결과: 0.0 ~ 10.0)
       float fuzzyLevel = runFuzzyLogic();

       // ========================================================
       // [보호 로직: Load 1, 2 개별 돌입전류 감지]
       // ========================================================
       
       // 1. Load 1 가동 시간 체크
       if (I_rms_load1 < 0.5) load1StartTime = 0; // 꺼짐 -> 리셋
       else if (load1StartTime == 0) load1StartTime = millis(); // 켜진 순간 기록

       // 2. Load 2 가동 시간 체크
       if (I_rms_load2 < 0.5) load2StartTime = 0; // 꺼짐 -> 리셋
       else if (load2StartTime == 0) load2StartTime = millis(); // 켜진 순간 기록

       // 3. 돌입 전류 구간 판별 (각각 2초 이내인지 확인)
       bool inrush1 = (load1StartTime != 0) && (millis() - load1StartTime < 2000);
       bool inrush2 = (load2StartTime != 0) && (millis() - load2StartTime < 2000);
       
       // 둘 중 하나라도 돌입 전류 구간이면 '보호 완화 모드' 진입
       bool isAnyInrush = inrush1 || inrush2;

       // ========================================================
       // [수정됨] 4. 퍼지 로직 위험 카운터 (반한시 특성 적용)
       // TRIP_DELAY_COUNT = 8 (약 4초 기준)
       // ========================================================
       if (fuzzyLevel > 8.0) {
           // [Level 10] 치명적: 매우 빠르게 카운트 증가
           // 8 / 4 = 2회 루프 (약 1초) 만에 차단
           fuzzyTripCounter += 4; 
       } 
       else if (fuzzyLevel > 5.0) {
           // [Level 6] 위험: 빠르게 카운트 증가
           // 8 / 2 = 4회 루프 (약 2초) 만에 차단
           fuzzyTripCounter += 2;
       } 
       else if (fuzzyLevel > 2.0) {
           // [Level 3] 주의: 일반 속도
           // 8 / 1 = 8회 루프 (약 4초) 만에 차단
           fuzzyTripCounter += 1;
       } 
       else {
           // [안전] 카운터 감소 (서서히 식힘, Cooling logic)
           // 즉시 0으로 만들지 않고 서서히 줄여서, 
           // 과부하가 반복되면 더 빨리 차단되도록 메모리 효과 부여
           if (fuzzyTripCounter > 0) fuzzyTripCounter--;
       }

       // 카운터가 기준치(8)를 넘으면 트립 신호 발생
       bool fuzzy_trip = (fuzzyTripCounter >= TRIP_DELAY_COUNT);


       // ========================================================
       // 5. 과전류 및 쇼트 판별 로직
       // ========================================================
       bool overvoltage_trip = (V_rms > VOLTAGE_THRESHOLD);
       bool overcurrent_trip = (I_rms > 7.0);  // 일반 과전류
       bool short_circuit    = (I_rms > 15.0); // 쇼트(단락) 기준
       
       // [핵심] 돌입 전류 구간인 경우 보호 로직 완화 (면제권)
       if (isAnyInrush) {
           // 돌입 구간에서는 하드웨어 한계(30A)가 넘지 않는 한 봐줌
           if (I_rms < 30.0) {
               overcurrent_trip = false;
               short_circuit = false;
               fuzzy_trip = false; // ★ 퍼지가 위험하다고 해도 기동 초기는 무시
               
               // (선택사항) 기동 중에는 카운터가 쌓이지 않게 하려면 아래 주석 해제
               // fuzzyTripCounter = 0; 
           } else {
               // 30A 넘으면 즉시 차단 (진짜 쇼트)
               short_circuit = true; 
           }
       }
       
       // 쇼트 감지 시 즉시 트립 (퍼지 카운터고 뭐고 바로 차단)
       if (short_circuit) fuzzy_trip = true;


       // ========================================================
       // 6. 최종 트립 실행 (레벨별 동작 차별화 적용)
       // ========================================================
       bool master_trip = fuzzy_trip || overvoltage_trip || overcurrent_trip;
       
       if (master_trip) {
          // [메시지 설정]
          if (overvoltage_trip) tripReason = "OVER V";
          else if (short_circuit) tripReason = "SHORT CIRC"; 
          else if (overcurrent_trip) tripReason = "OVER I";
          else if (fuzzy_trip) tripReason = "OVER P (Fuzzy)";
          
          warningActive = true;
          fuzzyTripCounter = 0; // 리셋

          // -------------------------------------------------------
          // ★ [핵심 수정] 위험도에 따른 차단 범위 결정
          // -------------------------------------------------------
          
          // [상황 A] 치명적인 위험 (Level 10 이상 또는 쇼트) -> "전체 차단"
          if (fuzzyLevel > 8.0 || short_circuit || overvoltage_trip) {
              relay1_state = true; // OFF
              relay2_state = true; // OFF
              digitalWrite(RELAY_1_PIN, HIGH);
              digitalWrite(RELAY_2_PIN, HIGH);
              
              warningMessage = "CRITICAL: ALL OFF";
              tripRelay = "ALL";
          }
          // [상황 B] 일반적인 과부하 (Level 3~6) -> "원인 제공자만 차단"
          else {
              if (I_rms_load1 > I_rms_load2) {
                 relay1_state = true;
                 digitalWrite(RELAY_1_PIN, HIGH);
                 warningMessage = "TRIP: LOAD 1";
                 tripRelay = "R1";
              } else if (I_rms_load2 > I_rms_load1) {
                 relay2_state = true;
                 digitalWrite(RELAY_2_PIN, HIGH);
                 warningMessage = "TRIP: LOAD 2";
                 tripRelay = "R2";
              } else {
                 // 둘이 똑같으면 중요도가 낮은 R2부터 끈다는 정책 (예시)
                 relay2_state = true; 
                 digitalWrite(RELAY_2_PIN, HIGH);
                 warningMessage = "TRIP: LOAD 2 (EQ)";
                 tripRelay = "R2"; 
              }
          }
          
          sendMainData(); // 차단 즉시 상태 전송

       } else {
          // 경고 해제
          if (warningMessage != "TIMER FINISHED") {
             warningActive = false;
          }
       }
       
       sendMainData();
    }
  }
}

// ================================================================
// Serial Command Processing
// ================================================================
void checkSerialCommand() {
  if (Serial1.available() > 0) {
    String line = Serial1.readStringUntil('\n');
    DeserializationError error = deserializeJson(rxJsonDoc, line);
    
    if (!error) {
      String cmd = rxJsonDoc["CMD"];
      if (cmd == "ACK_WARN") {
         warningActive = false;
      }
      else if (cmd == "REQ_DATA") {
         sendMainData();
      }
      
      if (cmd == "REQ_WAVEFORM") {
         int mode = rxJsonDoc["MODE"];
         if (mode == 1) {
            isWaveformStreaming = true;
            waveformSyncMode = rxJsonDoc["SYNC"];
         } else {
            isWaveformStreaming = false;
         }
      } 
      else if (cmd == "SET_WAVE_TYPE") { waveformType = rxJsonDoc["TYPE"];
      }
      else if (cmd == "SET_CALIB") {
         float v_mult = rxJsonDoc["V_MULT"];
         float i_mult = rxJsonDoc["I_MULT"];
         if (v_mult > 0) V_MULTIPLIER = v_mult;
         if (i_mult > 0) I_MULTIPLIER = i_mult;
      }
      else if (cmd == "SET_PROTECT") {
         float v_thr = rxJsonDoc["V_THR"];
         if (v_thr > 0) VOLTAGE_THRESHOLD = v_thr;
      }
      else if (cmd == "SET_RELAY") {
         int id = rxJsonDoc["ID"];
         if (id == 1) relay1_state = !relay1_state;
         if (id == 2) relay2_state = !relay2_state;
         digitalWrite(RELAY_1_PIN, relay1_state ? HIGH : LOW);
         digitalWrite(RELAY_2_PIN, relay2_state ? HIGH : LOW);
         sendMainData();
      }
      else if (cmd == "SET_TIMER") {
         timer_setting_seconds = rxJsonDoc["SEC"];
         if (timer_setting_seconds > 0) {
            timer_seconds_left = timer_setting_seconds;
            is_timer_active = true;
         } else {
            is_timer_active = false;
         }
         sendMainData();
      }
      else if (cmd == "RESET_WARNING") {
         warningActive = false;
         relay1_state = false;
         relay2_state = false;
         digitalWrite(RELAY_1_PIN, LOW);
         digitalWrite(RELAY_2_PIN, LOW);
      }
      else if (cmd == "RESET_SETTINGS") {
         V_MULTIPLIER = 1.0;
         I_MULTIPLIER = 1.0; VOLTAGE_THRESHOLD = 240.0;
      }
      else if (cmd == "CALIBRATE") {
         float true_v = rxJsonDoc["TRUE_V"];
         float true_i = rxJsonDoc["TRUE_I"];
         if (true_v > 0 && true_i > 0) calculateNewGains(true_v, true_i);
      }
    }
  }
}

// ================================================================
// Data Sending (JSON)
// ================================================================
void sendMainData() {
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
  
  txJsonDoc["V_MULT"] = V_MULTIPLIER;
  txJsonDoc["I_MULT"] = I_MULTIPLIER;
  txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD;
  
  txJsonDoc["R1"] = relay1_state;
  txJsonDoc["R2"] = relay2_state;
  txJsonDoc["T_ACTIVE"] = is_timer_active;
  txJsonDoc["T_LEFT_S"] = timer_seconds_left;
  txJsonDoc["T_SEC"] = timer_setting_seconds;

  txJsonDoc["WARN"] = warningActive;
  if (warningActive) {
    txJsonDoc["MSG"] = warningMessage;
    txJsonDoc["T_TYPE"] = tripReason; 
    txJsonDoc["T_RLY"] = tripRelay;
  }

  String harmonics_v_str = "";
  String harmonics_i_str = "";
  
  // [수정] 이미 Normalization이 끝난 vReal 값을 사용
  double v_fund = vReal[FUNDALMENTAL_BIN]; 
  double i_fund = iReal[FUNDALMENTAL_BIN];

  // k를 2씩 증가시켜 홀수 차수(1, 3, 5...)만 String으로 전송
  for (int k = 1; k <= MAX_HARMONIC; k += 2) { 

      if (k > 1) { 
        harmonics_v_str += ",";
        harmonics_i_str += ","; 
      }
      
      int bin = FUNDALMENTAL_BIN * k;
      if (bin >= FFT_N / 2) break;

      // 이미 perform_unified_analysis에서 complexToMagnitude & Normalization 완료됨
      // vImag는 0이므로 hypot 불필요, vReal 바로 사용
      double v_mag = vReal[bin];
      double i_mag = iReal[bin];

      if (v_fund > 0.01) {
         float pct = (v_mag / v_fund) * 100.0;
         harmonics_v_str += String(pct, 1);
      } else {
         harmonics_v_str += "0.0";
      }

      if (i_fund > 0.001 && I_rms > CURRENT_CUTOFF_THRES) {
         float pct = (i_mag / i_fund) * 100.0;
         harmonics_i_str += String(pct, 1);
      } else {
         harmonics_i_str += "0.0";
      }
  }
  
  txJsonDoc["H_V_STR"] = harmonics_v_str;
  txJsonDoc["H_I_STR"] = harmonics_i_str;

  serializeJson(txJsonDoc, Serial1);
  Serial1.println();
}

void perform_unified_analysis() {
  
  const int SHIFT_90_DEG = 16;
  
  // (Offset correction)
  const float PHASE_CAL_DEG = -20.0;
  const float PHASE_CAL_LOAD1 = -23.0;
  const float PHASE_CAL_LOAD2 = -23.0;

  // Code A와 동일한 FFT 미세 보정 스케일링 팩터
  const float FFT_SCALE_V = 1.244f;
  const float FFT_SCALE_I = 1.120f;
  


  waitForVoltageZeroCross();
  unsigned long startTime = micros();
  for (int i = 0; i < FFT_N; i++) {
    raw_v_buf[i] = analogRead(VOLTAGE_PIN);
    raw_i_buf[i] = analogRead(CURRENT_PIN);
    raw_i1_buf[i] = analogRead(CURRENT_PIN_LOAD1);
    raw_i2_buf[i] = analogRead(CURRENT_PIN_LOAD2);
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US);
  }

  long v_sum_raw = 0, i_sum_raw = 0;
  long i1_sum_raw = 0, i2_sum_raw = 0;
  for (int i = 0; i < FFT_N; i++) {
    v_sum_raw += raw_v_buf[i];
    i_sum_raw += raw_i_buf[i];
    i1_sum_raw += raw_i1_buf[i];
    i2_sum_raw += raw_i2_buf[i];
  }
  
  float v_offset = (float)v_sum_raw / FFT_N;
  float i_offset = (float)i_sum_raw / FFT_N;
  float i1_offset = (float)i1_sum_raw / FFT_N;
  float i2_offset = (float)i2_sum_raw / FFT_N;
  double sumV2 = 0, sumI2 = 0;
  double sumP_raw = 0, sumQ_raw = 0;
  
  float cal_v = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float cal_i = BASE_I_CALIB_RMS * I_MULTIPLIER;

  float v_inst_arr[FFT_N];
  float i_inst_arr[FFT_N];
  for (int i = 0; i < FFT_N; i++) {
    v_inst_arr[i] = ((float)raw_v_buf[i] - v_offset) * cal_v;
    i_inst_arr[i] = ((float)raw_i_buf[i] - i_offset) * cal_i;
    
    // RMS 누적
    sumV2 += v_inst_arr[i] * v_inst_arr[i];
    sumI2 += i_inst_arr[i] * i_inst_arr[i];

    // [중요] 유효전력(P)은 여전히 '전체 파형 적분' 사용 (요금 계산용)
    sumP_raw += v_inst_arr[i] * i_inst_arr[i];

    // Q용 90도 지연 데이터도 계산은 해두지만, 나중에 기본파 Q로 덮어쓸 예정
    int i_delayed_idx = i - SHIFT_90_DEG;
    if (i_delayed_idx < 0) i_delayed_idx += FFT_N;
    
    float i_90 = ((float)raw_i_buf[i_delayed_idx] - i_offset) * cal_i; 
    sumQ_raw += v_inst_arr[i] * i_90;
    
    // FFT용 데이터 복사 (Main V, I)
    vReal[i] = v_inst_arr[i]; vImag[i] = 0;
  }

  V_rms = sqrt(sumV2 / FFT_N);
  I_rms = sqrt(sumI2 / FFT_N);

  if (I_rms > CURRENT_CUTOFF_THRES && I_rms < 1.0) {
      
  }

  if (V_rms < 1.0) V_rms = 0;
  if (I_rms < CURRENT_CUTOFF_THRES) { 
      I_rms = 0;
      sumP_raw = 0; sumQ_raw = 0; 
  }

  // Load 1, 2 RMS 계산
  double sumI1_sq = 0, sumI2_sq = 0;
  for(int i=0; i<FFT_N; i++) {
      float val1 = ((float)raw_i1_buf[i] - i1_offset) * BASE_I_CALIB_RMS1 * I_MULTIPLIER;
      float val2 = ((float)raw_i2_buf[i] - i2_offset) * BASE_I_CALIB_RMS2 * I_MULTIPLIER;
      sumI1_sq += val1*val1; sumI2_sq += val2*val2;
  }
  I_rms_load1 = sqrt(sumI1_sq / FFT_N);
  I_rms_load2 = sqrt(sumI2_sq / FFT_N);
  if (I_rms_load1 < CURRENT_CUTOFF_THRES) I_rms_load1 = 0;
  if (I_rms_load2 < CURRENT_CUTOFF_THRES) I_rms_load2 = 0;

  S_Physic = V_rms * I_rms;
  
  // 기존 방식대로 P, Q 일단 계산 (P는 이걸 쓸 거임)
  double P_measured = sumP_raw / FFT_N;
  double Q_measured = sumQ_raw / FFT_N; 

  if (S_Physic > 0.1) {
      double angle_rad = PHASE_CAL_DEG * PI / 180.0;
      
      P_real = P_measured * cos(angle_rad) + Q_measured * sin(angle_rad);
      
      // ★ Q_reactive는 여기서 계산하지 않고, 아래쪽 FFT 파트에서 덮어씌웁니다.
      // Q_reactive = -P_measured * sin(angle_rad) + Q_measured * cos(angle_rad);
      
      phase_main_deg = 0.0;
      lead_lag_status = "---";
  } else {
      P_real = 0; Q_reactive = 0; PF = 0.0;
      phase_main_deg = 0;
      lead_lag_status = "---";
  }

  // =================================================================================
  // [STEP 1] 전압(Voltage) FFT 수행
  // =================================================================================
  FFT_V.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT_V.compute(FFTDirection::Forward);
  
  double ang_v = atan2(vImag[FUNDALMENTAL_BIN], vReal[FUNDALMENTAL_BIN]);

  FFT_V.complexToMagnitude(); 
  float norm_factor_v = (2.0 / FFT_N) * FFT_SCALE_V;
  for(int i=0; i < (FFT_N/2); i++) vReal[i] *= norm_factor_v;

  // Voltage Fundamental Calculation
  double v_sq_fund = sq(vReal[FUNDALMENTAL_BIN]) + sq(vReal[FUNDALMENTAL_BIN-1]) + sq(vReal[FUNDALMENTAL_BIN+1]);
  double v_fund = sqrt(v_sq_fund);
  
  // ★ [수정] 정확한 기본파 전압값(V)을 백업해둠 (THD 계산으로 변조되기 전에!)
  V_fund_accurate = v_fund; 

  // (이하 THD 계산 로직 동일)
  double sum_sq_v_harm = 0.0;
  for(int k=2; k<=MAX_HARMONIC; k++) {
     int bin = FUNDALMENTAL_BIN * k;
     if (bin >= FFT_N/2) break;
     double mag = vReal[bin];
     if (k % 2 == 0) { mag *= EVEN_HARM_FACTOR; vReal[bin] = mag; }
     sum_sq_v_harm += (mag * mag);
  }
  if (v_fund > 0.5) thd_v_value = (sqrt(sum_sq_v_harm) / v_fund) * 100.0;
  else thd_v_value = 0.0;

  // [전압] 고조파 크기(%) 변환
  float v_fund_mag_rms = vReal[FUNDALMENTAL_BIN]; 
  if (v_fund_mag_rms > 0.1) {
      vReal[FUNDALMENTAL_BIN] = 100.0; 
      for(int k=2; k<=MAX_HARMONIC; k++) {
          int bin = FUNDALMENTAL_BIN * k;
          if (bin >= FFT_N/2) break;
          vReal[bin] = (vReal[bin] / v_fund_mag_rms) * 100.0; 
      }
  } else {
      for(int i=FUNDALMENTAL_BIN; i<FFT_N/2; i++) vReal[i] = 0.0;
  }

  // =================================================================================
  // [STEP 2] 부하(Load 1, 2) FFT 수행 (생략 - 기존과 동일)
  // ...
  // =================================================================================
  
  // (이전 코드에서 Load 1, 2 Phase 계산 부분은 그대로 유지해주세요)
  if (I_rms_load1 > CURRENT_CUTOFF_THRES) {
      // ... (Load 1 FFT) ...
      // ... phase_load1_deg 계산 ...
  } else {
      phase_load1_deg = 0.0;
  }
  if (I_rms_load2 > CURRENT_CUTOFF_THRES) {
      // ... (Load 2 FFT) ...
      // ... phase_load2_deg 계산 ...
  } else {
      phase_load2_deg = 0.0;
  }

  // =================================================================================
  // [STEP 3] 메인 전류(Main Current) FFT 수행
  // =================================================================================
  if (I_rms > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = i_inst_arr[k]; 
         iImag[k] = 0;
      }

      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double diff = (ang_i - ang_v) * 180.0 / PI;
      phase_main_deg = diff - PHASE_CAL_DEG;
      if (phase_main_deg < -0.5) lead_lag_status = "Lag";
      else if (phase_main_deg > 0.5) lead_lag_status = "Lead";
      else lead_lag_status = "---";
      
      FFT_I.complexToMagnitude(); 
      float norm_factor_i = (2.0 / FFT_N) * FFT_SCALE_I;
      for(int i=0; i < (FFT_N/2); i++) iReal[i] *= norm_factor_i;
      
      float harm_noise_floor = 0.0; 
      double i_sq_fund = sq(iReal[FUNDALMENTAL_BIN]) + sq(iReal[FUNDALMENTAL_BIN-1]) + sq(iReal[FUNDALMENTAL_BIN+1]);
      double i_fund = sqrt(i_sq_fund);
      
      // ★ [수정] 정확한 기본파 전류값(I)을 백업해둠
      I_fund_accurate = i_fund;

      double sum_sq_i_harm = 0.0;
      
      for(int k=2; k<=MAX_HARMONIC; k++) {
          int bin = FUNDALMENTAL_BIN * k;
          if (bin >= FFT_N/2 - 1) break; 
          double mag_sq = sq(iReal[bin]) + sq(iReal[bin-1]) + sq(iReal[bin+1]);
          double mag = sqrt(mag_sq);

          if (mag > harm_noise_floor) {
              mag -= harm_noise_floor;
          } else {
              mag = 0.0;
          }
          iReal[bin] = mag; 
          sum_sq_i_harm += (mag * mag);
      }
      
      if (i_fund > 0.01) {
          thd_i_value = (sqrt(sum_sq_i_harm) / i_fund) * 100.0;
      } else {
          thd_i_value = 0.0;
      }

      float i_fund_display = i_fund; 
      if (i_fund_display > 0.01) {
          iReal[FUNDALMENTAL_BIN] = 100.0; 
          for(int k=2; k<=MAX_HARMONIC; k++) {
              int bin = FUNDALMENTAL_BIN * k;
              if (bin >= FFT_N/2) break;
              iReal[bin] = (iReal[bin] / i_fund_display) * 100.0; 
          }
      } else {
          for(int i=FUNDALMENTAL_BIN; i<FFT_N/2; i++) iReal[i] = 0.0;
      }

  } else {
      thd_i_value = 0.0;
      phase_main_deg = 0.0;
      I_fund_accurate = 0.0; // 전류 없으면 기본파 전류도 0
  }

  // =================================================================================
  // [STEP 4] ★ 최종 전력 재계산 (기본파 Q 적용)
  // =================================================================================
  if (S_Physic > 0.1) {
      // 1. 위상차를 라디안으로 변환
      double theta_rad = phase_main_deg * PI / 180.0;

     // 2차 고조파가 높으면(반파 정류 특징) 위상을 좀 더 당겨주는 로직
      double harm2_mag = iReal[FUNDALMENTAL_BIN * 2]; // 2고조파 크기 확인
      if (harm2_mag > 5.0) { // 2고조파가 5% 이상이면 (드라이기 약풍 판단)
         theta_rad -= (1.45 * PI / 180.0); // 강제로 3도 정도 더 빼줌
         }
      double Q_fund = V_fund_accurate * I_fund_accurate * sin(theta_rad);
      if (Q_fund < 0) Q_fund = -Q_fund; 

      // 3. 글로벌 변수 업데이트
      Q_reactive = Q_fund; 

      // 4. 피상전력(S)과 역률(PF)도 새로 맞춤 (Power Triangle 유지)
      S_apparent = hypot(P_real, Q_reactive);
      PF = P_real / S_apparent;

      if (PF > 1.0) PF = 1.0;
      if (PF < -1.0) PF = -1.0;

  } else {
      Q_reactive = 0.0;
      S_apparent = 0.0;
      PF = 0.0;
  }

  // 퍼지 로직 실행
  runFuzzyLogic();
}

 
// ================================================================
// Helpers
// ================================================================
void calculateNewGains(float true_v, float true_i) {
  perform_unified_analysis(); 
  float v_rms_raw_approx = V_rms / BASE_V_CALIB_RMS;
  float i_rms_raw_approx = I_rms / BASE_I_CALIB_RMS;

  if (v_rms_raw_approx > 1.0) BASE_V_CALIB_RMS = true_v / v_rms_raw_approx;
  if (i_rms_raw_approx > 1.0) BASE_I_CALIB_RMS = true_i / i_rms_raw_approx;

  V_MULTIPLIER = 1.0; I_MULTIPLIER = 1.0;
}

void waitForVoltageZeroCross() {
  unsigned long startTime = micros();
  const unsigned long TIMEOUT_US = 18000; 
  const int NOISE_THRESHOLD = 150;
  int midPoint = (int)V_ADC_MIDPOINT; 

  while (true) {
    if (micros() - startTime > TIMEOUT_US) return;
    int val = analogRead(VOLTAGE_PIN);
    if (val < (midPoint - NOISE_THRESHOLD)) break;
  }

  while (true) {
    if (micros() - startTime > TIMEOUT_US) return;
    int val = analogRead(VOLTAGE_PIN);
    if (val >= midPoint) return; 
  }
}

void runWaveformStreaming() {
  const int LAG_BUFFER_SIZE = 42; 
  float v_lag_buffer[LAG_BUFFER_SIZE];
  int buffer_head = 0;
  bool buffer_filled = false;

  if (waveformSyncMode == 1) {
    waitForVoltageZeroCross();
  }

  unsigned long startTime = micros();
  int sampleCount = 300; 

  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  for (int i = 0; i < sampleCount; i++) {
    if (i % 20 == 0 && Serial1.available()) {
       String s = Serial1.peek() == '{' ?
       Serial1.readStringUntil('\n') : "";
       if (s.indexOf("MODE\":0") != -1) {
          isWaveformStreaming = false;
          return;
       }
    }

    int V_raw = analogRead(VOLTAGE_PIN);
    int I_raw = analogRead(CURRENT_PIN);
    int I1_raw = analogRead(CURRENT_PIN_LOAD1);
    int I2_raw = analogRead(CURRENT_PIN_LOAD2);
    
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    int I1_ac_bits = I1_raw - (int)I_ADC_MIDPOINT;
    int I2_ac_bits = I2_raw - (int)I_ADC_MIDPOINT;

    float v_inst = V_ac_bits * effective_V_Calib + effective_V_Offset;
    float i_inst = I_ac_bits * effective_I_Calib - effective_I_Offset;
    float i1_inst = I1_ac_bits * effective_I_Calib - effective_I_Offset;
    float i2_inst = I2_ac_bits * effective_I_Calib - effective_I_Offset;

    v_lag_buffer[buffer_head] = v_inst;
    float v_lagged = v_lag_buffer[(buffer_head + 1) % LAG_BUFFER_SIZE];
    buffer_head = (buffer_head + 1) % LAG_BUFFER_SIZE;
    if (buffer_head == 0) buffer_filled = true;

    if (waveformType == 0) { 
       Serial1.print(v_inst); Serial1.print(","); Serial1.println(i_inst);
    } 
    else if (waveformType == 1) { 
       float p_inst = v_inst * i_inst;
       float q_inst = buffer_filled ? (v_lagged * i_inst) : 0.0;
       Serial1.print(p_inst); Serial1.print(","); Serial1.println(q_inst);
    } 
    else { 
       Serial1.print(i_inst); Serial1.print(","); 
       Serial1.print(i1_inst); Serial1.print(","); 
       Serial1.println(i2_inst);
    }

    while(micros() - startTime < (i + 1) * 100);
  }
}

// ================================================================
// Fuzzy Logic Functions
// ================================================================
void buildFuzzySystem() {
  safePower = new FuzzySet(0, 0, 1400, 1500);
  warningPower = new FuzzySet(1400, 1500, 1600, 1700);
  dangerPower = new FuzzySet(1600, 1700, 1900, 2000);
  criticalPower = new FuzzySet(1900, 2000, 2500, 2500);
  
  realPowerInput->addFuzzySet(safePower);
  realPowerInput->addFuzzySet(warningPower);
  realPowerInput->addFuzzySet(dangerPower);
  realPowerInput->addFuzzySet(criticalPower);
  fuzzy->addFuzzyInput(realPowerInput); 

  stablePowerChange = new FuzzySet(-200, -100, 100, 200);
  slowPowerIncrease = new FuzzySet(150, 200, 400, 500);
  suddenPowerSurge = new FuzzySet(700, 1000, 3000, 3000);
  
  powerChangeRate->addFuzzySet(stablePowerChange);
  powerChangeRate->addFuzzySet(slowPowerIncrease);
  powerChangeRate->addFuzzySet(suddenPowerSurge);
  fuzzy->addFuzzyInput(powerChangeRate);

  level0 = new FuzzySet(0, 0, 0, 1);
  level3 = new FuzzySet(2, 3, 3, 4);
  level6 = new FuzzySet(5, 6, 6, 7);
  level10 = new FuzzySet(9, 10, 10, 10);
  shutdownLevel->addFuzzySet(level0);
  shutdownLevel->addFuzzySet(level3);
  shutdownLevel->addFuzzySet(level6);
  shutdownLevel->addFuzzySet(level10);
  fuzzy->addFuzzyOutput(shutdownLevel);

  FuzzyRuleAntecedent* if_Safe = new FuzzyRuleAntecedent();
  if_Safe->joinSingle(safePower); 
  FuzzyRuleConsequent* then_Level0 = new FuzzyRuleConsequent();
  then_Level0->addOutput(level0);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_Safe, then_Level0));

  FuzzyRuleAntecedent* if_Warning_and_Stable = new FuzzyRuleAntecedent();
  if_Warning_and_Stable->joinWithAND(warningPower, stablePowerChange); 
  FuzzyRuleConsequent* then_Level3 = new FuzzyRuleConsequent();
  then_Level3->addOutput(level3);
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_Warning_and_Stable, then_Level3));

  FuzzyRuleAntecedent* if_Danger_and_Stable = new FuzzyRuleAntecedent();
  if_Danger_and_Stable->joinWithAND(dangerPower, stablePowerChange); 
  FuzzyRuleConsequent* then_Level6 = new FuzzyRuleConsequent();
  then_Level6->addOutput(level6);
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_Danger_and_Stable, then_Level6));

  FuzzyRuleAntecedent* if_Warning_and_Slow = new FuzzyRuleAntecedent(); 
  if_Warning_and_Slow->joinWithAND(warningPower, slowPowerIncrease);
  fuzzy->addFuzzyRule(new FuzzyRule(4, if_Warning_and_Slow, then_Level6)); 
  
  FuzzyRuleAntecedent* if_Critical = new FuzzyRuleAntecedent();
  if_Critical->joinSingle(criticalPower);
  FuzzyRuleConsequent* then_Level10 = new FuzzyRuleConsequent();
  then_Level10->addOutput(level10);
  fuzzy->addFuzzyRule(new FuzzyRule(5, if_Critical, then_Level10)); 

  FuzzyRuleAntecedent* if_Danger_and_Surge = new FuzzyRuleAntecedent();
  if_Danger_and_Surge->joinWithAND(dangerPower, suddenPowerSurge);
  fuzzy->addFuzzyRule(new FuzzyRule(6, if_Danger_and_Surge, then_Level10));
}

float runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  
  if (deltaTime >= 0.1) {
    float dP_dt = (P_real - last_P_real) / deltaTime;
    last_P_real = P_real;
    lastFuzzyTime = currentTime; 

    fuzzy->setInput(1, P_real); 
    fuzzy->setInput(2, dP_dt); 
    fuzzy->fuzzify(); 
    fuzzy_output_level = fuzzy->defuzzify(1); 
  }
  return fuzzy_output_level;
}
