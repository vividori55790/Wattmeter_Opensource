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
float BASE_V_CALIB_RMS = 0.19177;
float BASE_I_CALIB_RMS = 0.003190;
float BASE_I_CALIB_RMS1 = 0.003150; // Load 1 (기존 0.003058 -> 1.00A 측정됨)
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
// LOOP
// ================================================================
void loop() {
  checkSerialCommand();

  // Timer Logic
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

  // Main Operation
  if (isWaveformStreaming) {
    runWaveformStreaming();
  } else {
    // 0.5초(500ms)마다 데이터 전송 및 보호 로직 수행
    // 주의: runFuzzyLogic 내부는 0.1초마다 갱신되므로, 
    // 여기 loop 주기가 500ms라면 카운터가 느리게 올라갈 수 있습니다.
    // 하지만 현재 구조상 perform_unified_analysis가 시간을 잡아먹으므로 
    // 500ms 주기 안에서 퍼지 판단을 수행합니다.
    if (millis() - lastDataSendTime >= 500) {
       lastDataSendTime = millis();
       perform_unified_analysis();
       
       // 퍼지 로직 실행 (내부적으로 0.1초 경과 체크함)
       float fuzzyLevel = runFuzzyLogic();

       // [수정된 트립 판단 로직: 카운터 적용] -----------------------
       
       // 1. 위험 감지 시 카운트 증가 vs 정상 시 리셋
       if (fuzzyLevel > 2.0) {
           fuzzyTripCounter++;
       } else {
           fuzzyTripCounter = 0; // 정상이면 즉시 리셋 (돌입전류였다면 여기서 0됨)
       }

       // 2. 카운터가 설정값(4회)을 넘어야 진짜 트립으로 인정
       // (참고: loop 주기가 500ms라면 4회는 2초가 됩니다. 
       //  빠른 반응을 원하시면 TRIP_DELAY_COUNT를 1~2로 줄이거나 loop 주기를 당겨야 합니다.)
       bool fuzzy_trip = (fuzzyTripCounter >= TRIP_DELAY_COUNT);

       // 3. [안전장치] 단, 전류가 너무 크면(15A) 카운터 무시하고 즉시 차단 (Short Circuit)
       if (I_rms > 15.0) fuzzy_trip = true; 
       // --------------------------------------------------------

       bool overvoltage_trip = (V_rms > VOLTAGE_THRESHOLD);
       bool overcurrent_trip = (I_rms > 7.0); // 지속 과전류(7A)

       bool master_trip = fuzzy_trip || overvoltage_trip || overcurrent_trip;
       
       if (master_trip) {
          if (overvoltage_trip) tripReason = "OVER V";
          else if (I_rms > 15.0) tripReason = "SHORT CIRC"; // 쇼트 감지 표시
          else if (overcurrent_trip) tripReason = "OVER I";
          else if (fuzzy_trip) tripReason = "OVER P (Fuzzy)";
          
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
             relay2_state = true; 
             digitalWrite(RELAY_2_PIN, HIGH);
             warningMessage = "TRIP: LOAD 2 (EQ)";
             tripRelay = "R2"; 
          }
          warningActive = true;
          
          // 트립 발생 시 카운터 리셋 (재가동 준비)
          fuzzyTripCounter = 0; 

       } else {
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
  const float FFT_SCALE_V = 1.125f;
  const float FFT_SCALE_I = 1.31f;

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

    sumP_raw += v_inst_arr[i] * i_inst_arr[i];

    int i_delayed_idx = i - SHIFT_90_DEG;
    if (i_delayed_idx < 0) i_delayed_idx += FFT_N;
    
    float i_90 = ((float)raw_i_buf[i_delayed_idx] - i_offset) * cal_i; 
    sumQ_raw += v_inst_arr[i] * i_90;
    
    // FFT용 데이터 복사 (Main V, I)
    vReal[i] = v_inst_arr[i]; vImag[i] = 0;
    // iReal은 나중에 채웁니다 (순서 변경됨)
  }

  V_rms = sqrt(sumV2 / FFT_N);
  I_rms = sqrt(sumI2 / FFT_N);

  // =========================================================================
  // [보정] 저전류(1A 미만) 구간 노이즈 강제 차감 로직 (Soft Noise Gate)
  // 이유: 1A 이상은 정확하나, 저전류에서만 약 0.06~0.07A 높게 측정됨
  // 해결: 1A 미만일 때만 오차만큼 빼줌.
  // =========================================================================
  if (I_rms > CURRENT_CUTOFF_THRES && I_rms < 1.0) {
      // float low_current_noise = 0.01; // (측정값 0.27 - 기준값 0.205)
      // if (I_rms > low_current_noise) {
      //     I_rms -= low_current_noise;
      // }
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
  
  double P_measured = sumP_raw / FFT_N;
  double Q_measured = sumQ_raw / FFT_N; 

  if (S_Physic > 0.1) {
      double angle_rad = PHASE_CAL_DEG * PI / 180.0;
      P_real = P_measured * cos(angle_rad) + Q_measured * sin(angle_rad);
      Q_reactive = -P_measured * sin(angle_rad) + Q_measured * cos(angle_rad);
      if (Q_reactive < 0) Q_reactive = -Q_reactive;
      S_apparent = hypot(P_real, Q_reactive);
      PF = P_real / S_apparent;
      
      if (PF > 1.0) PF = 1.0;
      if (PF < -1.0) PF = -1.0;
      
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
  
  // 전압 위상 저장
  double ang_v = atan2(vImag[FUNDALMENTAL_BIN], vReal[FUNDALMENTAL_BIN]);

  FFT_V.complexToMagnitude(); 
  float norm_factor_v = (2.0 / FFT_N) * FFT_SCALE_V;
  for(int i=0; i < (FFT_N/2); i++) vReal[i] *= norm_factor_v;

  // Voltage THD Calc
  double v_sq_fund = sq(vReal[FUNDALMENTAL_BIN]) + sq(vReal[FUNDALMENTAL_BIN-1]) + sq(vReal[FUNDALMENTAL_BIN+1]);
  double v_fund = sqrt(v_sq_fund);
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

  // [전압] 고조파 크기(%) 변환 (RMS -> %)
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
  // [STEP 2] 부하(Load 1, 2) FFT 수행 (iReal 버퍼 임시 사용)
  // =================================================================================
  
  // [Load 1] Phase Calculation
  if (I_rms_load1 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = ((float)raw_i1_buf[k] - i1_offset) * BASE_I_CALIB_RMS1 * I_MULTIPLIER;
         iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i1 = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double diff1 = (ang_i1 - ang_v) * 180.0 / PI;
      while(diff1 > 180.0) diff1 -= 360.0;
      while(diff1 < -180.0) diff1 += 360.0;
      phase_load1_deg = diff1 - PHASE_CAL_LOAD1;
  } else {
      phase_load1_deg = 0.0;
  }

  // [Load 2] Phase Calculation
  if (I_rms_load2 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = ((float)raw_i2_buf[k] - i2_offset) * BASE_I_CALIB_RMS2 * I_MULTIPLIER;
         iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i2 = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double diff2 = (ang_i2 - ang_v) * 180.0 / PI;
      while(diff2 > 180.0) diff2 -= 360.0;
      while(diff2 < -180.0) diff2 += 360.0;
      phase_load2_deg = diff2 - PHASE_CAL_LOAD2;
  } else {
      phase_load2_deg = 0.0;
  }

// ... (앞부분 동일) ...

  // =================================================================================
  // [STEP 3] 메인 전류(Main Current) FFT 수행
  // =================================================================================
  if (I_rms > CURRENT_CUTOFF_THRES) {
      // 메인 전류 데이터 다시 로드
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = i_inst_arr[k]; 
         iImag[k] = 0;
      }

      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      // ... (위상 계산 부분 동일) ...
      double ang_i = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double diff = (ang_i - ang_v) * 180.0 / PI;
      phase_main_deg = diff - PHASE_CAL_DEG;
      if (phase_main_deg < -0.5) lead_lag_status = "Lag";
      else if (phase_main_deg > 0.5) lead_lag_status = "Lead";
      else lead_lag_status = "---";
      
      // Magnitude 변환 및 정규화
      FFT_I.complexToMagnitude(); 
      float norm_factor_i = (2.0 / FFT_N) * FFT_SCALE_I;
      for(int i=0; i < (FFT_N/2); i++) iReal[i] *= norm_factor_i;
      
      // [수정 1] 노이즈 기준을 0.05A -> 0.005A (5mA)로 대폭 완화
      // 작은 고조파 신호를 살리기 위함입니다.
      float harm_noise_floor = 0.0; 
      
      // THD 계산 및 고조파 크기 보정 루프
      double i_sq_fund = sq(iReal[FUNDALMENTAL_BIN]) + sq(iReal[FUNDALMENTAL_BIN-1]) + sq(iReal[FUNDALMENTAL_BIN+1]);
      double i_fund = sqrt(i_sq_fund);
      double sum_sq_i_harm = 0.0;
      
      // [수정 2] 고조파 에너지 합산 방식 개선 (Leakage 보정)
      for(int k=2; k<=MAX_HARMONIC; k++) {
         int bin = FUNDALMENTAL_BIN * k;
         if (bin >= FFT_N/2 - 1) break; // 배열 범위 안전장치
         
         // 중심 주파수(bin)와 양옆(bin-1, bin+1)의 에너지를 모아서 계산
         double mag_sq = sq(iReal[bin]) + sq(iReal[bin-1]) + sq(iReal[bin+1]);
         double mag = sqrt(mag_sq);

         // 노이즈 제거 (이제 0.005A보다 크면 살아남음)
         if (mag > harm_noise_floor) {
             mag -= harm_noise_floor;
         } else {
             mag = 0.0;
         }
         
         // 보정된 값을 다시 배열에 저장 (전송용)
         iReal[bin] = mag; 
         
         // THD 합산
         sum_sq_i_harm += (mag * mag);
      }
      
      if (i_fund > 0.01) {
          thd_i_value = (sqrt(sum_sq_i_harm) / i_fund) * 100.0;
         //  thd_i_value *= 1.13; 
      } else {
          thd_i_value = 0.0;
      }

      // [전류] 고조파 크기(%) 변환 (RMS -> %)
      // i_fund_mag_rms도 주변 에너지를 합친 i_fund 값을 사용하는 것이 더 정확함
      float i_fund_display = i_fund; 
      
      if (i_fund_display > 0.01) {
          // 기본파 위치에 100% 저장
          // (주의: 주변 bin 에너지는 이미 i_fund에 합쳐졌으므로, 대표 bin에 100을 넣음)
          iReal[FUNDALMENTAL_BIN] = 100.0; 
          
          for(int k=2; k<=MAX_HARMONIC; k++) {
              int bin = FUNDALMENTAL_BIN * k;
              if (bin >= FFT_N/2) break;
              // 위에서 계산된 mag가 iReal[bin]에 들어있음
              iReal[bin] = (iReal[bin] / i_fund_display) * 100.0; 
          }
      } else {
          for(int i=FUNDALMENTAL_BIN; i<FFT_N/2; i++) iReal[i] = 0.0;
      }
      
      // // [RMS 보정] 전체 I_rms도 노이즈만큼 빼서 디스플레이 값(A)을 맞춤
      // if (I_rms > harm_noise_floor && I_rms < 1.0) {
      //     I_rms -= harm_noise_floor;
      // }

  } else {
      // ... (전류가 없을 때 처리) ...
      thd_i_value = 0.0;
      phase_main_deg = 0.0;
      // ...
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
