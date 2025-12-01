/*
 * ==============================================================================
 * File: Wattmeter_MCU1_Processor.ino
 * Version: v107_Fixed (Voltage 214V Calib & Phase +20 Correction)
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

const float FREQUENCY = 60.0;
const float CURRENT_CUTOFF_THRES = 0.06;

// Calibration Factors
// [보정] 209V -> 214V로 맞추기 위해 값 상향 (0.16961 * 214/209)
float BASE_V_CALIB_RMS = 0.19172; 
float BASE_I_CALIB_RMS = 0.00316; 
float BASE_I_CALIB_RMS1 = 0.0030998;
float BASE_I_CALIB_RMS2 = 0.0030998;

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
float calculateTHD_FFT(double* mags, int fundamentalBin); 
// [Deleted] performStartupCalibration removed

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
    if (millis() - lastDataSendTime >= 500) {
       lastDataSendTime = millis();

       perform_unified_analysis();
       float fuzzyLevel = runFuzzyLogic();

       bool fuzzy_trip = (fuzzyLevel > 2.0);
       bool overvoltage_trip = (V_rms > VOLTAGE_THRESHOLD);
       bool overcurrent_trip = (I_rms > 7.0); 

       bool master_trip = fuzzy_trip || overvoltage_trip || overcurrent_trip;

       if (master_trip) {
          if (overvoltage_trip) tripReason = "OVER V";
          else if (overcurrent_trip) tripReason = "OVER I";
          else if (fuzzy_trip) tripReason = "OVER P";

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
             relay1_state = true; 
             digitalWrite(RELAY_1_PIN, HIGH);
             warningMessage = "TRIP: LOAD 2 (EQ)";
             tripRelay = "ALL"; 
          }
          warningActive = true;
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
      else if (cmd == "SET_WAVE_TYPE") { waveformType = rxJsonDoc["TYPE"]; }
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
         V_MULTIPLIER = 1.0; I_MULTIPLIER = 1.0; VOLTAGE_THRESHOLD = 240.0;
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
  
  double v_fund = hypot(vReal[FUNDALMENTAL_BIN], vImag[FUNDALMENTAL_BIN]);
  double i_fund = hypot(iReal[FUNDALMENTAL_BIN], iImag[FUNDALMENTAL_BIN]);
  
  // [수정 후] k를 2씩 증가시켜 홀수 차수(1, 3, 5...)만 처리
for (int k = 1; k <= MAX_HARMONIC; k += 2) { 

      // 첫 번째 데이터(1차) 뒤부터 쉼표(,) 추가
      if (k > 1) { harmonics_v_str += ","; harmonics_i_str += ","; }
      
      int bin = FUNDALMENTAL_BIN * k;
      
      double v_mag = hypot(vReal[bin], vImag[bin]);
      double i_mag = hypot(iReal[bin], iImag[bin]);
      
      if (v_fund > 0.1) {
         float pct = (v_mag / v_fund) * 100.0;
         harmonics_v_str += String(pct, 1);
      } else {
         harmonics_v_str += "0.0";
      }

      if (i_fund > 0.1 && I_rms > CURRENT_CUTOFF_THRES) {
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

// ================================================================
// Unified Analysis (Upgraded: Time-Domain Shift & Rotation Matrix)
// ================================================================
void perform_unified_analysis() {
  
  const int SHIFT_90_DEG = 16; 
  // [수정] 보정값 부호 반전 (-20.0 -> 20.0)
  // 이유: 측정값(-20도)에 -20도를 더해서 -40도가 되었으므로, 
  // +20도를 더해야 0도가 됨. (Offset correction)
  const float PHASE_CAL_DEG = -20.0;
  const float PHASE_CAL_LOAD1 = -23.0;
  const float PHASE_CAL_LOAD2 = -23.0;

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
    iReal[i] = i_inst_arr[i]; iImag[i] = 0;
  }

  V_rms = sqrt(sumV2 / FFT_N);
  I_rms = sqrt(sumI2 / FFT_N);
  
  if (V_rms < 1.0) V_rms = 0;
  if (I_rms < CURRENT_CUTOFF_THRES) { 
      I_rms = 0; sumP_raw = 0; sumQ_raw = 0; 
  }

  // Load 1, 2 RMS 계산 (기존 필터 대신 단순 RMS)
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
      
      // [수정] 무효전력 부호 제거 (절대값 처리)
      if (Q_reactive < 0) Q_reactive = -Q_reactive;
      S_apparent = hypot(P_real, Q_reactive);
      PF = P_real / S_apparent;
      
      if (PF > 1.0) PF = 1.0; 
      if (PF < -1.0) PF = -1.0;
      
      phase_main_deg = 0.0;
      lead_lag_status = "---";
  } else {
      P_real = 0; Q_reactive = 0; PF = 0.0;
      phase_main_deg = 0; lead_lag_status = "---";
  }

  // [8] FFT 처리 (Main V, I)
  FFT_V.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT_V.compute(FFTDirection::Forward);
  
  // 전압 위상 저장 (Magnitudes 계산 전)
  double ang_v = atan2(vImag[FUNDALMENTAL_BIN], vReal[FUNDALMENTAL_BIN]);

  FFT_V.complexToMagnitude(); 
  thd_v_value = calculateTHD_FFT(vReal, FUNDALMENTAL_BIN) * 0.50; 

  if (I_rms > CURRENT_CUTOFF_THRES) {
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      
      // [수정] 위상차 계산 순서 변경 (Current - Voltage)
      double angle_diff_rad = ang_i - ang_v; 
      double angle_diff_deg = angle_diff_rad * 180.0 / PI;
      
      while(angle_diff_deg > 180.0) angle_diff_deg -= 360.0;
      while(angle_diff_deg < -180.0) angle_diff_deg += 360.0;
      
      // [중요 수정] 하드웨어 위상 오차 보정 적용
      // +20.0도(PHASE_CAL_DEG)를 더해서 보정
      phase_main_deg = angle_diff_deg - PHASE_CAL_DEG;
      
      // [수정] 음수일 때 Lag, 양수일 때 Lead로 판단 (보정된 값 기준)
      if (phase_main_deg < -0.5) lead_lag_status = "Lag";
      else if (phase_main_deg > 0.5) lead_lag_status = "Lead";
      else lead_lag_status = "---";
      
      // Main Current THD 계산
      FFT_I.complexToMagnitude(); // iReal이 Magnitude로 덮어써짐
      thd_i_value = calculateTHD_FFT(iReal, FUNDALMENTAL_BIN) * 1.13; 
  } else {
      thd_i_value = 0.0;
      phase_main_deg = 0.0;
      lead_lag_status = "---";
  }

  // [New] 부하 1 (Load 1) 위상각 계산
  // 메모리 절약을 위해 iReal, iImag 버퍼를 재사용합니다.
  if (I_rms_load1 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = ((float)raw_i1_buf[k] - i1_offset) * BASE_I_CALIB_RMS1 * I_MULTIPLIER;
         iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i1 = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      // [수정] Current - Voltage (일관성 유지)
      double diff1 = (ang_i1 - ang_v) * 180.0 / PI;
      while(diff1 > 180.0) diff1 -= 360.0;
      while(diff1 < -180.0) diff1 += 360.0;
      
      // [중요] Load 1에도 하드웨어 위상 보정 적용
      phase_load1_deg = diff1 - PHASE_CAL_LOAD1;
  } else {
      phase_load1_deg = 0.0;
  }

  // [New] 부하 2 (Load 2) 위상각 계산
  if (I_rms_load2 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
         iReal[k] = ((float)raw_i2_buf[k] - i2_offset) * BASE_I_CALIB_RMS2 * I_MULTIPLIER;
         iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double ang_i2 = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      // [수정] Current - Voltage (일관성 유지)
      double diff2 = (ang_i2 - ang_v) * 180.0 / PI;
      while(diff2 > 180.0) diff2 -= 360.0;
      while(diff2 < -180.0) diff2 += 360.0;
      
      // [중요] Load 2에도 하드웨어 위상 보정 적용
      phase_load2_deg = diff2 - PHASE_CAL_LOAD2;
  } else {
      phase_load2_deg = 0.0;
  }

  // [9] 퍼지 로직 실행
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

// [Fix] Use float instead of float32_t
float calculateTHD_FFT(double* mags, int fundamentalBin) {
  double fundamental_power = mags[fundamentalBin] * mags[fundamentalBin];
  
  fundamental_power += mags[fundamentalBin-1] * mags[fundamentalBin-1];
  fundamental_power += mags[fundamentalBin+1] * mags[fundamentalBin+1];

  if (fundamental_power < 100.0) return 0.0; 

  double harmonics_power_sum = 0.0; 
  for (int n = 2; n <= MAX_HARMONIC; n++) {
    int binIndex = fundamentalBin * n;
    if (binIndex >= (FFT_N / 2)) break;
    harmonics_power_sum += mags[binIndex] * mags[binIndex]; 
  }
  
  float thd = sqrt(harmonics_power_sum / fundamental_power) * 100.0;
  if (thd > 200.0) return 0.0; 
  
  return thd;
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
       String s = Serial1.peek() == '{' ? Serial1.readStringUntil('\n') : "";
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
// Fuzzy Logic Functions (Added)
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
  suddenPowerSurge = new FuzzySet(450, 500, 2000, 2000);
  
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