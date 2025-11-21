/*
 * ==============================================================================
 * File: Wattmeter_MCU1_Processor.ino
 * Version: v101_Fixed (Undefined Reference Fix & float32_t Fix)
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
const float CURRENT_CUTOFF_THRES = 0.15;

// Calibration Factors
float BASE_V_CALIB_RMS = 0.1744;
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
float V_rms = 0.0, I_rms = 0.0, I_rms_load1 = 0.0, I_rms_load2 = 0.0;
float P_real = 0.0, Q_reactive = 0.0, S_apparent = 0.0, PF = 0.0;
float phase_main_deg = 0.0, phase_load1_deg = 0.0, phase_load2_deg = 0.0;
String lead_lag_status = "---";
float thd_v_value = 0.0, thd_i_value = 0.0;
bool warningActive = false;
String warningMessage = "";
String tripReason = ""; // [New] 트립 원인 저장 변수
String tripRelay = "";  // [New] 차단 릴레이 정보 저장 변수

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
// [Fix] Changed float32_t to float for standard compatibility
float calculateTHD_FFT(double* mags, int fundamentalBin); 
void performStartupCalibration();

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

  performStartupCalibration();
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
          // [New] 트립 원인 식별 로직
          if (overvoltage_trip) tripReason = "OVER V";
          else if (overcurrent_trip) tripReason = "OVER I";
          else if (fuzzy_trip) tripReason = "OVER P";

          if (I_rms_load1 > I_rms_load2) {
             relay1_state = true; 
             digitalWrite(RELAY_1_PIN, HIGH);
             warningMessage = "TRIP: LOAD 1";
             tripRelay = "R1"; // [New] Load 1 트립 식별
          } else if (I_rms_load2 > I_rms_load1) {
             relay2_state = true; 
             digitalWrite(RELAY_2_PIN, HIGH);
             warningMessage = "TRIP: LOAD 2";
             tripRelay = "R2"; // [New] Load 2 트립 식별
          } else {
             relay2_state = true; 
             digitalWrite(RELAY_2_PIN, HIGH);
             relay1_state = true; 
             digitalWrite(RELAY_1_PIN, HIGH);
             warningMessage = "TRIP: LOAD 2 (EQ)";
             tripRelay = "ALL"; // [New] Eq -> ALL 식별
          }
          warningActive = true;
       } else {
          // 정상 상태(트립 없음) 시, 타이머 경고가 아니라면 경고 해제
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
      else if (cmd == "REQ_DATA") { // [New] 데이터 요청 명령 처리
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
    txJsonDoc["T_TYPE"] = tripReason; // [New] 트립 상세 원인 추가
    txJsonDoc["T_RLY"] = tripRelay;   // [New] 트립 릴레이 정보 추가
  }

  String harmonics_v_str = "";
  String harmonics_i_str = "";
  
  // Calculate Magnitudes using hypot() - Standard C math
  double v_fund = hypot(vReal[FUNDALMENTAL_BIN], vImag[FUNDALMENTAL_BIN]);
  double i_fund = hypot(iReal[FUNDALMENTAL_BIN], iImag[FUNDALMENTAL_BIN]);
  
  for (int k = 1; k <= MAX_HARMONIC; k++) { 
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
// Unified Analysis (Standard ArduinoFFT)
// ================================================================
void perform_unified_analysis() {
  
  // Step 1: Sampling
  unsigned long startTime = micros();
  for (int i = 0; i < FFT_N; i++) {
    raw_v_buf[i] = analogRead(VOLTAGE_PIN);
    raw_i_buf[i] = analogRead(CURRENT_PIN);
    raw_i1_buf[i] = analogRead(CURRENT_PIN_LOAD1);
    raw_i2_buf[i] = analogRead(CURRENT_PIN_LOAD2);
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US);
  }

  // Step 2: DC Offset Calculation
  double v_sum = 0, i_sum = 0, i1_sum = 0, i2_sum = 0;
  for (int i = 0; i < FFT_N; i++) {
    v_sum += raw_v_buf[i]; i_sum += raw_i_buf[i]; 
    i1_sum += raw_i1_buf[i]; i2_sum += raw_i2_buf[i];
  }
  double v_offset = v_sum / FFT_N;
  double i_offset = i_sum / FFT_N;
  double i1_offset = i1_sum / FFT_N;
  double i2_offset = i2_sum / FFT_N;

  // Step 3: Integration & FFT Prep
  double sum_v_sq = 0; double sum_i_sq = 0; 
  double sum_i1_sq = 0; double sum_i2_sq = 0;

  for (int i = 0; i < FFT_N; i++) {
    double v_inst = (double)raw_v_buf[i] - v_offset;
    double i_inst = (double)raw_i_buf[i] - i_offset;
    double i1_inst = (double)raw_i1_buf[i] - i1_offset;
    double i2_inst = (double)raw_i2_buf[i] - i2_offset;

    sum_v_sq += v_inst * v_inst;
    sum_i_sq += i_inst * i_inst;
    sum_i1_sq += i1_inst * i1_inst;
    sum_i2_sq += i2_inst * i2_inst;

    // FFT Fill
    vReal[i] = v_inst; vImag[i] = 0;
    iReal[i] = i_inst; iImag[i] = 0;
  }

  // Step 4: Final Metrics
  double v_rms_raw = sqrt(sum_v_sq / FFT_N);
  double i_rms_raw = sqrt(sum_i_sq / FFT_N);
  double i1_rms_raw = sqrt(sum_i1_sq / FFT_N);
  double i2_rms_raw = sqrt(sum_i2_sq / FFT_N);

  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_I1_Calib = BASE_I_CALIB_RMS1 * I_MULTIPLIER; 
  float effective_I2_Calib = BASE_I_CALIB_RMS2 * I_MULTIPLIER;

  V_rms = v_rms_raw * effective_V_Calib;
  I_rms = i_rms_raw * effective_I_Calib;
  I_rms_load1 = i1_rms_raw * effective_I1_Calib;
  I_rms_load2 = i2_rms_raw * effective_I2_Calib;

  if (V_rms < 1.0) V_rms = 0;
  // if (I_rms < CURRENT_CUTOFF_THRES) I_rms = 0;
  // if (I_rms_load1 < CURRENT_CUTOFF_THRES) I_rms_load1 = 0;
  // if (I_rms_load2 < CURRENT_CUTOFF_THRES) I_rms_load2 = 0;

  S_apparent = V_rms * I_rms;

  // Step 5: FFT Execution
  FFT_V.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
  FFT_V.compute(FFTDirection::Forward);
  
  if (I_rms > CURRENT_CUTOFF_THRES) {
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
      FFT_I.compute(FFTDirection::Forward);
  } else {
      for(int k=0; k<FFT_N; k++) { iReal[k]=0; iImag[k]=0; }
  }

  // Step 6: Phase & Power from Complex FFT
  int bin = FUNDALMENTAL_BIN;
  
  double v_re = vReal[bin]; double v_im = vImag[bin];
  double i_re = iReal[bin]; double i_im = iImag[bin];

  double ang_v = atan2(v_im, v_re);
  double ang_i = atan2(i_im, i_re);
  const float PHASE_CORRECTION_DEG = 18.5; 

  double phase_diff_rad = ang_v - ang_i;
  
  // 보정값 적용 (도 -> 라디안 변환 후 차감)
  phase_diff_rad -= (PHASE_CORRECTION_DEG * PI / 180.0);
  while (phase_diff_rad > PI) phase_diff_rad -= 2.0 * PI;
  while (phase_diff_rad < -PI) phase_diff_rad += 2.0 * PI;

  phase_main_deg = phase_diff_rad * (180.0 / PI);
  
  if (S_apparent < 0.5) {
    PF = 0.0; P_real = 0.0; Q_reactive = 0.0; phase_main_deg = 0.0;
  } else {
    PF = cos(phase_diff_rad);
    P_real = fabs(S_apparent * PF); 
    Q_reactive = fabs(S_apparent * sin(phase_diff_rad));
  }

  if (phase_main_deg > 0) lead_lag_status = "Lag";
  else if (phase_main_deg < 0) lead_lag_status = "Lead";
  else lead_lag_status = "---";
  
  // Step 7: Magnitude & THD
  // Note: complexToMagnitude modifies the Real array to contain magnitude.
  FFT_V.complexToMagnitude(); 
  if (I_rms > CURRENT_CUTOFF_THRES) FFT_I.complexToMagnitude();
  
  thd_v_value = calculateTHD_FFT(vReal, FUNDALMENTAL_BIN); 
  
  float thd_v_raw = calculateTHD_FFT(vReal, FUNDALMENTAL_BIN);
  float thd_i_raw = 0.0;
  if (I_rms > CURRENT_CUTOFF_THRES) {
      thd_i_raw = calculateTHD_FFT(iReal, FUNDALMENTAL_BIN);
  }

  // [FIX] THD Calibration (0.5 for V, 1.13 for I)
  const float THD_V_CALIB_FACTOR = 0.50;
  const float THD_I_CALIB_FACTOR = 1.13;

  thd_v_value = thd_v_raw * THD_V_CALIB_FACTOR;
  thd_i_value = thd_i_raw * THD_I_CALIB_FACTOR;

  // ------------------------------------------------------------------
  // [New] Phase Calculation for Load 1 & Load 2
  // (Executed after Main Current THD calc to reuse iReal buffer safely)
  // ------------------------------------------------------------------
  
  // --- Load 1 Calculation ---
  if (I_rms_load1 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
          iReal[k] = (double)raw_i1_buf[k] - i1_offset;
          iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double i1_ang = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double phase_diff_1 = ang_v - i1_ang;
      
      phase_diff_1 -= (PHASE_CORRECTION_DEG * PI / 180.0);
      while (phase_diff_1 > PI) phase_diff_1 -= 2.0 * PI;
      while (phase_diff_1 < -PI) phase_diff_1 += 2.0 * PI;
      
      phase_load1_deg = phase_diff_1 * (180.0 / PI);
  } else {
      phase_load1_deg = 0.0;
  }

  // --- Load 2 Calculation ---
  if (I_rms_load2 > CURRENT_CUTOFF_THRES) {
      for(int k=0; k<FFT_N; k++) {
          iReal[k] = (double)raw_i2_buf[k] - i2_offset;
          iImag[k] = 0;
      }
      FFT_I.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT_I.compute(FFTDirection::Forward);
      
      double i2_ang = atan2(iImag[FUNDALMENTAL_BIN], iReal[FUNDALMENTAL_BIN]);
      double phase_diff_2 = ang_v - i2_ang;
      
      phase_diff_2 -= (PHASE_CORRECTION_DEG * PI / 180.0);
      while (phase_diff_2 > PI) phase_diff_2 -= 2.0 * PI;
      while (phase_diff_2 < -PI) phase_diff_2 += 2.0 * PI;
      
      phase_load2_deg = phase_diff_2 * (180.0 / PI);
  } else {
      phase_load2_deg = 0.0;
  }

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
  long startTime = micros();
  long timeout = 20000; 
  int V_ac_bits_prev = 0;
  int V_ac_bits = 0; 
  while (true) {
    int V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT; 
    if (V_ac_bits < -50) { V_ac_bits_prev = V_ac_bits; break; }
    if (micros() - startTime > timeout) return; 
  }
  while (true) {
    int V_raw = analogRead(VOLTAGE_PIN);
    V_ac_bits = V_raw - (int)V_ADC_MIDPOINT; 
    if (V_ac_bits_prev < 0 && V_ac_bits >= 0) return;
    V_ac_bits_prev = V_ac_bits;
    if (micros() - startTime > timeout * 2) return; 
  }
}

void performStartupCalibration() {
  double startupSumV = 0;
  int startupCount = 0;
  unsigned long start = millis();
  
  while (millis() - start < 2000) {
     perform_unified_analysis();
     startupSumV += V_rms;
     startupCount++;
  }
  
  if (startupCount > 0) {
     float avg_v = startupSumV / startupCount;
     if (avg_v > 1.0) {
        BASE_V_CALIB_RMS = BASE_V_CALIB_RMS * (218.9 / avg_v);
     }
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