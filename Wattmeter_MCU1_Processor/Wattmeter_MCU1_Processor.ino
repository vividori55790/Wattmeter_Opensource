/*
 * ==============================================================================
 * 파일명: Wattmeter_MCU1_Processor.ino
 * 버전: v209 (Auto-Clear Warning Added)
 * 설명: 
 * - 전압/전류 센서 RMS, 전력(P,Q,S), 역률, 위상차 계산.
 * - [Mod] 타이머 로직 제거 (MCU2 컨트롤러가 시간 관리 및 트리거 담당)
 * - [Mod] 정상 전압 복귀 시 warningActive 플래그 자동 해제 로직 추가
 * ==============================================================================
 */

#include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>
#include <ArduinoJson.h>

// --- 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5

// --- 물리 상수 및 보정 ---
const float V_ADC_MIDPOINT = 8192.0; 
const float I_ADC_MIDPOINT = 8192.0; 

#define SAMPLES_PER_CALC 334 
#define SAMPLE_PERIOD_US 1000
const float FREQUENCY = 60.0;

#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

float V_MULTIPLIER = 1.0;
float I_MULTIPLIER = 1.0;
float VOLTAGE_THRESHOLD = 240.0;

// --- FFT 상수 ---
#define FFT_N 512
#define SAMPLING_FREQ_HZ 7680.0f
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ)
#define FUNDALMENTAL_BIN 4
#define MAX_HARMONIC 40

// --- 전역 변수 (계산 결과) ---
float V_rms = 0.0, I_rms = 0.0, I_rms_load1 = 0.0, I_rms_load2 = 0.0;
float P_real = 0.0, Q_reactive = 0.0, S_apparent = 0.0, PF = 0.0;
float phase_main_deg = 0.0, phase_load1_deg = 0.0, phase_load2_deg = 0.0;
String lead_lag_status = "---";
float thd_v_value = 0.0, thd_i_value = 0.0;
String v_harmonics_str = "";
String i_harmonics_str = "";

bool warningActive = false;
String warningMessage = "";

// --- 릴레이 상태 ---
bool relay1_state = false;
bool relay2_state = false;

// --- FFT 객체 및 버퍼 ---
float32_t v_samples[FFT_N];
float32_t i_samples[FFT_N];
float32_t v_fft_output[FFT_N];
float32_t i_fft_output[FFT_N];
float32_t v_mags[FFT_N / 2];
float32_t i_mags[FFT_N / 2];
arm_rfft_fast_instance_f32 fft_inst_v;
arm_rfft_fast_instance_f32 fft_inst_i;

// --- 퍼지 로직 ---
Fuzzy *fuzzy; 
FuzzyInput *totalCurrent; 
FuzzyInput *currentChangeRate; 
FuzzyOutput *shutdownLevel; 
float last_I_rms = 0.0;
unsigned long lastFuzzyTime = 0; 

// --- 파형 스트리밍 제어 ---
bool isWaveformStreaming = false;
int waveformType = 0; 
int waveformSyncMode = 0; 
int waveformPeriodIdx = 0; // 0:1.0C, 1:1.5C, 2:3.0C
unsigned long lastDataSendTime = 0;
const unsigned long DATA_SEND_INTERVAL = 500; 

StaticJsonDocument<256> rxJsonDoc;
StaticJsonDocument<1536> txJsonDoc; 

// --- 함수 프로토타입 ---
void buildFuzzySystem();
void runFuzzyLogic();
void controlRelays(float level);
void performFFT_and_CalcTHD();
void calculatePowerMetrics();
void sendMainData();
void checkSerialCommand();
void runWaveformStreaming();
void waitForVoltageZeroCross();
float calculatePhase(long time_diff, float period_us);
float32_t calculateTHD(float32_t* mags, int fundamentalBin);
String getHarmonicsString(float32_t* mags, int fundamentalBin);

void setup() {
  Serial.begin(115200);  // USB Debugging
  Serial1.begin(115200); // Inter-MCU Communication
  
  analogReadResolution(14);
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);
  pinMode(RELAY_1_PIN, OUTPUT); 
  pinMode(RELAY_2_PIN, OUTPUT); 
  digitalWrite(RELAY_1_PIN, LOW); 
  digitalWrite(RELAY_2_PIN, LOW);

  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N); 
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N); 

  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1); 
  currentChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  buildFuzzySystem();
}

void loop() {
  checkSerialCommand(); // Reads from Serial1 (MCU2)
  
  if (isWaveformStreaming) {
    runWaveformStreaming();
  } else {
    performFFT_and_CalcTHD();
    calculatePowerMetrics();
    
    if (millis() - lastDataSendTime > DATA_SEND_INTERVAL) {
      sendMainData();
      lastDataSendTime = millis();
    }
  }
}

void checkSerialCommand() {
  if (Serial1.available() > 0) {
    String line = Serial1.readStringUntil('\n');
    DeserializationError error = deserializeJson(rxJsonDoc, line);
    if (!error) {
      String cmd = rxJsonDoc["CMD"];
      
      if (cmd == "REQ_WAVEFORM") {
         int mode = rxJsonDoc["MODE"]; 
         if (mode == 1) {
            isWaveformStreaming = true;
            waveformSyncMode = rxJsonDoc["SYNC"];
         } else {
            isWaveformStreaming = false;
         }
      } 
      else if (cmd == "SET_WAVE_TYPE") {
         waveformType = rxJsonDoc["TYPE"];
      }
      else if (cmd == "SET_WAVE_PERIOD") {
         waveformPeriodIdx = rxJsonDoc["IDX"];
      }
      else if (cmd == "SET_CALIB") {
         V_MULTIPLIER = rxJsonDoc["V_MULT"];
         I_MULTIPLIER = rxJsonDoc["I_MULT"];
      }
      else if (cmd == "SET_PROTECT") {
         VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"];
      }
      else if (cmd == "SET_RELAY") {
         int id = rxJsonDoc["ID"];
         int state = rxJsonDoc["STATE"]; // 0:OFF, 1:ON, 2:TOGGLE
         
         if (id == 1) {
            if(state == 2) relay1_state = !relay1_state;
            else relay1_state = (state == 1);
         }
         if (id == 2) {
            if(state == 2) relay2_state = !relay2_state;
            else relay2_state = (state == 1);
         }
         
         digitalWrite(RELAY_1_PIN, relay1_state ? HIGH : LOW);
         digitalWrite(RELAY_2_PIN, relay2_state ? HIGH : LOW);
         sendMainData(); 
      }
      else if (cmd == "RESET_WARNING") {
         warningActive = false;
         // 릴레이 상태 복구
         digitalWrite(RELAY_1_PIN, relay1_state ? HIGH : LOW);
         digitalWrite(RELAY_2_PIN, relay2_state ? HIGH : LOW);
      }
      else if (cmd == "RESET_SETTINGS") {
         V_MULTIPLIER = 1.0; I_MULTIPLIER = 1.0; VOLTAGE_THRESHOLD = 240.0;
      }
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

  int delay_us = 56; // Default 1.0C
  if (waveformPeriodIdx == 0) delay_us = 56;       // 1.0 Cycle
  else if (waveformPeriodIdx == 1) delay_us = 83;  // 1.5 Cycle 
  else if (waveformPeriodIdx == 2) delay_us = 167; // 3.0 Cycle 

  for (int i = 0; i < sampleCount; i++) {
    if (i % 20 == 0 && Serial1.available()) {
       String s = Serial1.readStringUntil('\n');
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
       float q_inst = 0.0;
       if (buffer_filled) {
          q_inst = v_lagged * i_inst; 
       }
       Serial1.print(p_inst); Serial1.print(","); Serial1.println(q_inst);
    } 
    else { 
       Serial1.print(i_inst); Serial1.print(","); 
       Serial1.print(i1_inst); Serial1.print(","); 
       Serial1.println(i2_inst);
    }

    while(micros() - startTime < (i + 1) * delay_us); 
  }
}

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
  
  txJsonDoc["H_V_STR"] = v_harmonics_str;
  txJsonDoc["H_I_STR"] = i_harmonics_str;
  
  txJsonDoc["V_MULT"] = V_MULTIPLIER;
  txJsonDoc["I_MULT"] = I_MULTIPLIER;
  txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD;
  
  txJsonDoc["R1"] = relay1_state;
  txJsonDoc["R2"] = relay2_state;
  
  txJsonDoc["T_ACTIVE"] = false; 
  txJsonDoc["T_LEFT_S"] = 0;
  txJsonDoc["T_SEC"] = 0;

  txJsonDoc["WARN"] = warningActive;
  if (warningActive) txJsonDoc["MSG"] = warningMessage;

  serializeJson(txJsonDoc, Serial1); 
  Serial1.println();
}

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
        time_V_cross = crossTime; found_V_cross = true; 
      }
      if (!found_I_cross && I_ac_bits_prev < 0 && I_ac_bits >= 0) {
        time_I_cross = crossTime; found_I_cross = true; 
      }
      if (!found_I1_cross && I1_ac_bits_prev < 0 && I_ac_bits_load1 >= 0) {
        time_I1_cross = crossTime; found_I1_cross = true; 
      }
      if (!found_I2_cross && I2_ac_bits_prev < 0 && I_ac_bits_load2 >= 0) {
        time_I2_cross = crossTime; found_I2_cross = true; 
      }
    }
    V_ac_bits_prev = V_ac_bits; I_ac_bits_prev = I_ac_bits; 
    I1_ac_bits_prev = I_ac_bits_load1; I2_ac_bits_prev = I_ac_bits_load2;
    
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
  
  if (V_rms < 0) V_rms = 0; if (I_rms < 0) I_rms = 0; 
  if (I_rms_load1 < 0) I_rms_load1 = 0; if (I_rms_load2 < 0) I_rms_load2 = 0;

  S_apparent = V_rms * I_rms; 
  
  if (S_apparent < 0.01) { 
    PF = 0.0; P_real = 0.0; Q_reactive = 0.0; 
  } else {
    PF = P_real / S_apparent; 
    if (PF > 1.0) PF = 1.0; if (PF < -1.0) PF = -1.0; 
    Q_reactive = sqrt(max(0.0, S_apparent * S_apparent - P_real * P_real)); 
  }

  float period_us = 1000000.0 / FREQUENCY; 
  if (found_V_cross) {
    if (found_I_cross) phase_main_deg = calculatePhase(time_I_cross - time_V_cross, period_us);
    if (found_I1_cross) phase_load1_deg = calculatePhase(time_I1_cross - time_V_cross, period_us);
    if (found_I2_cross) phase_load2_deg = calculatePhase(time_I2_cross - time_V_cross, period_us);
  }
  if (phase_main_deg < -2.0) lead_lag_status = "Lead";
  else if (phase_main_deg > 2.0) lead_lag_status = "Lag";
  else lead_lag_status = "---";

  // [Mod] 과전압 감지 로직 개선
  if (V_rms > VOLTAGE_THRESHOLD) {
    // 과전압 발생 시 동작
    digitalWrite(RELAY_1_PIN, HIGH);
    digitalWrite(RELAY_2_PIN, HIGH); 
    relay1_state = true; relay2_state = true;
    warningMessage = "OVER VOLTAGE!"; 
    warningActive = true;
  } else {
    // [Mod] 정상 전압으로 복귀 시 경고 신호 자동 해제
    // MCU1은 더 이상 WARN 신호를 보내지 않게 되며, 
    // MCU2는 화면 터치 전까지 마지막 경고 화면을 유지합니다 (Latch).
    if (warningActive) {
       warningActive = false;
       warningMessage = "";
    }
  }
}

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

  v_harmonics_str = getHarmonicsString(v_mags, FUNDALMENTAL_BIN);
  i_harmonics_str = getHarmonicsString(i_mags, FUNDALMENTAL_BIN);
}

String getHarmonicsString(float32_t* mags, int fundamentalBin) {
  String s = "";
  float32_t fund = mags[fundamentalBin];
  if (fund < 0.001) fund = 1.0; 

  // 기본파(1차) 포함 전송
  s += "100.0,"; 

  for (int k = 3; k <= 15; k += 2) {
    int bin = fundamentalBin * k;
    if (bin >= FFT_N/2) break;
    float pct = (mags[bin] / fund) * 100.0;
    s += String(pct, 1);
    if (k < 15) s += ",";
  }
  return s;
}

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

float calculatePhase(long time_diff, float period_us) {
  float phase = fmod(((float)time_diff / period_us) * 360.0, 360.0);
  if (phase > 180.0) phase -= 360.0;
  else if (phase < -180.0) phase += 360.0;
  return phase;
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

void buildFuzzySystem() {
  FuzzySet* safeCurrent = new FuzzySet(0, 0, 5, 6); 
  FuzzySet* warningCurrent = new FuzzySet(5, 6, 6, 7); 
  FuzzySet* dangerCurrent = new FuzzySet(6.5, 7.25, 7.25, 8); 
  FuzzySet* criticalCurrent = new FuzzySet(7.5, 8, 10, 10); 
  totalCurrent->addFuzzySet(safeCurrent);
  totalCurrent->addFuzzySet(warningCurrent); 
  totalCurrent->addFuzzySet(dangerCurrent);
  totalCurrent->addFuzzySet(criticalCurrent);
  fuzzy->addFuzzyInput(totalCurrent); 

  FuzzySet* stableChange = new FuzzySet(-2, -1, 1, 2);
  FuzzySet* slowIncrease = new FuzzySet(1, 3, 3, 5);
  FuzzySet* suddenSurge = new FuzzySet(4, 6, 10, 10); 
  currentChangeRate->addFuzzySet(stableChange);
  currentChangeRate->addFuzzySet(slowIncrease);
  currentChangeRate->addFuzzySet(suddenSurge);
  fuzzy->addFuzzyInput(currentChangeRate); 

  FuzzySet* level0 = new FuzzySet(0, 0, 0, 1);
  FuzzySet* level3 = new FuzzySet(2, 3, 3, 4);
  FuzzySet* level6 = new FuzzySet(5, 6, 6, 7);
  FuzzySet* level10 = new FuzzySet(9, 10, 10, 10);
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
}

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

void controlRelays(float level) {
  if (level > 9.0) {
    digitalWrite(RELAY_2_PIN, HIGH); digitalWrite(RELAY_1_PIN, HIGH);
    relay1_state = true; relay2_state = true;
    if (!warningActive) { warningMessage = "FUZZY TRIP"; warningActive = true; }
  } 
}