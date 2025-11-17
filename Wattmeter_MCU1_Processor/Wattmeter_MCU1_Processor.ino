/*
 * ==============================================================================
 * [Processor.ino]
 * - 릴레이 제어, 물리량 계산, 퍼지 로직, FFT/THD 연산 담당
 * - D0/D1 (Serial)을 통해 Controller와 통신합니다.
 * ==============================================================================
 */

// --- 라이브러리 포함 ---
#include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>

// ==============================================================================
// [NEW] 공통 데이터 구조체 (Controller.ino에도 동일하게 정의되어야 함)
// ==============================================================================

// 1. Processor -> Controller (데이터 전송용)
struct ProcessorData {
  // 메인 전력 값
  float V_rms;
  float I_rms;
  float I_rms_load1;
  float I_rms_load2;
  float P_real;
  float Q_reactive;
  float S_apparent;
  float PF;

  // THD 값
  float thd_v_value;
  float thd_i_value;
  
  // 위상 값
  float phase_main_deg;
  float phase_load1_deg;
  float phase_load2_deg;
  char lead_lag_status[8]; // "Lead", "Lag", "---"

  // 상태 값
  bool relay1_state;
  bool relay2_state;
  bool warningActive;
  char warningMessage[20]; // "OVER VOLTAGE!" 등
};

// 2. Controller -> Processor (설정 전송용)
struct ControllerSettings {
  float VOLTAGE_THRESHOLD;
  float V_MULTIPLIER;
  float I_MULTIPLIER;
  // [FIX] setting_step_index는 Controller가 소유하므로 전송할 필요 없음.
};

// 3. 통신 커맨드 (단일 바이트)
#define CMD_HEADER_DATA 'D'     // 'D' + ProcessorData struct
#define CMD_HEADER_SETTINGS 'S' // 'S' + ControllerSettings struct
#define CMD_TOGGLE_RELAY_1 '1'
#define CMD_TOGGLE_RELAY_2 '2'
#define CMD_RESET_WARNING 'W'
#define CMD_RESET_SETTINGS 'R'

// ==============================================================================
// --- 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5

// --- [v26] "BASE" 값 (수정되지 않는 기본값)
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

// --- [v27] "DEFAULT" 값 (초기화용)
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0
// #define DEFAULT_SETTING_STEP_INDEX 3 (Controller 소유)

// --- 실시간 설정 변수 (Controller로부터 받아옴) ---
float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

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

// --- 전역 변수 (물리량) ---
// [NEW] 모든 계산 값은 이 전역 데이터 구조체에 저장됩니다.
ProcessorData g_ProcData = {0}; 
volatile bool warningActive = false; // [NEW] 경고 상태는 Processor가 소유

// --- FFT 버퍼 ---
float32_t v_samples[FFT_N];
float32_t i_samples[FFT_N];
float32_t v_fft_output[FFT_N];
float32_t i_fft_output[FFT_N];
float32_t v_mags[FFT_N / 2];
float32_t i_mags[FFT_N / 2];
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
unsigned long lastDataSendTime = 0;
const unsigned long DATA_SEND_INTERVAL = 100; // 100ms마다 데이터 전송 (10Hz)

// --- 함수 프로토타입 ---
void buildFuzzySystem();
void runFuzzyLogic(); 
void controlRelays(float level);
float calculatePhase(long time_diff, float period_us);
void restoreDefaultSettings();
void performFFT_and_CalcTHD();
void calculatePowerMetrics();
float32_t calculateTHD(float32_t* mags, int fundamentalBin);
void checkSerialCommands(); // [NEW]
void sendDataToController(); // [NEW]

// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200); // D0/D1을 통한 Controller와 통신
  
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
  lastFuzzyTime = millis(); 

  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N); 
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N); 
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. Controller로부터 명령 수신
  checkSerialCommands();
  
  // 2. 모든 계산 수행
  performFFT_and_CalcTHD(); // THD 계산
  calculatePowerMetrics();  // V, I, P, 위상, 퍼지 로직 계산
  
  // 3. 100ms마다 Controller로 데이터 전송
  if (millis() - lastDataSendTime > DATA_SEND_INTERVAL) {
    sendDataToController();
    lastDataSendTime = millis();
  }
}

// ==============================================================================
// [NEW] 3. Controller 명령 수신 함수
// ==============================================================================
void checkSerialCommands() {
  while (Serial.available() > 0) {
    byte cmd = Serial.read();
    
    switch (cmd) {
      case CMD_TOGGLE_RELAY_1:
        if (!warningActive) { // 경고 상태가 아닐 때만 수동 토글 허용
          digitalWrite(RELAY_1_PIN, !digitalRead(RELAY_1_PIN));
        }
        break;
        
      case CMD_TOGGLE_RELAY_2:
        if (!warningActive) { // 경고 상태가 아닐 때만 수동 토글 허용
          digitalWrite(RELAY_2_PIN, !digitalRead(RELAY_2_PIN));
        }
        break;
        
      case CMD_RESET_WARNING:
        warningActive = false;
        g_ProcData.warningActive = false;
        strcpy(g_ProcData.warningMessage, "");
        // 경고 해제 시 릴레이도 끔
        digitalWrite(RELAY_1_PIN, LOW); 
        digitalWrite(RELAY_2_PIN, LOW);
        break;

      case CMD_RESET_SETTINGS:
        restoreDefaultSettings();
        break;

      case CMD_HEADER_SETTINGS:
        if (Serial.available() >= sizeof(ControllerSettings)) {
          ControllerSettings newSettings;
          Serial.readBytes((uint8_t*)&newSettings, sizeof(ControllerSettings));
          
          // 수신한 설정값으로 Processor의 전역 변수 업데이트
          VOLTAGE_THRESHOLD = newSettings.VOLTAGE_THRESHOLD;
          V_MULTIPLIER = newSettings.V_MULTIPLIER;
          I_MULTIPLIER = newSettings.I_MULTIPLIER;
        }
        break;
    }
  }
}

// ==============================================================================
// [NEW] 4. Controller 데이터 전송 함수
// ==============================================================================
void sendDataToController() {
  // 현재 릴레이 상태를 데이터 구조체에 업데이트
  g_ProcData.relay1_state = digitalRead(RELAY_1_PIN);
  g_ProcData.relay2_state = digitalRead(RELAY_2_PIN);
  
  // 헤더 바이트 + 데이터 구조체 전송
  Serial.write(CMD_HEADER_DATA);
  Serial.write((uint8_t*)&g_ProcData, sizeof(g_ProcData));
}

// ==============================================================================
// 13. 실시간 전력 계산 (모든 위상차 계산 포함)
// ==============================================================================
void calculatePowerMetrics() {
  
  unsigned long V_sq_sum = 0; 
  unsigned long I_sq_sum = 0; 
  unsigned long I_sq_sum_load1 = 0;
  unsigned long I_sq_sum_load2 = 0;
  long P_sum = 0; 
  int V_ac_max = 0;
  int I_ac_max = 0;

  int V_ac_bits_prev = 0; 
  int I_ac_bits_prev = 0; 
  int I1_ac_bits_prev = 0;
  int I2_ac_bits_prev = 0;
  long time_V_cross = -1, time_I_cross = -1, time_I1_cross = -1, time_I2_cross = -1;
  bool found_V_cross = false, found_I_cross = false, found_I1_cross = false, found_I2_cross = false; 

  unsigned long startTimePass = micros();
  
  // [NEW] 퍼지 로직은 전력 계산 루프 *시작 전*에 이전 I_rms 값으로 실행
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

    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits); 
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits); 
    
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

  // [NEW] 모든 계산 결과를 g_ProcData 구조체에 저장
  g_ProcData.V_rms = (V_rms_adc * effective_V_Calib) - effective_V_Offset;
  g_ProcData.I_rms = (I_rms_adc * effective_I_Calib) - effective_I_Offset; 
  g_ProcData.I_rms_load1 = (I_rms_adc_load1 * effective_I_Calib) - effective_I_Offset;
  g_ProcData.I_rms_load2 = (I_rms_adc_load2 * effective_I_Calib) - effective_I_Offset;
  g_ProcData.P_real = P_avg_adc * effective_V_Calib * effective_I_Calib; 
  
  if (g_ProcData.V_rms < 0) g_ProcData.V_rms = 0; 
  if (g_ProcData.I_rms < 0) g_ProcData.I_rms = 0; 
  if (g_ProcData.I_rms_load1 < 0) g_ProcData.I_rms_load1 = 0;
  if (g_ProcData.I_rms_load2 < 0) g_ProcData.I_rms_load2 = 0;

  g_ProcData.S_apparent = g_ProcData.V_rms * g_ProcData.I_rms; 
  
  if (g_ProcData.S_apparent < 0.01) { 
    g_ProcData.PF = 0.0; g_ProcData.P_real = 0.0; g_ProcData.Q_reactive = 0.0; 
  } else {
    g_ProcData.PF = g_ProcData.P_real / g_ProcData.S_apparent; 
    if (g_ProcData.PF > 1.0) g_ProcData.PF = 1.0; 
    if (g_ProcData.PF < -1.0) g_ProcData.PF = -1.0; 
    g_ProcData.Q_reactive = sqrt(max(0.0, g_ProcData.S_apparent * g_ProcData.S_apparent - g_ProcData.P_real * g_ProcData.P_real)); 
  }

  float period_us = 1000000.0 / FREQUENCY; 
  g_ProcData.phase_main_deg = 0.0;
  g_ProcData.phase_load1_deg = 0.0;
  g_ProcData.phase_load2_deg = 0.0;
  
  if (found_V_cross) {
    if (found_I_cross) g_ProcData.phase_main_deg = calculatePhase(time_I_cross - time_V_cross, period_us);
    if (found_I1_cross) g_ProcData.phase_load1_deg = calculatePhase(time_I1_cross - time_V_cross, period_us);
    if (found_I2_cross) g_ProcData.phase_load2_deg = calculatePhase(time_I2_cross - time_V_cross, period_us);
  }

  char temp_status[8] = "---";
  if (g_ProcData.phase_main_deg < -2.0) strcpy(temp_status, "Lead");
  else if (g_ProcData.phase_main_deg > 2.0) strcpy(temp_status, "Lag");
  
  if (g_ProcData.I_rms < 0.05) {
      g_ProcData.phase_main_deg = 0.0;
      strcpy(temp_status, "---"); 
      g_ProcData.PF = (g_ProcData.V_rms > 10.0) ? 1.0 : 0.0;
      g_ProcData.P_real = 0.0; 
      g_ProcData.Q_reactive = 0.0; 
  }
  if (g_ProcData.I_rms_load1 < 0.05) { g_ProcData.I_rms_load1 = 0.0; g_ProcData.phase_load1_deg = 0.0; }
  if (g_ProcData.I_rms_load2 < 0.05) { g_ProcData.I_rms_load2 = 0.0; g_ProcData.phase_load2_deg = 0.0; }

  // [NEW] 문자열 복사
  strcpy(g_ProcData.lead_lag_status, temp_status);

  // 과전압 보호 로직
  if (g_ProcData.V_rms > VOLTAGE_THRESHOLD) {
    digitalWrite(RELAY_1_PIN, HIGH);
    digitalWrite(RELAY_2_PIN, HIGH); 
    
    // [NEW] 경고 상태 및 메시지를 g_ProcData에 설정
    strcpy(g_ProcData.warningMessage, "OVER VOLTAGE!"); 
    g_ProcData.warningActive = true;
    warningActive = true; // Processor의 로컬 상태도 업데이트
  }
}

// ==============================================================================
// 17. 퍼지 로직 시스템 빌드 함수
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
// 18. 퍼지 로직 실행 헬퍼 함수
// ==============================================================================
void runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  if (deltaTime < 0.1) return; 

  // [NEW] g_ProcData.I_rms 사용
  float dI_dt = (g_ProcData.I_rms - last_I_rms) / deltaTime;
  last_I_rms = g_ProcData.I_rms;
  lastFuzzyTime = currentTime; 

  fuzzy->setInput(1, g_ProcData.I_rms); 
  fuzzy->setInput(2, dI_dt); 
  fuzzy->fuzzify(); 
  float outputLevel = fuzzy->defuzzify(1); 
  controlRelays(outputLevel); 
}

// ==============================================================================
// 19. 퍼지 출력 -> 릴레이 제어 변환 함수
// ==============================================================================
void controlRelays(float level) {
  // 이미 경고 상태(예: 과전압)이면 퍼지 로직이 릴레이를 LOW로 내리지 않도록 함
  if (warningActive) return; 

  if (level > 9.0) {
    digitalWrite(RELAY_2_PIN, HIGH);
    digitalWrite(RELAY_1_PIN, HIGH);
    
    // [NEW] 퍼지 로직 트립 경고 설정
    strcpy(g_ProcData.warningMessage, "FUZZY LOGIC TRIP");
    g_ProcData.warningActive = true;
    warningActive = true; // 로컬 상태 업데이트
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
    digitalWrite(RELAY_2_PIN, LOW);
    digitalWrite(RELAY_1_PIN, LOW);
  }
}

// ==============================================================================
// 23. CMSIS-DSP FFT 및 THD 계산 헬퍼 함수
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
// 24. 메인 FFT/THD 연산 함수
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
  
  // [NEW] g_ProcData에 THD 값 저장
  g_ProcData.thd_v_value = calculateTHD(v_mags, FUNDALMENTAL_BIN); 
  g_ProcData.thd_i_value = calculateTHD(i_mags, FUNDALMENTAL_BIN); 
}

// ==============================================================================
// 32. [v25] 초기화 헬퍼
// ==============================================================================
void restoreDefaultSettings() {
  VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
  V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
  I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
}

// ==============================================================================
// 4. [Processor.ino] 헬퍼 함수
// ==============================================================================
float calculatePhase(long time_diff, float period_us) {
  float phase = fmod(((float)time_diff / period_us) * 360.0, 360.0);
  if (phase > 180.0) phase -= 360.0;
  else if (phase < -180.0) phase += 360.0;
  return phase;
}