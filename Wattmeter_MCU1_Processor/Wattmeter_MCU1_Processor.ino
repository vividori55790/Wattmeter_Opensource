/*
 * ==============================================================================
 * [전력계 2-MCU 분리 시스템 - MCU 1: 연산 프로세서]
 * - 담당: 모든 아날로그 샘플링(FFT_N), 전력 계산(RMS, P, Q, S, PF),
 * FFT 및 THD 계산, 퍼지 로직 연산.
 * - 제외: 디스플레이, 터치, Wi-Fi, 파형 그리기(runCombinedWaveformLoop)
 * - 통신: 계산된 최종 데이터(struct PowerData)를 Serial1(D1, TX)을 통해 전송.
 * ==============================================================================
 */

// --- 계산용 라이브러리 ---
#include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>

// --- 핀 정의 (센서 입력) ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5

// --- [v20] 실시간 설정 변수 (연산에 필요한 부분만 유지) ---
#define BASE_V_CALIB_RMS 0.1775
#define BASE_I_CALIB_RMS 0.005
#define BASE_V_OFFSET_ADJUST 7.1
#define BASE_I_OFFSET_ADJUST 2.5546

#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0

float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER;

// --- [v40] FFT 상수 (통합 샘플링 기준) ---
#define FFT_N 512
#define SAMPLING_FREQ_HZ 7680.0f
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ) // 약 130.2 us
#define FUNDALMENTAL_BIN 4
#define MAX_HARMONIC 40

// --- 캘리브레이션 상수 (ADC 오프셋) ---
const float V_ADC_MIDPOINT = 8192.0; 
const float I_ADC_MIDPOINT = 8192.0; 

const float FREQUENCY = 60.0;

// --- 전역 변수 (물리량) ---
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
  float fuzzy_output_level; // 퍼지 로직 결과
  float voltage_threshold;  // 과전압 경고용
};
PowerData dataToSend;

// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("MCU 1 (Processor) Booting...");

  // MCU 2(제어부)와 통신하기 위한 하드웨어 직렬 포트
  Serial1.begin(115200); 
  
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  // 퍼지 로직 시스템 빌드
  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1); 
  currentChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  buildFuzzySystem();
  Serial.println("Fuzzy Logic System Built."); 
  lastFuzzyTime = millis(); 

  // FFT 초기화
  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N);
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N);
  Serial.println("CMSIS-DSP FFT Initialized.");
}

// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. 핵심 연산 수행 (약 67ms 소요)
  perform_unified_analysis(); 

  // 2. 퍼지 로직 실행 및 결과 저장
  float fuzzyLevel = runFuzzyLogic();

  // 3. 전송할 구조체에 데이터 패키징
  dataToSend.V_rms = V_rms;
  dataToSend.I_rms = I_rms;
  dataToSend.I_rms_load1 = I_rms_load1;
  dataToSend.I_rms_load2 = I_rms_load2;
  dataToSend.P_real = P_real;
  dataToSend.Q_reactive = Q_reactive;
  dataToSend.S_apparent = S_apparent;
  dataToSend.PF = PF;
  dataToSend.phase_main_deg = phase_main_deg;
  dataToSend.phase_load1_deg = phase_load1_deg;
  dataToSend.phase_load2_deg = phase_load2_deg;
  dataToSend.thd_v_value = thd_v_value;
  dataToSend.thd_i_value = thd_i_value;
  dataToSend.fuzzy_output_level = fuzzyLevel;
  dataToSend.voltage_threshold = VOLTAGE_THRESHOLD; // 현재 설정된 임계값 전송

  // 4. Serial1을 통해 MCU 2로 구조체 전송
  Serial1.write((uint8_t*)&dataToSend, sizeof(dataToSend));

  // 연산 주기에 맞춰 대기 (이미 perform_unified_analysis 내부에서 처리됨)
  // 너무 빠른 전송을 막기 위해 최소 딜레이 추가
  delay(10); 
}

// ==============================================================================
// 3. [v40] 통합 샘플링 및 분석 함수
// ==============================================================================
void perform_unified_analysis() {
  
  // --- 1. 합산 변수 초기화 (Core Metrics용) ---
  unsigned long V_sq_sum = 0; 
  unsigned long I_sq_sum = 0; 
  unsigned long I_sq_sum_load1 = 0;
  unsigned long I_sq_sum_load2 = 0;
  long P_sum = 0; 
  int V_ac_max = 0;
  int I_ac_max = 0;

  // --- 2. 위상차 변수 초기화 (Phase용) ---
  int V_ac_bits_prev = 0; 
  int I_ac_bits_prev = 0; 
  int I1_ac_bits_prev = 0;
  int I2_ac_bits_prev = 0;
  long time_V_cross = -1, time_I_cross = -1, time_I1_cross = -1, time_I2_cross = -1;
  bool found_V_cross = false, found_I_cross = false, found_I1_cross = false, found_I2_cross = false; 

  unsigned long startTime = micros();
  float period_us_fft = 1000000.0 / FREQUENCY; // FFT용 위상차 계산 기준 (약 16666 us)

  // --- 3. [v40] 통합 샘플링 루프 (FFT 기준: 512 샘플 @ 7680 Hz) ---
  for (int i = 0; i < FFT_N; i++) { 
    // 3-1. ADC 샘플링
    int V_raw = analogRead(VOLTAGE_PIN); 
    int I_raw = analogRead(CURRENT_PIN); 
    int I_raw_load1 = analogRead(CURRENT_PIN_LOAD1);
    int I_raw_load2 = analogRead(CURRENT_PIN_LOAD2);
    
    // 3-2. DC 오프셋 제거
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    int I_ac_bits_load1 = I_raw_load1 - (int)I_ADC_MIDPOINT;
    int I_ac_bits_load2 = I_raw_load2 - (int)I_ADC_MIDPOINT;
    
    // 3-3. (Core Metrics) RMS 및 전력 합산
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits; 
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits; 
    I_sq_sum_load1 += (unsigned long)I_ac_bits_load1 * I_ac_bits_load1;
    I_sq_sum_load2 += (unsigned long)I_ac_bits_load2 * I_ac_bits_load2;
    P_sum += (long)V_ac_bits * I_ac_bits; 

    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits); 
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits); 
    
    // 3-4. (Phase) 0점 교차 감지 (첫 1주기 내에서만)
    if (i < (SAMPLING_FREQ_HZ / FREQUENCY)) { // (7680 / 60) = 약 128 샘플
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

    // 3-5. (FFT) Hanning 윈도우 적용 및 버퍼 저장
    float32_t window_factor = 0.5f - 0.5f * arm_cos_f32(2.0f * PI * i / (FFT_N - 1));
    v_samples[i] = (float32_t)V_ac_bits * window_factor; 
    i_samples[i] = (float32_t)I_ac_bits * window_factor;
    
    // 3-6. 샘플링 주기 대기
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US); 
  }

  // --- 4. [v40] 계산 (Core Metrics) ---
  float V_rms_adc = sqrt((float)V_sq_sum / FFT_N); // [v40] SAMPLES_PER_CALC -> FFT_N
  float I_rms_adc = sqrt((float)I_sq_sum / FFT_N); 
  float I_rms_adc_load1 = sqrt((float)I_sq_sum_load1 / FFT_N);
  float I_rms_adc_load2 = sqrt((float)I_sq_sum_load2 / FFT_N);
  float P_avg_adc = (float)P_sum / FFT_N;

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

  // --- 5. [v40] 계산 (Phase) ---
  phase_main_deg = 0.0;
  phase_load1_deg = 0.0;
  phase_load2_deg = 0.0;
  
  if (found_V_cross) {
    if (found_I_cross) phase_main_deg = calculatePhase(time_I_cross - time_V_cross, period_us_fft);
    if (found_I1_cross) phase_load1_deg = calculatePhase(time_I1_cross - time_V_cross, period_us_fft);
    if (found_I2_cross) phase_load2_deg = calculatePhase(time_I2_cross - time_V_cross, period_us_fft);
  }

  phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
  if (phase_main_deg < -2.0) lead_lag_status = "Lead";
  else if (phase_main_deg > 2.0) lead_lag_status = "Lag";
  else lead_lag_status = "---";

  if (I_rms < 0.05) {
     phase_degrees = 0.0;
     phase_main_deg = 0.0;
     lead_lag_status = "---"; 
     PF = (V_rms > 10.0) ? 1.0 : 0.0;
     P_real = 0.0; 
     Q_reactive = 0.0; 
  }
  if (I_rms_load1 < 0.05) { I_rms_load1 = 0.0; phase_load1_deg = 0.0; }
  if (I_rms_load2 < 0.05) { I_rms_load2 = 0.0; phase_load2_deg = 0.0; }

  // --- 6. [v40] 계산 (Protection Check) ---
  // (MCU 2가 처리하도록 전송만 함)
  // VOLTAGE_THRESHOLD 값은 dataToSend에 포함됨.

  // --- 7. [v40] 계산 (Fuzzy Logic Protection) ---
  // (runFuzzyLogic()이 loop에서 호출됨)

  // --- 8. [v40] 계산 (FFT/THD) ---
  arm_rfft_fast_f32(&fft_inst_v, v_samples, v_fft_output, 0); 
  arm_cmplx_mag_f32(v_fft_output, v_mags, FFT_N / 2); 
  arm_rfft_fast_f32(&fft_inst_i, i_samples, i_fft_output, 0); 
  arm_cmplx_mag_f32(i_fft_output, i_mags, FFT_N / 2); 
  
  thd_v_value = calculateTHD(v_mags, FUNDALMENTAL_BIN); 
  thd_i_value = calculateTHD(i_mags, FUNDALMENTAL_BIN); 
}

// ==============================================================================
// 4. 계산 헬퍼 함수
// ==============================================================================
float calculatePhase(long time_diff, float period_us) {
  float phase = fmod(((float)time_diff / period_us) * 360.0, 360.0);
  if (phase > 180.0) phase -= 360.0;
  else if (phase < -180.0) phase += 360.0;
  return phase;
}

// ==============================================================================
// 5. [v40] CMSIS-DSP FFT 및 THD 계산 헬퍼 함수
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
// 6. 퍼지 로직 시스템 빌드 함수
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
// 7. 퍼지 로직 실행 헬퍼 함수 (MCU 1)
// ==============================================================================
float runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  if (deltaTime < 0.1) {
    // 딜레이가 너무 짧으면 이전 값을 반환
    return fuzzy->defuzzify(1); 
  }

  float dI_dt = (I_rms - last_I_rms) / deltaTime;
  last_I_rms = I_rms;
  lastFuzzyTime = currentTime; 

  fuzzy->setInput(1, I_rms); 
  fuzzy->setInput(2, dI_dt); 
  fuzzy->fuzzify(); 
  float outputLevel = fuzzy->defuzzify(1); 
  
  // MCU 1은 릴레이를 제어하지 않고, 레벨 값만 반환
  return outputLevel; 
}