// v.54 - 자동 보정 기능 통합 (MCU 2로부터 'C' 명령 수신)
/*
 * ==============================================================================
 * [전력계 2-MCU 분리 시스템 - MCU 1: 연산 프로세서]
 * - v54 변경점:
 * - 'PowerData' 구조체에 'D' 헤더 추가 (통신 안정성).
 * - 'CalibrationRequest' ('C') 구조체 추가.
 * - Test.ino로부터 measureOffsets(), calculateNewGains() 이식.
 * - 'C' 명령 수신 시 보정 수행 및 'S' (SettingsPacket) 패킷으로 MCU 2에 결과 전송.
 * - calculateNewGains()가 4개 센서 모두의 Gain/Offset 값을 설정하도록 수정.
 * ==============================================================================
 */

// --- 계산용 라이브러리 ---
// #include <math.h>
#include <Fuzzy.h>
#include <FuzzySet.h>
#include <arm_math.h>
#include <avr/pgmspace.h> 

// --- 핀 정의 (센서 입력) ---
#define VOLTAGE_PIN A3
#define CURRENT_PIN A2
#define CURRENT_PIN_LOAD1 A4
#define CURRENT_PIN_LOAD2 A5

// --- 릴레이 핀 정의 ---
#define RELAY_1_PIN 4
#define RELAY_2_PIN 5

// --- 기본 설정값 ---
#define DEFAULT_VOLTAGE_THRESHOLD 240.0
#define DEFAULT_V_MULTIPLIER 1.0
#define DEFAULT_I_MULTIPLIER 1.0

// --- [v54] 기본 보정 계수 (Test.ino 기준) ---
#define DEFAULT_V_ADC_MIDPOINT 8192.0
#define DEFAULT_I_ADC_MIDPOINT 8192.0
#define DEFAULT_I1_ADC_MIDPOINT 8192.0
#define DEFAULT_I2_ADC_MIDPOINT 8192.0
#define DEFAULT_V_CALIB_RMS 0.1775
#define DEFAULT_I_CALIB_RMS 0.005
#define DEFAULT_V_OFFSET_ADJUST 7.1
#define DEFAULT_I_OFFSET_ADJUST 2.5546

// --- 보정 계수 (글로벌 변수, MCU2의 SettingsPacket으로 덮어써짐) ---
float V_ADC_MIDPOINT = DEFAULT_V_ADC_MIDPOINT; 
float I_ADC_MIDPOINT = DEFAULT_I_ADC_MIDPOINT; 
float I1_ADC_MIDPOINT = DEFAULT_I1_ADC_MIDPOINT;
float I2_ADC_MIDPOINT = DEFAULT_I2_ADC_MIDPOINT;

float BASE_V_CALIB_RMS = DEFAULT_V_CALIB_RMS;
float BASE_I_CALIB_RMS = DEFAULT_I_CALIB_RMS;
float BASE_I1_CALIB_RMS = DEFAULT_I_CALIB_RMS; // I-Main과 동일하게 시작
float BASE_I2_CALIB_RMS = DEFAULT_I_CALIB_RMS; // I-Main과 동일하게 시작

float BASE_V_OFFSET_ADJUST = DEFAULT_V_OFFSET_ADJUST;
float BASE_I_OFFSET_ADJUST = DEFAULT_I_OFFSET_ADJUST;
float BASE_I1_OFFSET_ADJUST = DEFAULT_I_OFFSET_ADJUST; // I-Main과 동일하게 시작
float BASE_I2_OFFSET_ADJUST = DEFAULT_I_OFFSET_ADJUST; // I-Main과 동일하게 시작

// --- 실시간 설정 변수 (MCU2의 SettingsPacket으로 덮어써짐) ---
float VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
float V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
float I_MULTIPLIER = DEFAULT_I_MULTIPLIER; 

// --- FFT 상수 ---
#define FFT_N 512
#define SAMPLING_FREQ_HZ 7680.0f
#define SAMPLING_PERIOD_US (1000000.0f / SAMPLING_FREQ_HZ) 
#define FUNDALMENTAL_BIN 4
#define MAX_HARMONIC 40
#define NUM_HARMONICS_TO_SEND 20 
const float FREQUENCY = 60.0;

// --- 전역 변수 (물리량) ---
float V_rms = 0.0, I_rms = 0.0, I_rms_load1 = 0.0, I_rms_load2 = 0.0;
float P_real = 0.0, Q_reactive = 0.0, S_apparent = 0.0, PF = 0.0; 
float phase_degrees = 0.0;
String lead_lag_status = "---"; 
float phase_main_deg = 0.0, phase_load1_deg = 0.0, phase_load2_deg = 0.0;

// --- FFT 버퍼 ---
float32_t v_samples[FFT_N], i_samples[FFT_N];
float32_t v_fft_output[FFT_N], i_fft_output[FFT_N];
float32_t v_mags[FFT_N / 2], i_mags[FFT_N / 2];
float32_t thd_v_value = 0.0, thd_i_value = 0.0;
arm_rfft_fast_instance_f32 fft_inst_v, fft_inst_i;

// --- 퍼지 로직 변수 ---
Fuzzy *fuzzy; 
FuzzyInput *totalCurrent, *currentChangeRate; 
FuzzyOutput *shutdownLevel; 
FuzzySet *safeCurrent, *warningCurrent, *dangerCurrent, *criticalCurrent; 
FuzzySet *stableChange, *slowIncrease, *suddenSurge;
FuzzySet *level0, *level3, *level6, *level10; 
float last_I_rms = 0.0;
unsigned long lastFuzzyTime = 0; 

// --- 릴레이 상태 변수 ---
volatile bool relay1_on = true;
volatile bool relay2_on = true;
volatile bool auto_trip_active = false;

// --- [v54] 통신용 데이터 구조체 (헤더 'D' 추가) ---
struct PowerData {
  char command_char; // 'D'
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
  float thd_v; 
  float thd_i; 
  float fuzzy_output_level; 
  float voltage_threshold;  
  float v_harmonics[NUM_HARMONICS_TO_SEND]; 
  float i_harmonics[NUM_HARMONICS_TO_SEND]; 
  bool relay1_trip_command;
  bool relay2_trip_command;
  bool actual_relay1_state;
  bool actual_relay2_state;
};
PowerData dataToSend;

// --- [v52] MCU 2로부터 수신할 제어 명령 구조체 ---
struct ControlRequest {
  char command_char; // 'T' (Toggle), 'R' (Reset)
  char relay_num;    // '1', '2' (Toggle용)
};
ControlRequest controlReq;

// --- [v53] MCU 2와 주고받을 설정/보정 구조체 ---
struct SettingsPacket {
  char command_char; // MUST BE 'S'
  float v_multiplier;
  float i_multiplier;
  float voltage_threshold;
  
  // Full calibration data
  float v_midpoint;
  float i_midpoint;
  float i1_midpoint;
  float i2_midpoint;
  float v_gain;
  float i_gain;
  float i1_gain;
  float i2_gain;
  
  // Offset adjusts
  float v_offset_adjust;
  float i_offset_adjust;
  float i1_offset_adjust;
  float i2_offset_adjust;
};

// --- [v54] MCU 2로부터 수신할 보정 명령 구조체 ---
struct CalibrationRequest {
  char command_char; // 'C'
  char calib_step; // '1' (offsets), '2' (gains)
  float true_v;    // for step 2
  float true_i;    // for step 2
};


// ==============================================================================
// 1. Setup 함수
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("MCU 1 (Processor) v54 Booting..."));

  Serial1.begin(115200); 
  
  analogReadResolution(14); 
  pinMode(VOLTAGE_PIN, INPUT); 
  pinMode(CURRENT_PIN, INPUT); 
  pinMode(CURRENT_PIN_LOAD1, INPUT);
  pinMode(CURRENT_PIN_LOAD2, INPUT);

  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  digitalWrite(RELAY_1_PIN, HIGH);
  digitalWrite(RELAY_2_PIN, HIGH);
  relay1_on = true;
  relay2_on = true;
  Serial.println(F("Relay pins initialized on MCU 1."));

  fuzzy = new Fuzzy();
  totalCurrent = new FuzzyInput(1); 
  currentChangeRate = new FuzzyInput(2); 
  shutdownLevel = new FuzzyOutput(1); 
  buildFuzzySystem();
  Serial.println(F("Fuzzy Logic System Built.")); 
  lastFuzzyTime = millis(); 

  arm_rfft_fast_init_f32(&fft_inst_v, FFT_N);
  arm_rfft_fast_init_f32(&fft_inst_i, FFT_N);
  Serial.println(F("CMSIS-DSP FFT Initialized."));
}

// ==============================================================================
// [v54] MCU 2로부터 제어 명령 및 설정 수신 함수 (대폭 수정)
// ==============================================================================
void checkSerialCommands() {
  if (Serial1.available() == 0) return;
  
  char command_char = Serial1.peek();
  
  if (command_char == 'T' || command_char == 'R') {
    // --- ControlRequest (토글, 리셋) ---
    if (Serial1.available() >= sizeof(ControlRequest)) {
      Serial1.readBytes((uint8_t*)&controlReq, sizeof(ControlRequest));

      if (controlReq.command_char == 'T') {
        if (!auto_trip_active) { 
          if (controlReq.relay_num == '1') relay1_on = !relay1_on;
          else if (controlReq.relay_num == '2') relay2_on = !relay2_on;
        }
      } else if (controlReq.command_char == 'R') {
        auto_trip_active = false;
        relay1_on = true; 
        relay2_on = true;
      }
    }
  }
  else if (command_char == 'S') {
    // --- SettingsPacket (설정, 보정) ---
    if (Serial1.available() >= sizeof(SettingsPacket)) {
      SettingsPacket newSettings;
      Serial1.readBytes((uint8_t*)&newSettings, sizeof(SettingsPacket));
      
      if (newSettings.command_char == 'S') {
          // 새 설정 적용
          V_MULTIPLIER = newSettings.v_multiplier;
          I_MULTIPLIER = newSettings.i_multiplier;
          VOLTAGE_THRESHOLD = newSettings.voltage_threshold;
          
          // 새 보정 계수 적용
          V_ADC_MIDPOINT = newSettings.v_midpoint;
          I_ADC_MIDPOINT = newSettings.i_midpoint;
          I1_ADC_MIDPOINT = newSettings.i1_midpoint;
          I2_ADC_MIDPOINT = newSettings.i2_midpoint;
          
          BASE_V_CALIB_RMS = newSettings.v_gain;
          BASE_I_CALIB_RMS = newSettings.i_gain;
          BASE_I1_CALIB_RMS = newSettings.i1_gain;
          BASE_I2_CALIB_RMS = newSettings.i2_gain;
          
          BASE_V_OFFSET_ADJUST = newSettings.v_offset_adjust;
          BASE_I_OFFSET_ADJUST = newSettings.i_offset_adjust;
          BASE_I1_OFFSET_ADJUST = newSettings.i1_offset_adjust;
          BASE_I2_OFFSET_ADJUST = newSettings.i2_offset_adjust;
          
          Serial.println(F("MCU1: Received and applied new SettingsPacket."));
      } else {
         Serial.println(F("MCU1: Settings packet header mismatch. Flushing..."));
         while(Serial1.available()) Serial1.read(); 
      }
    }
  }
  else if (command_char == 'C') {
    // --- [v54] CalibrationRequest (자동 보정) ---
    if (Serial1.available() >= sizeof(CalibrationRequest)) {
      CalibrationRequest calReq;
      Serial1.readBytes((uint8_t*)&calReq, sizeof(CalibrationRequest));
      
      if (calReq.command_char == 'C') {
        if (calReq.calib_step == '1') {
          Serial.println(F("MCU1: Received CALIB_STEP_1. Measuring offsets..."));
          measureOffsets();
        } else if (calReq.calib_step == '2') {
          Serial.print(F("MCU1: Received CALIB_STEP_2. Calculating gains (V:"));
          Serial.print(calReq.true_v); Serial.print(F(", I:")); Serial.print(calReq.true_i); Serial.println(F(")"));
          calculateNewGains(calReq.true_v, calReq.true_i);
        }
        
        // 보정 수행 후, 갱신된 전체 설정을 MCU 2로 즉시 전송
        sendCurrentSettingsToMCU2();
        
      } else {
         Serial.println(F("MCU1: Calibration packet header mismatch. Flushing..."));
         while(Serial1.available()) Serial1.read();
      }
    }
  }
  else {
    Serial.print(F("MCU1: Unknown command byte: ")); Serial.println(command_char);
    Serial1.read(); 
  }
}

// ==============================================================================
// [v54] 현재 설정을 MCU 2로 전송하는 헬퍼 함수
// ==============================================================================
void sendCurrentSettingsToMCU2() {
  SettingsPacket packetToSend;
  packetToSend.command_char = 'S';
  
  // 현재 설정값
  packetToSend.v_multiplier = V_MULTIPLIER;
  packetToSend.i_multiplier = I_MULTIPLIER;
  packetToSend.voltage_threshold = VOLTAGE_THRESHOLD;
  
  // 현재 보정 계수 (Midpoints)
  packetToSend.v_midpoint = V_ADC_MIDPOINT;
  packetToSend.i_midpoint = I_ADC_MIDPOINT;
  packetToSend.i1_midpoint = I1_ADC_MIDPOINT;
  packetToSend.i2_midpoint = I2_ADC_MIDPOINT;
  
  // 현재 보정 계수 (Gains)
  packetToSend.v_gain = BASE_V_CALIB_RMS;
  packetToSend.i_gain = BASE_I_CALIB_RMS;
  packetToSend.i1_gain = BASE_I1_CALIB_RMS;
  packetToSend.i2_gain = BASE_I2_CALIB_RMS;
  
  // 현재 보정 계수 (Offset Adjusts)
  packetToSend.v_offset_adjust = BASE_V_OFFSET_ADJUST;
  packetToSend.i_offset_adjust = BASE_I_OFFSET_ADJUST;
  packetToSend.i1_offset_adjust = BASE_I1_OFFSET_ADJUST;
  packetToSend.i2_offset_adjust = BASE_I2_OFFSET_ADJUST;
  
  Serial.println(F("MCU1: Sending updated SettingsPacket to MCU2."));
  Serial1.write((uint8_t*)&packetToSend, sizeof(SettingsPacket));
}


// ==============================================================================
// 2. Main Loop
// ==============================================================================
void loop() {
  // 1. [v54] MCU 2로부터 제어/설정/보정 명령 확인
  checkSerialCommands();

  // 2. 핵심 연산 수행
  perform_unified_analysis(); 

  // 3. 퍼지 로직 실행
  float fuzzyLevel = runFuzzyLogic();

  // 4. 릴레이 트립 명령 생성 및 상태 결정
  bool fuzzy_trip_all = (fuzzyLevel > 5.0);
  bool fuzzy_trip_2 = (fuzzyLevel > 2.0 && fuzzyLevel <= 5.0); 
  bool overvoltage_trip = (V_rms > VOLTAGE_THRESHOLD);

  bool trip1_request = fuzzy_trip_all || overvoltage_trip;
  bool trip2_request = fuzzy_trip_all || fuzzy_trip_2 || overvoltage_trip;

  if (trip1_request) {
    relay1_on = false;
    auto_trip_active = true;
  }
  if (trip2_request) {
    relay2_on = false; 
    auto_trip_active = true;
  }

  // 5. 릴레이 핀 실제 구동
  digitalWrite(RELAY_1_PIN, relay1_on ? HIGH : LOW);
  digitalWrite(RELAY_2_PIN, relay2_on ? HIGH : LOW);

  // 6. 전송할 구조체에 데이터 패키징
  dataToSend.command_char = 'D'; // [v54] 헤더 설정
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
  dataToSend.thd_v = thd_v_value; 
  dataToSend.thd_i = thd_i_value; 
  dataToSend.fuzzy_output_level = fuzzyLevel;
  dataToSend.voltage_threshold = VOLTAGE_THRESHOLD; 
  
  dataToSend.relay1_trip_command = trip1_request;
  dataToSend.relay2_trip_command = trip2_request;
  dataToSend.actual_relay1_state = relay1_on;
  dataToSend.actual_relay2_state = relay2_on;

  // 7. Serial1을 통해 MCU 2로 구조체 전송
  Serial1.write((uint8_t*)&dataToSend, sizeof(dataToSend));

  delay(10); 
}

// ==============================================================================
// 3. [v53] 통합 샘플링 및 분석 함수
// ==============================================================================
void perform_unified_analysis() {
  
  // --- 1. 합산 변수 초기화 ---
  unsigned long V_sq_sum = 0; 
  unsigned long I_sq_sum = 0; 
  unsigned long I_sq_sum_load1 = 0;
  unsigned long I_sq_sum_load2 = 0;
  long P_sum = 0; 
  int V_ac_max = 0;
  int I_ac_max = 0;

  // --- 2. 위상차 변수 초기화 ---
  int V_ac_bits_prev = 0, I_ac_bits_prev = 0, I1_ac_bits_prev = 0, I2_ac_bits_prev = 0;
  long time_V_cross = -1, time_I_cross = -1, time_I1_cross = -1, time_I2_cross = -1;
  bool found_V_cross = false, found_I_cross = false, found_I1_cross = false, found_I2_cross = false; 

  unsigned long startTime = micros();
  float period_us_fft = 1000000.0 / FREQUENCY; 

  // --- 3. [v40] 통합 샘플링 루프 (FFT 기준: 512 샘플 @ 7680 Hz) ---
  for (int i = 0; i < FFT_N; i++) { 
    // 3-1. ADC 샘플링
    int V_raw = analogRead(VOLTAGE_PIN); 
    int I_raw = analogRead(CURRENT_PIN); 
    int I_raw_load1 = analogRead(CURRENT_PIN_LOAD1);
    int I_raw_load2 = analogRead(CURRENT_PIN_LOAD2);
    
    // 3-2. [v53] 개별 DC 오프셋(Midpoint) 제거
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    int I_ac_bits_load1 = I_raw_load1 - (int)I1_ADC_MIDPOINT;
    int I_ac_bits_load2 = I_raw_load2 - (int)I2_ADC_MIDPOINT;
    
    // 3-3. (Core Metrics) RMS 및 전력 합산
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits; 
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits; 
    I_sq_sum_load1 += (unsigned long)I_ac_bits_load1 * I_ac_bits_load1;
    I_sq_sum_load2 += (unsigned long)I_ac_bits_load2 * I_ac_bits_load2;
    P_sum += (long)V_ac_bits * I_ac_bits; 

    if (abs(V_ac_bits) > V_ac_max) V_ac_max = abs(V_ac_bits); 
    if (abs(I_ac_bits) > I_ac_max) I_ac_max = abs(I_ac_bits); 
    
    // 3-4. (Phase) 0점 교차 감지
    if (i < (SAMPLING_FREQ_HZ / FREQUENCY)) { // 약 128 샘플
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

  // --- 4. [v53] 계산 (Core Metrics - 개별 계수 적용) ---
  float V_rms_adc = sqrt((float)V_sq_sum / FFT_N);
  float I_rms_adc = sqrt((float)I_sq_sum / FFT_N); 
  float I_rms_adc_load1 = sqrt((float)I_sq_sum_load1 / FFT_N);
  float I_rms_adc_load2 = sqrt((float)I_sq_sum_load2 / FFT_N);
  float P_avg_adc = (float)P_sum / FFT_N;

  // [v53] 개별 Gain(Calib) 및 Offset(Adjust) 계수 적용
  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;
  
  float effective_I1_Calib = BASE_I1_CALIB_RMS * I_MULTIPLIER; 
  float effective_I1_Offset = BASE_I1_OFFSET_ADJUST * I_MULTIPLIER;
  
  float effective_I2_Calib = BASE_I2_CALIB_RMS * I_MULTIPLIER;
  float effective_I2_Offset = BASE_I2_OFFSET_ADJUST * I_MULTIPLIER;

  V_rms = (V_rms_adc * effective_V_Calib) - effective_V_Offset;
  I_rms = (I_rms_adc * effective_I_Calib) - effective_I_Offset; 
  I_rms_load1 = (I_rms_adc_load1 * effective_I1_Calib) - effective_I1_Offset;
  I_rms_load2 = (I_rms_adc_load2 * effective_I2_Calib) - effective_I2_Offset;
  
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

  // --- 8. [v40] 계산 (FFT/THD) ---
  arm_rfft_fast_f32(&fft_inst_v, v_samples, v_fft_output, 0); 
  arm_cmplx_mag_f32(v_fft_output, v_mags, FFT_N / 2); 
  arm_rfft_fast_f32(&fft_inst_i, i_samples, i_fft_output, 0); 
  arm_cmplx_mag_f32(i_fft_output, i_mags, FFT_N / 2); 
  
  thd_v_value = calculateTHD(v_mags, FUNDALMENTAL_BIN); 
  thd_i_value = calculateTHD(i_mags, FUNDALMENTAL_BIN); 

  // --- 9. [v46] 계산 (Individual Harmonics) ---
  float32_t v_fundamental_mag = v_mags[FUNDALMENTAL_BIN];
  float32_t i_fundamental_mag = i_mags[FUNDALMENTAL_BIN];

  for (int i = 0; i < NUM_HARMONICS_TO_SEND; i++) {
    int n = i + 2; 
    int binIndex = FUNDALMENTAL_BIN * n;
    
    if (binIndex >= (FFT_N / 2)) {
      dataToSend.v_harmonics[i] = 0.0;
      dataToSend.i_harmonics[i] = 0.0;
    } else {
      dataToSend.v_harmonics[i] = (v_fundamental_mag > 1e-9) ? (v_mags[binIndex] / v_fundamental_mag) : 0.0;
      dataToSend.i_harmonics[i] = (i_fundamental_mag > 1e-9) ? (i_mags[binIndex] / i_fundamental_mag) : 0.0;
    }
  }
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
// 6. [v54] 자동 보정 헬퍼 함수 (Test.ino에서 이식)
// ==============================================================================

// Step 1: 오프셋 측정
void measureOffsets() {
  long v_sum = 0;
  long i_sum = 0;
  long i1_sum = 0;
  long i2_sum = 0;
  int num_samples = 1000; // 1초간 샘플링 (1ms마다)

  for(int i=0; i<num_samples; i++) {
    v_sum += analogRead(VOLTAGE_PIN);
    i_sum += analogRead(CURRENT_PIN);
    i1_sum += analogRead(CURRENT_PIN_LOAD1);
    i2_sum += analogRead(CURRENT_PIN_LOAD2);
    delay(1);
  }

  // 새 오프셋(Midpoint)을 전역 변수에 즉시 적용
  V_ADC_MIDPOINT = (float)v_sum / num_samples;
  I_ADC_MIDPOINT = (float)i_sum / num_samples;
  I1_ADC_MIDPOINT = (float)i1_sum / num_samples;
  I2_ADC_MIDPOINT = (float)i2_sum / num_samples;

  Serial.println(F("--- Offsets Measured ---")); 
  Serial.print(F("V_ADC_MIDPOINT: ")); Serial.println(V_ADC_MIDPOINT, 4); 
  Serial.print(F("I_ADC_MIDPOINT: ")); Serial.println(I_ADC_MIDPOINT, 4); 
  Serial.print(F("I1_ADC_MIDPOINT: ")); Serial.println(I1_ADC_MIDPOINT, 4); 
  Serial.print(F("I2_ADC_MIDPOINT: ")); Serial.println(I2_ADC_MIDPOINT, 4); 
}

// Step 2: 게인 계산
void calculateNewGains(float true_v, float true_i) {
  // 현재 ADC RMS 값을 측정 (보정값 적용 전)
  unsigned long V_sq_sum = 0; 
  unsigned long I_sq_sum = 0;
  unsigned long startTime = micros();

  // 게인 계산은 FFT_N 샘플(약 67ms)을 기반으로 수행
  for (int i = 0; i < FFT_N; i++) { 
    int V_raw = analogRead(VOLTAGE_PIN); 
    int I_raw = analogRead(CURRENT_PIN); 
    
    // 방금 측정한 새 오프셋(중간값) 사용
    int V_ac_bits = V_raw - (int)V_ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)I_ADC_MIDPOINT;
    
    V_sq_sum += (unsigned long)V_ac_bits * V_ac_bits; 
    I_sq_sum += (unsigned long)I_ac_bits * I_ac_bits; 
    
    while(micros() - startTime < (i + 1) * SAMPLING_PERIOD_US); 
  }
  
  float V_rms_adc = sqrt((float)V_sq_sum / FFT_N);
  float I_rms_adc = sqrt((float)I_sq_sum / FFT_N); 

  // 새로운 보정 계수(게인) 계산 및 전역 변수 적용
  if (V_rms_adc > 1) { 
    BASE_V_CALIB_RMS = true_v / V_rms_adc;
  }
  if (I_rms_adc > 1) {
    BASE_I_CALIB_RMS = true_i / I_rms_adc;
  }

  // [v54] 요청대로, I-Main의 Gain 값을 I1, I2에도 동일하게 적용
  BASE_I1_CALIB_RMS = BASE_I_CALIB_RMS;
  BASE_I2_CALIB_RMS = BASE_I_CALIB_RMS;

  // [v54] 오프셋 보정값은 0으로 리셋 (ADC 중간값을 직접 사용하므로)
  BASE_V_OFFSET_ADJUST = 0.0;
  BASE_I_OFFSET_ADJUST = 0.0;
  BASE_I1_OFFSET_ADJUST = 0.0;
  BASE_I2_OFFSET_ADJUST = 0.0;
  
  // 수동 배율값(Multiplier)은 1.0으로 초기화
  V_MULTIPLIER = 1.0;
  I_MULTIPLIER = 1.0;

  Serial.println(F("--- New Gains Calculated ---")); 
  Serial.print(F("True V: ")); Serial.println(true_v); 
  Serial.print(F("V_rms_adc: ")); Serial.println(V_rms_adc, 4); 
  Serial.print(F("New BASE_V_CALIB_RMS: ")); Serial.println(BASE_V_CALIB_RMS, 7);
  Serial.print(F("True I: ")); Serial.println(true_i); 
  Serial.print(F("I_rms_adc: ")); Serial.println(I_rms_adc, 4); 
  Serial.print(F("New BASE_I_CALIB_RMS (all): ")); Serial.println(BASE_I_CALIB_RMS, 7);
}


// ==============================================================================
// 7. 퍼지 로직 시스템 빌드 함수
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
// 8. 퍼지 로직 실행 헬퍼 함수
// ==============================================================================
float runFuzzyLogic() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastFuzzyTime) / 1000.0; 
  if (deltaTime < 0.1) {
    return fuzzy->defuzzify(1); 
  }

  float dI_dt = (I_rms - last_I_rms) / deltaTime;
  last_I_rms = I_rms;
  lastFuzzyTime = currentTime; 

  fuzzy->setInput(1, I_rms); 
  fuzzy->setInput(2, dI_dt); 
  fuzzy->fuzzify(); 
  float outputLevel = fuzzy->defuzzify(1); 
  
  return outputLevel; 
}