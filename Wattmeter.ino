#include "arduino_secrets.h"
//small changes

// ==============================================================================
// 0. ë¼ì´ë¸ë¬ë¦¬ í¬í¨ ë° ì ì
// ==============================================================================
#include <Adafruit_GFX.h>      // ILI9341 ê·¸ëí½ ë¼ì´ë¸ë¬ë¦¬
#include <Adafruit_ILI9341.h>  // ILI9341 ëë¼ì´ë² ë¼ì´ë¸ë¬ë¦¬
#include <ArduinoFFT.h>        // FFT ë¼ì´ë¸ë¬ë¦¬ (THD ê³ì°ì©)
#include <math.h>              // ìí í¨ì (sqrt, sin, acos ë±)

// í ì ì (Nano R4 ê¸°ì¤)
#define VOLTAGE_PIN    A3       // ZMPT101B ì¶ë ¥ (OPAMP ì°ê²° ê°ì )
#define CURRENT_PIN    A4       // ACS712 ì¶ë ¥
#define RELAY_PIN      13       // ë¦´ë ì´ ëª¨ë ì ì´ í (ìì)
#define TFT_CS         10       // Chip Select (D10)
#define TFT_DC         9        // Data/Command (D9)
#define TFT_RST        8        // Reset (D8) 

// ILI9341 ê°ì²´ ì ì¸
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
ArduinoFFT FFT = ArduinoFFT(); // FFT ê°ì²´ ì ì¸

// ADC ë° Timer ì¤ì 
#define SAMPLE_PERIOD_MS    1      // 1ms (1kHz ìíë§ ì£¼íì)
#define SAMPLES_PER_CALC    167    // ì½ 10ì£¼ê¸° (RMS ê³ì° ì£¼ê¸°)
#define FFT_SAMPLES         256    // FFT ìí ì (2^N)
#define MAX_CURRENT_LIMIT   7.0    // ë¦´ë ì´ ë³´í¸ ì ë¥ (ì½ 1500W/220V + ë§ì§)

// êµì  ë° ìµì ê³ì (14-bit ADC ê¸°ì¤)
const int V_OFFSET_ADC = 8192; // 2.5V
const int I_OFFSET_ADC = 8192; 
const float K_V_REAL = 0.0431; // V_RMS/ADC_RMS ë³í ê³ì
const float K_I_REAL = 0.0305; // A_RMS/ADC_RMS ë³í ê³ì

// ëì  ë° ë²í¼ ë³ì (ISRìì ì ê·¼íë¯ë¡ volatile)
volatile int sampleCount = 0;
volatile float V_sq_sum = 0.0;
volatile float I_sq_sum = 0.0;
volatile float P_sum = 0.0;
volatile bool calculationReady = false;

// FFT ë²í¼ (V, I íí ë¶ìì©)
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
int fft_count = 0;

// ==============================================================================
// 1. í¨ì íë¡í íì ì ì¸ ë° Timer ISR (Interrupt Service Routine)
// ==============================================================================
void setup_timer_interrupt();
void timer_ISR_routine();
void setup_display();
void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD);

void timer_ISR_routine() {
  if (calculationReady) return;

  // 1. ADC ëê¸° ìíë§
  int V_raw = analogRead(VOLTAGE_PIN);
  int I_raw = analogRead(CURRENT_PIN);
  
  // 2. DC ìµì ì ê±° (AC ì±ë¶ ì¶ì¶)
  float V_ac = (float)(V_raw - V_OFFSET_ADC);
  float I_ac = (float)(I_raw - I_OFFSET_ADC);
  
  // 3. RMS ë° P ê³ì° ëì 
  V_sq_sum += V_ac * V_ac;
  I_sq_sum += I_ac * I_ac;
  P_sum += V_ac * I_ac;
  
  // 4. FFT ë²í¼ ì ì¥ (FFT_SAMPLES ë§í¼ë§)
  if (fft_count < FFT_SAMPLES) {
    vReal[fft_count] = (double)V_ac;
    vImag[fft_count] = 0; // íìë¶ë 0
  }
  fft_count++;
  
  sampleCount++;
  
  // 5. ìíë§ íì ëë¬ íì¸
  if (sampleCount >= SAMPLES_PER_CALC) {
    calculationReady = true;
    // (ì¬ê¸°ì Timerë¥¼ ë©ì¶ë ì½ëë¥¼ ì¶ê°íë©´ ê³ì° ìê° íë³´ì ì ë¦¬í©ëë¤.)
  }
}

// ==============================================================================
// 2. Setup í¨ì
// ==============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // ë¦´ë ì´ ì´ê¸° ìí: ì ì ì¸ê° (NO ì ì  ë«í ê°ì )
  
  setup_display();
  // Nano R4 íê²½ì ë§ë Timer ì¤ì  í¨ì í¸ì¶
  // setup_timer_interrupt(); 
  
  Serial.println("System Initialized. Starting Power Measurement.");
}

// ==============================================================================
// 3. Main Loop - ê³ì° ìí ë° ê²°ê³¼ ì¶ë ¥
// ==============================================================================
void loop() {
  if (calculationReady) {
    // 1. ë³ì ìì  ë³µì¬ ë° ì´ê¸°í
    float temp_V_sq_sum = V_sq_sum;
    float temp_I_sq_sum = I_sq_sum;
    float temp_P_sum = P_sum;
    int temp_sampleCount = sampleCount;
    
    V_sq_sum = 0.0; I_sq_sum = 0.0; P_sum = 0.0; sampleCount = 0;
    calculationReady = false; 

    // 2. RMS ê³ì° ë° ì¤ì  ë¬¼ë¦¬ë ë³í
    float V_rms_adc = sqrt(temp_V_sq_sum / temp_sampleCount);
    float I_rms_adc = sqrt(temp_I_sq_sum / temp_sampleCount);
    float V_rms_real = V_rms_adc * K_V_REAL; // ì¤ì  RMS V
    float I_rms_real = I_rms_adc * K_I_REAL; // ì¤ì  RMS I

    // 3. ì í¨ ì ë ¥ (P) ê³ì°
    float P_avg_real = (temp_P_sum / temp_sampleCount) * K_V_REAL * K_I_REAL; 

    // 4. ë¬´í¨ ì ë ¥ (Q) ê³ì° (ë¶ê° ê¸°ë¥)
    float S_apparent = V_rms_real * I_rms_real; // í¼ì ì ë ¥ |S|
    float PF = P_avg_real / S_apparent; // ì­ë¥ 
    if (PF > 1.0) PF = 1.0; 
    if (PF < -1.0) PF = -1.0; 
    float Q_reactive = S_apparent * sin(acos(PF)); 

    // 5. THD ê³ì° (ë¶ê° ê¸°ë¥)
    // float THD_value = calculate_THD(); // THD ê³ì° í¨ì í¸ì¶ (ë³ë êµ¬í íì)
    float THD_value = 0.0; // ìì ê°

    // 6. ê²°ê³¼ ì¶ë ¥
    display_power_values(V_rms_real, I_rms_real, P_avg_real, Q_reactive, PF, THD_value);
    
    // 7. ë¦´ë ì´ ë³´í¸ ë¡ì§ ì¤í
    relay_protection_check(I_rms_real);
    
    fft_count = 0; // FFT ì¹´ì´í° ì´ê¸°í
  }
}


// ==============================================================================
// 4. ë³´ì¡° í¨ì êµ¬í (ë¦´ë ì´, ëì¤íë ì´ ì¤ì )
// ==============================================================================

void setup_display() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.println("Digital Wattmeter Ready");
}

void display_power_values(float V_rms, float I_rms, float P_real, float Q_reactive, float PF, float THD) {
  // LCD íë©´ì V, I, P, Q, PF, THD ê° ì¶ë ¥ ë¡ì§ (ì´ì  ëµë³ ì°¸ì¡°)
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); 
  tft.setTextSize(2);
  // V_RMS ì¶ë ¥
  tft.setCursor(10, 40); tft.print("V: "); tft.setTextColor(ILI9341_CYAN); tft.print(V_rms, 2); tft.println(" V  ");
  // I_RMS ì¶ë ¥
  tft.setCursor(10, 70); tft.print("I: "); tft.setTextColor(ILI9341_YELLOW); tft.print(I_rms, 2); tft.println(" A  ");
  // P_REAL ì¶ë ¥
  tft.setCursor(10, 100); tft.print("P: "); tft.setTextColor(ILI9341_GREEN); tft.print(P_real, 2); tft.println(" W  ");
  // Q_REACT ì¶ë ¥
  tft.setCursor(10, 130); tft.print("Q: "); tft.setTextColor(ILI9341_MAGENTA); tft.print(Q_reactive, 2); tft.println(" VAR");
  // PF ì¶ë ¥
  tft.setCursor(10, 160); tft.print("PF: "); tft.setTextColor(ILI9341_RED); tft.println(PF, 3);
  // THD ì¶ë ¥
  tft.setCursor(10, 190); tft.print("THD: "); tft.setTextColor(ILI9341_ORANGE); tft.print(THD, 2); tft.println(" %");
}

void relay_protection_check(float current) {
  if (current > MAX_CURRENT_LIMIT) {
    digitalWrite(RELAY_PIN, LOW); // ë¦´ë ì´ ëì (NO ì ì  ê°ë°©)
    tft.setCursor(10, 220); tft.setTextColor(ILI9341_RED); tft.println("OVERCURRENT FAULT!");
  }
}

float calculate_THD() {
    // 1. FFT ìí
    if (fft_count < FFT_SAMPLES) return 0.0; // ìí ë¶ì¡± ì ì¢ë£
    
    FFT.Compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, FFT_SAMPLES);
    
    // 2. ê¸°ë³¸í (60Hz) ì£¼íì ì¸ë±ì¤ ì°¾ê¸°
    // 1kHz ìíë§ ì£¼íì / 256 ìí = 3.9Hz/ì¸ë±ì¤
    int fundamental_idx = (int)(60.0 / (1000.0 / FFT_SAMPLES)); // ì½ 15.3 -> 15 ëë 16

    // 3. THD ê³ì° (THD = (ê³ ì¡°í RMS / ê¸°ë³¸í RMS) * 100%)
    float fundamental_mag = vReal[fundamental_idx];
    float harmonic_sq_sum = 0.0;

    for (int i = 2; i < FFT_SAMPLES / 2; i++) {
        // ê¸°ë³¸íë¥¼ ì ì¸í ëª¨ë  ê³ ì¡°í ì±ë¶ ì ê³± í©ì°
        if (i != fundamental_idx) {
            harmonic_sq_sum += vReal[i] * vReal[i];
        }
    }
    
    float THD = (sqrt(harmonic_sq_sum) / fundamental_mag) * 100.0;
    return THD;
}