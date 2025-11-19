/*
 * ==============================================================================
 * 파일명: 3. View_Dynamic.ino
 * 버전: v207 (Waveform Redesign: Trigger & Stepped Range)
 * 설명: 
 * - [Redesign] Waveform: Zero-crossing Trigger 도입
 * - [Redesign] Waveform: Stepped Auto-ranging (Hysteresis)
 * - [Redesign] Waveform: Dual Y-Axis & Explicit Unit Labels
 * - [Mod] Harmonics: Absolute Value Log Scale, 2-Col List, Button Logic Fix
 * ==============================================================================
 */

// --- 헬퍼: 값 출력 (Float) ---
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) return; 
  
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  dtostrf(prev_value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit);
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit); 
}

// --- 헬퍼: 값 출력 (Int) ---
void printTFTValue(int x, int y, int value, int prev_value, uint16_t color, String unit) {
  if (value == prev_value) return; 
  
  char buffer[20];
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  sprintf(buffer, "%d%s", prev_value, unit.c_str());
  tft.print(buffer);
  
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  sprintf(buffer, "%d%s", value, unit.c_str());
  tft.print(buffer); 
}

// --- 헬퍼: 값 출력 (String) ---
void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (value.equals(prev_value)) return; 
  
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setCursor(x, y);
  tft.print(prev_value);
  tft.setTextColor(color); 
  tft.setCursor(x, y); 
  tft.print(value);
}

// --- [Mod] 네트워크 설정 화면 동적 갱신 ---
void runSettingsNetwork() {
  // 상태 표시 (WAIT 상태 포함)
  if (screenNeedsRedraw) {
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
  }

  // 버튼 텍스트 및 상태 (OFF -> WAIT -> ON)
  static WifiState prev_wifiState = (WifiState)-1;
  
  if (wifiState != prev_wifiState || screenNeedsRedraw) {
    String statusMsg = "";
    String btnText = "";
    uint16_t statusColor = COLOR_BUTTON;

    if (wifiState == WIFI_CONNECTED_STATE) {
       statusMsg = "WiFi: ON (Connected)";
       statusColor = COLOR_GREEN;
       btnText = "TURN OFF";
    } else if (wifiState == WIFI_WAIT) {
       statusMsg = "WiFi: WAIT...";
       statusColor = COLOR_ORANGE;
       btnText = "WAITING...";
    } else {
       statusMsg = "WiFi: OFF";
       statusColor = COLOR_BUTTON; // Greyish/Blue
       btnText = "TURN ON";
    }

    // 상태 바
    tft.fillRoundRect(60, 50, 200, 50, 10, statusColor);
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
    tft.setTextColor((wifiState==WIFI_CONNECTED_STATE)?ILI9341_BLACK:COLOR_BUTTON_TEXT); 
    tft.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(statusMsg, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(60 + (200 - w)/2, 65);
    tft.print(statusMsg);
    
    prev_wifiState = wifiState;
  }

  static bool prev_send_V = !send_V;
  static bool prev_send_I = !send_I;
  static bool prev_send_P = !send_P;

  if (send_V != prev_send_V || screenNeedsRedraw) {
     drawButton(20, 135, 60, 40, send_V ? "[V]" : " V ");
     prev_send_V = send_V;
  }
  if (send_I != prev_send_I || screenNeedsRedraw) {
     drawButton(90, 135, 60, 40, send_I ? "[I]" : " I ");
     prev_send_I = send_I;
  }
  if (send_P != prev_send_P || screenNeedsRedraw) {
     drawButton(160, 135, 60, 40, send_P ? "[P]" : " P ");
     prev_send_P = send_P;
  }
}

// --- 메인 전력 화면 값 업데이트 ---
void displayMainScreenValues() {
  if (isMainPowerFrozen) return; 

  tft.setTextSize(2);
  char buffer[20];
  int col1_value_x = 60;
  int col2_value_x = 220; 
  int col_w_half = 90; 
  int col_w_half_wide = 100;
  
  static float prev_V_rms = -1.0;
  static float prev_I_rms = -1.0;
  static float prev_P_real = -1.0;
  static float prev_PF = -1.0;
  static float prev_Q_reactive = -1.0;
  static float prev_I_rms_load1 = -1.0;
  static float prev_I_rms_load2 = -1.0;
  static float prev_S_apparent = -1.0;
  static float prev_thd_v_main = -1.0; 
  static float prev_thd_i_main = -1.0;

  printTFTValue(col1_value_x, 40, V_rms, prev_V_rms, 1, COLOR_BLUE, " V");
  prev_V_rms = V_rms;
  
  if (abs(I_rms - prev_I_rms) > 0.001) {
      tft.fillRect(col1_value_x, 65, col_w_half, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_ORANGE);
      tft.setCursor(col1_value_x, 65);
      if (I_rms < 1.0) { 
        dtostrf(I_rms * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_I_rms = I_rms;
  }
  
  if (abs(P_real - prev_P_real) > 0.1) {
      tft.fillRect(col1_value_x, 90, col_w_half, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_DARKGREEN);
      tft.setCursor(col1_value_x, 90);
      if (P_real >= 1000.0) { 
        dtostrf(P_real / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kW");
      } else { 
        dtostrf(P_real, 4, 1, buffer); tft.print(buffer); tft.print(" W");
      }
      prev_P_real = P_real;
  }

  printTFTValue(col2_value_x, 115, PF, prev_PF, 2, COLOR_MAGENTA, "");
  prev_PF = PF;
  
  if (abs(Q_reactive - prev_Q_reactive) > 0.1) {
      tft.fillRect(col1_value_x, 115, col_w_half, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_ORANGE);
      tft.setCursor(col1_value_x, 115);
      if (Q_reactive >= 1000.0) { 
        dtostrf(Q_reactive / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVAR");
      } else { 
        dtostrf(Q_reactive, 4, 1, buffer); tft.print(buffer); tft.print(" VAR");
      }
      prev_Q_reactive = Q_reactive;
  }

  if (abs(I_rms_load1 - prev_I_rms_load1) > 0.001) {
      tft.fillRect(col2_value_x, 40, col_w_half_wide, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_RED);
      tft.setCursor(col2_value_x, 40); 
      if (I_rms_load1 < 1.0) { 
        dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms_load1, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_I_rms_load1 = I_rms_load1;
  }

  if (abs(I_rms_load2 - prev_I_rms_load2) > 0.001) {
      tft.fillRect(col2_value_x, 65, col_w_half_wide, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_GREEN);
      tft.setCursor(col2_value_x, 65); 
      if (I_rms_load2 < 1.0) { 
        dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms_load2, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_I_rms_load2 = I_rms_load2;
  }

  if (abs(S_apparent - prev_S_apparent) > 0.1) {
      tft.fillRect(col2_value_x, 90, col_w_half_wide, 20, COLOR_BACKGROUND); 
      tft.setTextColor(COLOR_TEXT_PRIMARY);
      tft.setCursor(col2_value_x, 90); 
      if (S_apparent >= 1000.0) { 
        dtostrf(S_apparent / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVA");
      } else { 
        dtostrf(S_apparent, 4, 1, buffer); tft.print(buffer); tft.print(" VA");
      }
      prev_S_apparent = S_apparent;
  }

  printTFTValue(col2_value_x, 140, thd_v_value * 100.0, prev_thd_v_main * 100.0, 1, COLOR_BLUE, " %");
  prev_thd_v_main = thd_v_value;
  printTFTValue(col2_value_x, 165, thd_i_value * 100.0, prev_thd_i_main * 100.0, 1, COLOR_ORANGE, " %");
  prev_thd_i_main = thd_i_value;
}

// --- 위상차 화면 값 업데이트 ---
void displayPhaseScreenValues() {
  if (isPhaseFrozen) return; 

  phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
  if (I_rms < 0.05) phase_degrees = 0.0;

  tft.setTextSize(2);
  printTFTValue(60, 50, PF, prev_phase_degrees, 2, COLOR_MAGENTA, "");
  printTFTValue(10, 100, lead_lag_status, prev_lead_lag_status, COLOR_TEXT_PRIMARY); 
  printTFTValue(60, 140, phase_main_deg, prev_phase_main_deg, 1, COLOR_ORANGE, "d"); 
  printTFTValue(60, 165, phase_load1_deg, prev_phase_load1_deg, 1, COLOR_RED, "d"); 
  printTFTValue(60, 190, phase_load2_deg, prev_phase_load2_deg, 1, COLOR_GREEN, "d"); 

  prev_phase_degrees = PF; 
  prev_lead_lag_status = lead_lag_status; 
  prev_phase_main_deg = phase_main_deg;
  prev_phase_load1_deg = phase_load1_deg;
  prev_phase_load2_deg = phase_load2_deg;

  float max_I_rms = max(I_rms, max(I_rms_load1, I_rms_load2));
  float I_scale_denominator = (max_I_rms < 0.05) ? 0.0 : max_I_rms;
  float v_len = PHASOR_RADIUS;
  float im_len = 0.0, i1_len = 0.0, i2_len = 0.0;

  if (I_scale_denominator > 0.0) {
    im_len = (I_rms / I_scale_denominator) * PHASOR_RADIUS;
    i1_len = (I_rms_load1 / I_scale_denominator) * PHASOR_RADIUS;
    i2_len = (I_rms_load2 / I_scale_denominator) * PHASOR_RADIUS;
  }

  int v_x = PHASOR_CX + (int)(v_len * cos(0));
  int v_y = PHASOR_CY - (int)(v_len * sin(0));
  int im_x = PHASOR_CX + (int)(im_len * cos(phase_main_deg * (M_PI / 180.0)));
  int im_y = PHASOR_CY - (int)(im_len * sin(phase_main_deg * (M_PI / 180.0)));
  int i1_x = PHASOR_CX + (int)(i1_len * cos(phase_load1_deg * (M_PI / 180.0)));
  int i1_y = PHASOR_CY - (int)(i1_len * sin(phase_load1_deg * (M_PI / 180.0)));
  int i2_x = PHASOR_CX + (int)(i2_len * cos(phase_load2_deg * (M_PI / 180.0)));
  int i2_y = PHASOR_CY - (int)(i2_len * sin(phase_load2_deg * (M_PI / 180.0)));

  if (v_x != prev_v_x || v_y != prev_v_y) {
    tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, COLOR_BACKGROUND);
    tft.drawLine(PHASOR_CX, PHASOR_CY, v_x, v_y, COLOR_BLUE);
    prev_v_x = v_x; prev_v_y = v_y;
  }
  if (im_x != prev_im_x || im_y != prev_im_y) {
    tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, COLOR_BACKGROUND);
    tft.drawLine(PHASOR_CX, PHASOR_CY, im_x, im_y, COLOR_ORANGE);
    prev_im_x = im_x; prev_im_y = im_y;
  }
  if (i1_x != prev_i1_x || i1_y != prev_i1_y) {
    tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, COLOR_BACKGROUND);
    tft.drawLine(PHASOR_CX, PHASOR_CY, i1_x, i1_y, COLOR_RED);
    prev_i1_x = i1_x; prev_i1_y = i1_y;
  }
  if (i2_x != prev_i2_x || i2_y != prev_i2_y) {
    tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, COLOR_BACKGROUND);
    tft.drawLine(PHASOR_CX, PHASOR_CY, i2_x, i2_y, COLOR_GREEN);
    prev_i2_x = i2_x; prev_i2_y = i2_y;
  }
}

// --- [Mod] Dual Y-Axis & Units Update ---
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  // Clear previous labels
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 

  // Draw Left Axis (Plot 1) Labels
  tft.setTextColor(COLOR_BLUE);
  dtostrf(plot1_axis_max, 3, 0, buffer);
  
  // Determine Unit for Left Axis
  String unit1 = "V";
  if (waveformPlotType == 1) unit1 = "W";       // Power
  else if (waveformPlotType == 2) unit1 = "A";  // Current (I/I1/I2)
  
  tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print(unit1); 
  tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print(unit1);
    
  // Draw Right Axis (Plot 2/3) Labels
  // If Type 2 (I/I1/I2), Plot 2/3 share the same unit as Plot 1 (Amps)
  if (waveformPlotType == 2) {
    // Same scale as left usually for I/I1/I2, or independent?
    // Requirement says "Dual Y-Axis". For I/I1/I2, it's all Current, so maybe single axis is enough,
    // but for consistency, we can show Right Axis too.
    tft.setTextColor(COLOR_ORANGE);
    dtostrf(plot2_axis_max, 3, 0, buffer); // Use plot1_axis_max or plot2? usually same for all I
    // For I/I1/I2, we use plot1_axis_max for all lines typically, but let's use plot2 if different.
    // Note: In I/I1/I2 mode, we forced plot1=plot2=plot3 max.
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
  } 
  else {
    // V/I or P/Q
    tft.setTextColor(COLOR_ORANGE);
    String unit2 = "A";
    if (waveformPlotType == 1) unit2 = "Vr"; // VAR
    
    if (plot2_axis_max < 1.0) { 
      // Small current/power handling
      dtostrf(plot2_axis_max * 1000, 3, 0, buffer);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("m"); tft.print(unit2);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("m"); tft.print(unit2);
    } else { 
      dtostrf(plot2_axis_max, 3, 0, buffer);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print(unit2);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print(unit2);
    }
  }
}

// --- [Redesign] Triggered Scope & Stepped Auto-ranging Loop ---
void runCombinedWaveformLoop() {
  if (isWaveformFrozen) return;

  // --- 0. Buffers for Triggered Capture ---
  static float v_buf[PLOT_WIDTH];
  static float p2_buf[PLOT_WIDTH]; // I or P or I1
  static float p3_buf[PLOT_WIDTH]; // I2 (only for Type 2)
  
  // --- 1. Range State Variables ---
  static int range_idx_v = NUM_V_RANGES - 1;
  static int range_idx_i = NUM_I_RANGES - 1;
  static int range_idx_p = NUM_P_RANGES - 1;
  
  // --- 2. Wait for Zero-Crossing Trigger (Rising Edge) ---
  // We need to read ADC_V continuously to find the crossing.
  // Add a timeout so we don't hang if signal is missing.
  unsigned long trigger_start = millis();
  bool triggered = false;
  int center = ADC_MIDPOINT;
  int hyst = 50; // Trigger Noise Hysteresis
  
  // State 0: Find Low (V < center - hyst)
  // State 1: Find High (V > center + hyst) -> Trigger!
  int trig_state = 0;
  
  // Trigger loop (Max 30ms for 60Hz, but let's give 50ms)
  while (millis() - trigger_start < 50) {
     // Must check Back button during wait to avoid locking UI
     checkTouchInput();
     if (screenNeedsRedraw) return; // Exit if screen changed
     
     int raw_v = analogRead(PIN_ADC_V);
     
     if (trig_state == 0) {
        if (raw_v < (center - hyst)) trig_state = 1;
     } else if (trig_state == 1) {
        if (raw_v > (center + hyst)) {
           triggered = true;
           break; 
        }
     }
  }
  // If timeout (not triggered), we just capture anyway (Rolling mode fallback)
  
  // --- 3. Fast Sampling to Buffer ---
  // Capture PLOT_WIDTH samples
  float eff_V_mult = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float eff_I_mult = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float eff_V_off = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float eff_I_off = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;
  
  int delay_us = WAVEFORM_DELAYS_US[waveformPeriodIndex];
  
  // P/Q Lag Buffer logic (Need local mini-buffer for P/Q calculation)
  const int LAG_SIZE = 40;
  float local_lag_buf[LAG_SIZE];
  int lag_head = 0;
  // Initialize lag buffer with current V
  float init_v = (analogRead(PIN_ADC_V) - ADC_MIDPOINT) * eff_V_mult + eff_V_off;
  for(int k=0; k<LAG_SIZE; k++) local_lag_buf[k] = init_v;

  float max_val_p1 = 0.0; // To track peaks for auto-ranging
  float max_val_p2 = 0.0;
  
  for (int i = 0; i < PLOT_WIDTH; i++) {
    unsigned long t0 = micros();
    
    int r_v = analogRead(PIN_ADC_V);
    int r_i = analogRead(PIN_ADC_I);
    int r_i1 = analogRead(PIN_ADC_I1);
    int r_i2 = analogRead(PIN_ADC_I2);
    
    float v = (r_v - center) * eff_V_mult + eff_V_off;
    float cur = (r_i - center) * eff_I_mult - eff_I_off;
    float cur1 = (r_i1 - center) * eff_I_mult - eff_I_off;
    float cur2 = (r_i2 - center) * eff_I_mult - eff_I_off;
    
    if (waveformPlotType == 0) { // V / I
       v_buf[i] = v;
       p2_buf[i] = cur;
       if (abs(v) > max_val_p1) max_val_p1 = abs(v);
       if (abs(cur) > max_val_p2) max_val_p2 = abs(cur);
    } 
    else if (waveformPlotType == 1) { // P / Q
       // P = v * i
       float p = v * cur;
       v_buf[i] = p; // Plot 1 is P
       
       // Q = v_lagged * i
       local_lag_buf[lag_head] = v;
       int lag_read_idx = (lag_head + 1) % LAG_SIZE;
       float v_lag = local_lag_buf[lag_read_idx];
       lag_head = lag_read_idx;
       
       float q = v_lag * cur;
       p2_buf[i] = q; // Plot 2 is Q
       
       if (abs(p) > max_val_p1) max_val_p1 = abs(p);
       if (abs(q) > max_val_p2) max_val_p2 = abs(q);
    } 
    else { // I / I1 / I2
       v_buf[i] = cur;   // Plot 1 is I
       p2_buf[i] = cur1; // Plot 2 is I1
       p3_buf[i] = cur2; // Plot 3 is I2
       
       float m = max(abs(cur), max(abs(cur1), abs(cur2)));
       if (m > max_val_p1) max_val_p1 = m;
       // Shared scale for all I
       max_val_p2 = max_val_p1;
    }
    
    while (micros() - t0 < delay_us);
  }
  
  // --- 4. Stepped Auto-Ranging Logic (Hysteresis) ---
  bool rangeChanged = false;
  
  // Helper to update range index
  auto updateRange = [&](int &idx, float peak, const float* ranges, int count) {
      bool changed = false;
      // 1. Check Upgrade
      if (peak > ranges[idx]) {
          while (idx < count - 1 && peak > ranges[idx]) {
             idx++; 
          }
          changed = true;
      } 
      // 2. Check Downgrade (Hysteresis: Peak < 80% of LOWER range's limit)
      else if (idx > 0) {
          // Limit of the range below current
          float lower_limit = ranges[idx-1];
          if (peak < (lower_limit * 0.8)) {
             idx--;
             changed = true;
          }
      }
      return changed;
  };

  if (waveformPlotType == 0) { // V / I
     if (updateRange(range_idx_v, max_val_p1, V_RANGES, NUM_V_RANGES)) rangeChanged = true;
     if (updateRange(range_idx_i, max_val_p2, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     
     plot1_axis_max = V_RANGES[range_idx_v];
     plot2_axis_max = I_RANGES[range_idx_i];
  } 
  else if (waveformPlotType == 1) { // P / Q
     // Combine peaks for shared range or separate? Usually shared for P/Q logic
     float max_pq = max(max_val_p1, max_val_p2);
     if (updateRange(range_idx_p, max_pq, P_RANGES, NUM_P_RANGES)) rangeChanged = true;
     
     plot1_axis_max = P_RANGES[range_idx_p];
     plot2_axis_max = P_RANGES[range_idx_p];
  } 
  else { // I / I1 / I2
     if (updateRange(range_idx_i, max_val_p1, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     
     plot1_axis_max = I_RANGES[range_idx_i];
     plot2_axis_max = I_RANGES[range_idx_i];
     plot3_axis_max = I_RANGES[range_idx_i];
  }

  // If range changed, redraw grid/labels completely
  if (rangeChanged) {
     updateYAxisLabels();
     tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  }
  
  // --- 5. Draw Buffered Data (Erase Old -> Draw New) ---
  // Calculate Scales
  float scale1 = (plot1_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
  float scale2 = (plot2_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  float scale3 = (plot3_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot3_axis_max); // Only for Type 2

  tft.startWrite();
  for (int i = 0; i < PLOT_WIDTH; i++) {
     int x = PLOT_X_START + i;
     
     // 5-1. Erase Old Pixels (Draw Background)
     // Optimization: Draw background line segment connecting prev y to curr y
     // But here we have points. Efficient way is to draw background color on last frame's lines.
     if (i > 0) {
         int xp = x - 1;
         // Erase Line 1
         if (last_frame_y_plot1[i] != PLOT_Y_CENTER || last_frame_y_plot1[i-1] != PLOT_Y_CENTER)
            tft.drawLine(xp, last_frame_y_plot1[i-1], x, last_frame_y_plot1[i], COLOR_BACKGROUND);
         // Erase Line 2
         if (last_frame_y_plot2[i] != PLOT_Y_CENTER || last_frame_y_plot2[i-1] != PLOT_Y_CENTER)
            tft.drawLine(xp, last_frame_y_plot2[i-1], x, last_frame_y_plot2[i], COLOR_BACKGROUND);
         // Erase Line 3
         if (waveformPlotType == 2) {
            if (last_frame_y_plot3[i] != PLOT_Y_CENTER || last_frame_y_plot3[i-1] != PLOT_Y_CENTER)
                tft.drawLine(xp, last_frame_y_plot3[i-1], x, last_frame_y_plot3[i], COLOR_BACKGROUND);
         }
     }
     
     // 5-2. Calculate New Y Coordinates
     int y1 = constrain(PLOT_Y_CENTER - (int)(v_buf[i] * scale1), PLOT_Y_START, PLOT_Y_END);
     int y2 = constrain(PLOT_Y_CENTER - (int)(p2_buf[i] * scale2), PLOT_Y_START, PLOT_Y_END);
     int y3 = PLOT_Y_CENTER;
     if (waveformPlotType == 2) {
        y3 = constrain(PLOT_Y_CENTER - (int)(p3_buf[i] * scale3), PLOT_Y_START, PLOT_Y_END);
     }
     
     // 5-3. Draw New Lines
     if (i > 0) {
         int xp = x - 1;
         // Re-draw Center Line if crossed (Visual Polish) - Optional but looks better
         if ((y1 < PLOT_Y_CENTER && last_frame_y_plot1[i-1] > PLOT_Y_CENTER) || 
             (y1 > PLOT_Y_CENTER && last_frame_y_plot1[i-1] < PLOT_Y_CENTER)) {
             tft.drawPixel(x, PLOT_Y_CENTER, COLOR_GRID);
         }

         tft.drawLine(xp, last_frame_y_plot1[i-1], x, y1, COLOR_BLUE);
         tft.drawLine(xp, last_frame_y_plot2[i-1], x, y2, COLOR_ORANGE);
         if (waveformPlotType == 2) {
             tft.drawLine(xp, last_frame_y_plot3[i-1], x, y3, COLOR_RED);
         }
     }
     
     // 5-4. Update Last Frame Buffer
     last_frame_y_plot1[i] = y1;
     last_frame_y_plot2[i] = y2;
     last_frame_y_plot3[i] = y3;
  }
  // Restore Center Grid Line (Simple sweep)
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  tft.endWrite();
  
  // Check Single Trigger Mode
  if (waveformTriggerMode == 2 && triggered) {
    isWaveformFrozen = true;
  }
  
  checkSerialInput(); // Process any incoming data after drawing
}

// [Mod] HARMONICS 화면 값 업데이트 (로그 스케일 절대값, 색상 분리, 그리드 갱신)
void displayHarmonicsScreenValues() {
  if (harmonicsSrcLabel != prev_harmonicsSrcLabel) {
     drawButton(10, 200, 90, 35, harmonicsSrcLabel);
     prev_harmonicsSrcLabel = harmonicsSrcLabel;
     // 소스 변경 시 화면/그리드 갱신을 위해 강제 호출
     displayHarmonicsScreenStatic();
  }
  
  // [Mod] Button Logic Inverted: Measuring(Active) -> "HOLD", Frozen -> "RUN"
  String currRunLabel = isHarmonicsFrozen ? "RUN" : "HOLD";
  if (currRunLabel != prev_harmonicsRunLabel) {
     drawButton(110, 200, 100, 35, currRunLabel);
     prev_harmonicsRunLabel = currRunLabel;
  }

  if (isHarmonicsFrozen) return;

  // Use data based on Source
  float* dataPtr = (harmonicsSource == 0) ? v_harmonics : i_harmonics;
  float fundamental_rms = (harmonicsSource == 0) ? V_rms : I_rms;
  
  static int prev_bar_heights[16]; 
  static float prev_text_vals[8];  

  if (harmonicsViewMode == 0) {
     int graph_x = 40;
     int graph_y = 45;
     int graph_h = 145;
     int bar_w = 270 / 8; // Show 8 items (Fund + 7 Harmonics)

     // [Mod] Log Scale Range Definition
     // Voltage: 1V (Log 0) ~ 1000V (Log 3)
     // Current: 0.01A (Log -2) ~ 10A (Log 1)
     float min_log = (harmonicsSource == 0) ? 0.0 : -2.0; 
     float max_log = (harmonicsSource == 0) ? 3.0 : 1.0;
     float log_range = max_log - min_log;

     // Loop for indices 1 to 8 (Corresponds to 1st, 3rd, 5th... in dataPtr)
     // dataPtr[1] is Fundamental (100%), dataPtr[2] is 3rd harmonic...
     for (int i = 1; i <= 8; i++) {
        float percent = dataPtr[i];
        // Calculate Absolute Value
        float val_abs = (percent / 100.0) * fundamental_rms;
        
        int bar_h = 0;
        if (val_abs > 0) {
            float val_log = log10(val_abs);
            
            // Normalize Height based on Log Scale
            // y = (val_log - min) / range * max_height
            float normalized = (val_log - min_log) / log_range;
            
            if (normalized > 1.0) normalized = 1.0;
            if (normalized < 0.0) normalized = 0.0; // Below min threshold
            
            bar_h = (int)(normalized * graph_h);
        } else {
            bar_h = 1; // Minimum visible
        }
        
        // Draw Bar if changed
        if (bar_h != prev_bar_heights[i]) {
            int x_pos = graph_x + (i - 1) * bar_w + 2;
            int prev_y = graph_y + graph_h - prev_bar_heights[i];
            int curr_y = graph_y + graph_h - bar_h;
            
            // Color: V=Blue, I=Orange
            uint16_t color = (harmonicsSource == 0) ? COLOR_BLUE : COLOR_ORANGE;
            
            // Clear old bar area (background)
            tft.fillRect(x_pos, graph_y, bar_w - 4, graph_h, COLOR_BACKGROUND); // Clear full column
            
            // Draw Grid Lines back in the cleared area (Optional polish, or just draw bar)
            // Simple approach: Just draw the bar. Grid lines behind bars are usually occluded.
            
            // Draw New Bar
            tft.fillRect(x_pos, curr_y, bar_w - 4, bar_h, color);
            
            prev_bar_heights[i] = bar_h;
        }
     }
  } else {
     // [Mod] Text View: 2-Column Grid
     tft.setTextSize(2);
     tft.setTextColor(COLOR_TEXT_PRIMARY);
     
     // Column X positions
     int col1_x = 20; 
     int col2_x = 175; 
     int start_y = 90; 
     int line_h = 25;
     
     // Loop 1 to 4 (Left Col), 5 to 8 (Right Col)
     for (int i = 1; i <= 8; i++) {
         if (abs(dataPtr[i] - prev_text_vals[i]) > 0.05 || screenNeedsRedraw) {
             int col_x = (i <= 4) ? col1_x : col2_x;
             int row_idx = (i <= 4) ? (i - 1) : (i - 5);
             int y = start_y + (row_idx * line_h);
             
             float val_abs = (dataPtr[i] / 100.0) * fundamental_rms;
             
             char buff_pct[8]; 
             if (dataPtr[i] >= 100.0) dtostrf(dataPtr[i], 3, 0, buff_pct); // "100"
             else dtostrf(dataPtr[i], 4, 1, buff_pct); // " 5.2"
             
             char buff_abs[8];
             // Adaptive format for Abs
             if (val_abs >= 10.0) dtostrf(val_abs, 4, 0, buff_abs);
             else if (val_abs >= 1.0) dtostrf(val_abs, 4, 1, buff_abs);
             else dtostrf(val_abs, 4, 2, buff_abs);

             tft.fillRect(col_x, y, 140, line_h, COLOR_BACKGROUND);
             tft.setCursor(col_x, y);
             
             // Label: "1:" or "3:"
             int harmonic_order = (i==1) ? 1 : (2*i - 1);
             tft.print(harmonic_order); tft.print(":");
             
             // Values
             tft.setTextColor((harmonicsSource==0)?COLOR_BLUE:COLOR_ORANGE);
             tft.print(buff_abs); 
             tft.print((harmonicsSource==0)?"V":"A");
             
             tft.setTextColor(COLOR_TEXT_SECONDARY);
             tft.setTextSize(1);
             tft.print("("); tft.print(buff_pct); tft.print("%)");
             tft.setTextSize(2);
             tft.setTextColor(COLOR_TEXT_PRIMARY);

             prev_text_vals[i] = dataPtr[i];
         }
     }
  }
}

void displaySettingsCalibManualValues() {
  int y_positions[] = {50, 85, 120};
  if (prev_calib_selection != calib_selection) {
    if (prev_calib_selection != -1) tft.fillRect(10, y_positions[prev_calib_selection], 15, 18, COLOR_BACKGROUND); 
    tft.setTextColor(COLOR_RED); tft.setTextSize(2);
    tft.setCursor(10, y_positions[calib_selection]); tft.print(">");
    prev_calib_selection = calib_selection;
  }
  printTFTValue(190, 50, V_MULTIPLIER, prev_V_MULTIPLIER, 2, COLOR_BLUE, " x"); 
  printTFTValue(190, 85, I_MULTIPLIER, prev_I_MULTIPLIER, 2, COLOR_ORANGE, " x"); 
  printTFTValue(190, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 
  prev_V_MULTIPLIER = V_MULTIPLIER; prev_I_MULTIPLIER = I_MULTIPLIER; prev_setting_step_index = setting_step_index;
}

void runAutoCalib() {
  if (auto_calib_step == 1) {
     measureOffsets();
     auto_calib_step = 2;
     screenNeedsRedraw = true;
  } else if (auto_calib_step == 3) {
      printTFTValue(120, 50, temp_true_v, prev_temp_true_v, 1, COLOR_BLUE, " V");
      printTFTValue(120, 85, temp_true_i, prev_temp_true_i, 2, COLOR_ORANGE, " A");
      printTFTValue(120, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 
      prev_temp_true_v = temp_true_v; prev_temp_true_i = temp_true_i; prev_setting_step_index = setting_step_index;
      
      int y_pos[] = {50, 85, 120};
      static int prev_sel = -1;
      if (calib_selection != prev_sel) {
         if(prev_sel != -1) tft.fillRect(10, y_pos[prev_sel], 15, 20, COLOR_BACKGROUND);
         tft.setTextColor(COLOR_RED); tft.setCursor(10, y_pos[calib_selection]); tft.print(">");
         prev_sel = calib_selection;
      }
  } else if (auto_calib_step == 5) {
      float eff_V_mult = BASE_V_CALIB_RMS * V_MULTIPLIER;
      float eff_I_mult = BASE_I_CALIB_RMS * I_MULTIPLIER;
      static float prev_disp_v = -1;
      static float prev_disp_i = -1;
      printTFTValue(60, 80, V_rms, prev_disp_v, 1, COLOR_BLUE, " V");
      printTFTValue(60, 110, I_rms, prev_disp_i, 2, COLOR_ORANGE, " A");
      prev_disp_v = V_rms; prev_disp_i = I_rms;
  }
}

void displaySettingsProtectValues() {
  int y_positions[] = {70, 110};
  if (prev_protect_selection != protect_selection) {
    if (prev_protect_selection != -1) tft.fillRect(10, y_positions[prev_protect_selection], 15, 18, COLOR_BACKGROUND); 
    tft.setTextColor(COLOR_RED); tft.setTextSize(2);
    tft.setCursor(10, y_positions[protect_selection]); tft.print(">");
    prev_protect_selection = protect_selection;
  }
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 
  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD; prev_setting_step_index = setting_step_index;
}

void runRelayControl() {
  if (relay1_state != prev_r1_state || relay2_state != prev_r2_state || screenNeedsRedraw) {
    String r1_status = relay1_state ? "ON" : "OFF";
    String r2_status = relay2_state ? "ON" : "OFF";
    
    if (currentScreen == SCREEN_RELAY_CONTROL) {
        drawButton(20, 70, 280, 40, "Relay 1: " + r1_status);
        drawButton(20, 130, 280, 40, "Relay 2: " + r2_status);
    }
    prev_r1_state = relay1_state; prev_r2_state = relay2_state;
  }
}

void runSettingsTheme() {}
void runPresetScreen() {}

// [Mod] 타이머 값 동적 표시 (간소화된 버전)
void displaySettingsTimerValues() {
  // Target Relay Toggle Button (상단 중앙 20, 45, 280, 40 통합됨 in Logic, but here we redraw)
  if (timer_target_relay != prev_timer_target_relay || screenNeedsRedraw) {
     String label = "Target: ";
     if (timer_target_relay == 1) label += "Relay 1";
     else if (timer_target_relay == 2) label += "Relay 2";
     else label += "BOTH";
     drawButton(60, 45, 200, 40, label);
     prev_timer_target_relay = timer_target_relay;
  }

  // Time Display (중앙)
  // 만약 타이머가 Active라면 남은 시간 표시, Idle이라면 설정 시간 표시
  uint32_t display_time = is_timer_active ? timer_seconds_left : temp_timer_setting_seconds;
  static uint32_t prev_display_time = 0xFFFFFFFF;
  static bool prev_active_state = false;

  if (display_time != prev_display_time || is_timer_active != prev_active_state || screenNeedsRedraw) {
     uint16_t hours = display_time / 3600;
     uint16_t minutes = (display_time % 3600) / 60;
     uint8_t seconds = display_time % 60;
     
     char buffer[20]; sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
     
     // 배경 지우기
     tft.fillRect(60, 95, 200, 40, COLOR_BACKGROUND);
     
     tft.setTextSize(4);
     tft.setTextColor(is_timer_active ? COLOR_GREEN : COLOR_ORANGE);
     int16_t x1, y1; uint16_t w, h;
     tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w, &h);
     tft.setCursor((SCREEN_WIDTH - w)/2, 95 + 5); 
     tft.print(buffer);
     
     prev_display_time = display_time;
     prev_active_state = is_timer_active;
  }

  // Step Button (STEP 텍스트 대신 "Step: 10s" 표시)
  if (timer_step_index != prev_timer_step_index || screenNeedsRedraw) {
     String stepLabel = "Step: " + String(TIMER_STEP_LABELS[timer_step_index]);
     // 공간 확보를 위해 짧게 표시: "10s"
     drawButton(90, 150, 60, 40, TIMER_STEP_LABELS[timer_step_index]);
     prev_timer_step_index = timer_step_index;
  }

  // START/STOP Action Button
  static bool prev_btn_active = false;
  if (is_timer_active != prev_btn_active || screenNeedsRedraw) {
     String actionLabel = is_timer_active ? "STOP" : "START";
     uint16_t color = is_timer_active ? COLOR_RED : COLOR_GREEN;
     
     // Custom drawButton with color override logic would be better, 
     // but here manual draw for custom color
     tft.fillRoundRect(230, 150, 80, 40, 8, color);
     tft.drawRoundRect(230, 150, 80, 40, 8, COLOR_BUTTON_OUTLINE);
     tft.setTextColor(ILI9341_WHITE);
     tft.setTextSize(2);
     tft.setCursor(240, 162); 
     tft.print(actionLabel);
     
     prev_btn_active = is_timer_active;
  }
}