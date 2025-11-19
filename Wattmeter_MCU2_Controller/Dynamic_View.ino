/*
 * ==============================================================================
 * 파일명: 3. Dynamic_View.ino
 * 버전: v210 (Clean Cont. Mode with Immediate Refresh)
 * 설명: 
 * - [Mod] Cont. Mode: Replaced Rolling Logic with Immediate Frame Capture (Like Trig mode w/o wait)
 * - [Mod] Drawing: Added explicit line clearing using previous frame buffer to prevent flicker
 * - [Fix] Harmonics & Timer logic retained
 * ==============================================================================
 */

// 외부 전역 변수 참조 (Controller에서 정의됨)
extern bool prev_is_timer_active; 

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
  if (screenNeedsRedraw) {
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
  }

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
       statusColor = COLOR_BUTTON; 
       btnText = "TURN ON";
    }

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

// --- [Mod] Waveform Axis & Units Update ---
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 

  tft.setTextColor(COLOR_BLUE);
  dtostrf(plot1_axis_max, 3, 0, buffer);
  
  String unit1 = "V";
  if (waveformPlotType == 1) unit1 = "W";       
  else if (waveformPlotType == 2) unit1 = "A";  
  
  tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print(unit1); 
  tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print(unit1);
    
  if (waveformPlotType == 2) {
    tft.setTextColor(COLOR_ORANGE);
    dtostrf(plot2_axis_max, 3, 0, buffer); 
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
  } 
  else {
    tft.setTextColor(COLOR_ORANGE);
    String unit2 = "A";
    if (waveformPlotType == 1) unit2 = "Vr"; 
    
    if (plot2_axis_max < 1.0) { 
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

// --- [Fix] Harmonics Y-Axis Update Logic ---
void updateHarmonicsYAxisLabels() {
   tft.fillRect(0, 45, 39, 150, COLOR_BACKGROUND);
   displayHarmonicsScreenStatic();
}

// --- [Mod] Triggered Scope & Rolling Logic ---
void runCombinedWaveformLoop() {
  if (isWaveformFrozen) return;

  static float v_buf[PLOT_WIDTH];
  static float p2_buf[PLOT_WIDTH];
  static float p3_buf[PLOT_WIDTH];
  
  const int LAG_SIZE = 40;
  float eff_V_mult = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float eff_I_mult = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float eff_V_off = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float eff_I_off = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;
  
  // [Mod] 변경된 딜레이 값 적용 (MCU2_Controller에서 수정됨)
  int delay_us = WAVEFORM_DELAYS_US[waveformPeriodIndex];
  int center = ADC_MIDPOINT;
  
  bool triggered = false;

  // ============================================================
  // 1. 샘플링 (Sampling) 영역
  // ============================================================
  
  // [Mod] Cont. Mode (연속 모드) 로직 전면 재작성
  // - 기존 롤링 방식 제거 -> Trig 모드와 동일한 "전체 프레임 캡처" 방식 적용
  // - 제로 크로싱 대기 없이 즉시 루프 시작
  if (waveformTriggerMode == 0) {
      // Q 계산을 위한 Lag Buffer 채우기 (Trig 모드 로직 차용)
      float local_lag_buf[LAG_SIZE];
      int lag_head = 0;

      // 버퍼 초기화 (Type 1: P/Q 모드일 때 필요)
      if (waveformPlotType == 1) {
          for(int k=0; k<LAG_SIZE; k++) {
              int r_v = analogRead(PIN_ADC_V);
              float v = (r_v - center) * eff_V_mult + eff_V_off;
              local_lag_buf[lag_head] = v;
              lag_head = (lag_head + 1) % LAG_SIZE;
              delayMicroseconds(delay_us);
          }
      }

      // 전체 프레임 즉시 샘플링
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

          if (waveformPlotType == 0) { // V/I
             v_buf[i] = v;
             p2_buf[i] = cur;
          } 
          else if (waveformPlotType == 1) { // P/Q
             float p = v * cur;
             v_buf[i] = p; 
             
             local_lag_buf[lag_head] = v;
             int lag_read_idx = (lag_head + 1) % LAG_SIZE;
             float v_lag = local_lag_buf[lag_read_idx];
             lag_head = lag_read_idx;
             
             float q = v_lag * cur;
             p2_buf[i] = q;
          } 
          else { // I/I1/I2
             v_buf[i] = cur;
             p2_buf[i] = cur1; 
             p3_buf[i] = cur2;
          }
          
          // 사용자 입력 및 통신 체크 (응답성 유지)
          checkSerialInput();
          checkTouchInput();
          if (screenNeedsRedraw) return;

          while (micros() - t0 < delay_us);
      }
  } 
  // [Keep] Trig. / Single Mode (기존 로직 유지 - 제로 크로싱 대기 포함)
  else {
      unsigned long trigger_start = millis();
      int hyst = 50;
      int trig_state = 0;
      
      // 제로 크로싱 대기 (Timeout 50ms)
      while (millis() - trigger_start < 50) {
         checkTouchInput();
         if (screenNeedsRedraw) return;
         int raw_v = analogRead(PIN_ADC_V);
         if (trig_state == 0) { if (raw_v < (center - hyst)) trig_state = 1; } 
         else if (trig_state == 1) { if (raw_v > (center + hyst)) { triggered = true; break; } }
      }

      // Lag Buffer Pre-fill
      float local_lag_buf[LAG_SIZE];
      int lag_head = 0;
      for(int k=0; k<LAG_SIZE; k++) {
          int r_v = analogRead(PIN_ADC_V);
          float v = (r_v - center) * eff_V_mult + eff_V_off;
          local_lag_buf[lag_head] = v;
          lag_head = (lag_head + 1) % LAG_SIZE;
          delayMicroseconds(delay_us);
      }
      
      // 프레임 샘플링
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

        if (waveformPlotType == 0) { 
           v_buf[i] = v;
           p2_buf[i] = cur;
        } 
        else if (waveformPlotType == 1) { 
           float p = v * cur;
           v_buf[i] = p; 
           
           local_lag_buf[lag_head] = v;
           int lag_read_idx = (lag_head + 1) % LAG_SIZE;
           float v_lag = local_lag_buf[lag_read_idx];
           lag_head = lag_read_idx;
           
           float q = v_lag * cur;
           p2_buf[i] = q;
        } 
        else { 
           v_buf[i] = cur;
           p2_buf[i] = cur1; p3_buf[i] = cur2;
        }
        while (micros() - t0 < delay_us);
      }
  }
  
  // ============================================================
  // 2. 오토 레인징 (Auto-ranging) - 기존 유지
  // ============================================================
  float max_val_p1 = 0.0, max_val_p2 = 0.0;
  for(int i=0; i<PLOT_WIDTH; i++) {
     if (abs(v_buf[i]) > max_val_p1) max_val_p1 = abs(v_buf[i]);
     if (abs(p2_buf[i]) > max_val_p2) max_val_p2 = abs(p2_buf[i]);
     if (waveformPlotType == 2 && abs(p3_buf[i]) > max_val_p1) max_val_p1 = abs(p3_buf[i]);
  }
  if (waveformPlotType == 2) max_val_p2 = max_val_p1;

  static int range_idx_v = NUM_V_RANGES - 1;
  static int range_idx_i = NUM_I_RANGES - 1;
  static int range_idx_p = NUM_P_RANGES - 1;
  bool rangeChanged = false;

  auto updateRange = [&](int &idx, float peak, const float* ranges, int count) {
      bool changed = false;
      if (peak > ranges[idx]) {
          while (idx < count - 1 && peak > ranges[idx]) idx++;
          changed = true;
      } else if (idx > 0) {
          if (peak < (ranges[idx-1] * 0.8)) { idx--; changed = true; }
      }
      return changed;
  };

  if (waveformPlotType == 0) { 
     if (updateRange(range_idx_v, max_val_p1, V_RANGES, NUM_V_RANGES)) rangeChanged = true;
     if (updateRange(range_idx_i, max_val_p2, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     plot1_axis_max = V_RANGES[range_idx_v]; plot2_axis_max = I_RANGES[range_idx_i];
  } 
  else if (waveformPlotType == 1) { 
     float max_pq = max(max_val_p1, max_val_p2);
     if (updateRange(range_idx_p, max_pq, P_RANGES, NUM_P_RANGES)) rangeChanged = true;
     plot1_axis_max = P_RANGES[range_idx_p]; plot2_axis_max = P_RANGES[range_idx_p];
  } 
  else { 
     if (updateRange(range_idx_i, max_val_p1, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     plot1_axis_max = I_RANGES[range_idx_i]; plot2_axis_max = I_RANGES[range_idx_i]; plot3_axis_max = I_RANGES[range_idx_i];
  }

  if (rangeChanged) {
     updateYAxisLabels();
     tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  }
  
  // ============================================================
  // 3. 그리기 (Drawing) - Clearing 로직 강화
  // ============================================================
  float scale1 = (plot1_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
  float scale2 = (plot2_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  float scale3 = (plot3_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot3_axis_max); 

  // [Mod] 이전 프레임의 Y좌표 추적을 위한 지역 변수
  // last_frame_y_plot 배열은 루프 돌면서 즉시 갱신되므로, 
  // 지우기(Clearing) 단계에서 필요한 '이전 X점의 Y값'을 보존해야 함.
  int py1_old = PLOT_Y_CENTER, py2_old = PLOT_Y_CENTER, py3_old = PLOT_Y_CENTER; 
  int py1_new = PLOT_Y_CENTER, py2_new = PLOT_Y_CENTER, py3_new = PLOT_Y_CENTER;

  tft.startWrite();
  for (int i = 0; i < PLOT_WIDTH; i++) {
     int x = PLOT_X_START + i;
     
     // 현재 프레임의 이전 값 (지우기 용) -> 루프 시작 시점에 배열에서 읽어야 함
     int cy1_old = last_frame_y_plot1[i];
     int cy2_old = last_frame_y_plot2[i];
     int cy3_old = last_frame_y_plot3[i];

     // 새로운 Y값 계산
     int y1 = constrain(PLOT_Y_CENTER - (int)(v_buf[i] * scale1), PLOT_Y_START, PLOT_Y_END);
     int y2 = constrain(PLOT_Y_CENTER - (int)(p2_buf[i] * scale2), PLOT_Y_START, PLOT_Y_END);
     int y3 = PLOT_Y_CENTER;
     if (waveformPlotType == 2) y3 = constrain(PLOT_Y_CENTER - (int)(p3_buf[i] * scale3), PLOT_Y_START, PLOT_Y_END);

     // 그리기 (첫 점 제외)
     if (i > 0) {
         int xp = x - 1;
         
         // [Clear] 이전 선 지우기 (이전 프레임의 선)
         // 이전 점(xp, py_old) -> 현재 점(x, cy_old)
         if (cy1_old != PLOT_Y_CENTER || py1_old != PLOT_Y_CENTER)
            tft.drawLine(xp, py1_old, x, cy1_old, COLOR_BACKGROUND);
         if (cy2_old != PLOT_Y_CENTER || py2_old != PLOT_Y_CENTER)
            tft.drawLine(xp, py2_old, x, cy2_old, COLOR_BACKGROUND);
         if (waveformPlotType == 2) {
            if (cy3_old != PLOT_Y_CENTER || py3_old != PLOT_Y_CENTER)
                tft.drawLine(xp, py3_old, x, cy3_old, COLOR_BACKGROUND);
         }

         // [Draw] 새로운 선 그리기
         // 이전 점(xp, py_new) -> 현재 점(x, y)
         tft.drawLine(xp, py1_new, x, y1, COLOR_BLUE);
         tft.drawLine(xp, py2_new, x, y2, COLOR_ORANGE);
         if (waveformPlotType == 2) tft.drawLine(xp, py3_new, x, y3, COLOR_RED);
     }
     
     // 배열 갱신 (다음 프레임을 위해 현재 값 저장)
     last_frame_y_plot1[i] = y1; 
     last_frame_y_plot2[i] = y2; 
     last_frame_y_plot3[i] = y3;

     // 다음 루프를 위해 현재 값을 '이전 값' 변수에 저장
     py1_old = cy1_old; py2_old = cy2_old; py3_old = cy3_old;
     py1_new = y1;      py2_new = y2;      py3_new = y3;
  }
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  tft.endWrite();
  
  if (waveformTriggerMode == 2 && triggered) isWaveformFrozen = true;
  
  checkSerialInput(); 
}

// [Mod] HARMONICS 화면 값 업데이트 (로그 스케일 절대값, 색상 분리, 그리드 갱신)
void displayHarmonicsScreenValues() {
  // 소스 변경 시 Y축 라벨을 명시적으로 지우고 다시 그림
  if (harmonicsSrcLabel != prev_harmonicsSrcLabel) {
     drawButton(10, 200, 90, 35, harmonicsSrcLabel);
     prev_harmonicsSrcLabel = harmonicsSrcLabel;
     
     // [Fix] Y축 라벨 잔상 제거를 위해 갱신 함수 호출
     updateHarmonicsYAxisLabels();
  }
  
  String currRunLabel = isHarmonicsFrozen ? "RUN" : "HOLD";
  if (currRunLabel != prev_harmonicsRunLabel) {
     drawButton(110, 200, 100, 35, currRunLabel);
     prev_harmonicsRunLabel = currRunLabel;
  }

  if (isHarmonicsFrozen) return;

  float* dataPtr = (harmonicsSource == 0) ? v_harmonics : i_harmonics;
  float fundamental_rms = (harmonicsSource == 0) ? V_rms : I_rms;
  
  static int prev_bar_heights[16]; 
  static float prev_text_vals[8];  

  if (harmonicsViewMode == 0) {
     int graph_x = 40; int graph_y = 45; int graph_h = 145;
     int bar_w = 270 / 8; 
     float min_log = (harmonicsSource == 0) ? 0.0 : -2.0; 
     float max_log = (harmonicsSource == 0) ? 3.0 : 1.0;
     float log_range = max_log - min_log;

     for (int i = 1; i <= 8; i++) {
        float percent = dataPtr[i];
        float val_abs = (percent / 100.0) * fundamental_rms;
        
        int bar_h = 0;
        if (val_abs > 0) {
            float val_log = log10(val_abs);
            float normalized = (val_log - min_log) / log_range;
            if (normalized > 1.0) normalized = 1.0;
            if (normalized < 0.0) normalized = 0.0; 
            bar_h = (int)(normalized * graph_h);
        } else bar_h = 1; 
        
        if (bar_h != prev_bar_heights[i]) {
            int x_pos = graph_x + (i - 1) * bar_w + 2;
            int curr_y = graph_y + graph_h - bar_h;
            uint16_t color = (harmonicsSource == 0) ? COLOR_BLUE : COLOR_ORANGE;
            tft.fillRect(x_pos, graph_y, bar_w - 4, graph_h, COLOR_BACKGROUND); 
            tft.fillRect(x_pos, curr_y, bar_w - 4, bar_h, color);
            prev_bar_heights[i] = bar_h;
        }
     }
  } else {
     tft.setTextSize(2); tft.setTextColor(COLOR_TEXT_PRIMARY);
     int col1_x = 20; int col2_x = 175; int start_y = 90; int line_h = 25;
     
     for (int i = 1; i <= 8; i++) {
         if (abs(dataPtr[i] - prev_text_vals[i]) > 0.05 || screenNeedsRedraw) {
             int col_x = (i <= 4) ? col1_x : col2_x;
             int row_idx = (i <= 4) ? (i - 1) : (i - 5);
             int y = start_y + (row_idx * line_h);
             
             float val_abs = (dataPtr[i] / 100.0) * fundamental_rms;
             char buff_pct[8]; 
             if (dataPtr[i] >= 100.0) dtostrf(dataPtr[i], 3, 0, buff_pct); 
             else dtostrf(dataPtr[i], 4, 1, buff_pct); 
             char buff_abs[8];
             if (val_abs >= 10.0) dtostrf(val_abs, 4, 0, buff_abs);
             else if (val_abs >= 1.0) dtostrf(val_abs, 4, 1, buff_abs);
             else dtostrf(val_abs, 4, 2, buff_abs);

             tft.fillRect(col_x, y, 140, line_h, COLOR_BACKGROUND);
             tft.setCursor(col_x, y);
             int harmonic_order = (i==1) ? 1 : (2*i - 1);
             tft.print(harmonic_order); tft.print(":");
             tft.setTextColor((harmonicsSource==0)?COLOR_BLUE:COLOR_ORANGE);
             tft.print(buff_abs); tft.print((harmonicsSource==0)?"V":"A");
             tft.setTextColor(COLOR_TEXT_SECONDARY); tft.setTextSize(1);
             tft.print("("); tft.print(buff_pct); tft.print("%)");
             tft.setTextSize(2); tft.setTextColor(COLOR_TEXT_PRIMARY);
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

// [Mod] 타이머 값 동적 표시 (버튼 가시성 수정됨)
void displaySettingsTimerValues() {
  if (timer_target_relay != prev_timer_target_relay || screenNeedsRedraw) {
     String label = "Target: ";
     if (timer_target_relay == 1) label += "Relay 1";
     else if (timer_target_relay == 2) label += "Relay 2";
     else label += "BOTH";
     drawButton(60, 45, 200, 40, label);
     prev_timer_target_relay = timer_target_relay;
  }

  uint32_t display_time = is_timer_active ? timer_seconds_left : temp_timer_setting_seconds;
  static uint32_t prev_display_time = 0xFFFFFFFF;
  static bool prev_active_state = false;

  if (display_time != prev_display_time || is_timer_active != prev_active_state || screenNeedsRedraw) {
     uint16_t hours = display_time / 3600;
     uint16_t minutes = (display_time % 3600) / 60;
     uint8_t seconds = display_time % 60;
     
     char buffer[20]; sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
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

  if (timer_step_index != prev_timer_step_index || screenNeedsRedraw) {
     drawButton(90, 150, 60, 40, TIMER_STEP_LABELS[timer_step_index]);
     prev_timer_step_index = timer_step_index;
  }

  // [Fix] Start/Stop 버튼이 처음 화면에 진입했을 때 그려지지 않는 문제 수정
  // - 기존에는 함수 내부의 static 변수(prev_btn_active)가 false로 초기화되어 있어, 
  //   is_timer_active도 false일 경우 상태 변화가 감지되지 않아 버튼이 그려지지 않았음.
  // - Controller의 Static View에서 초기화해주는 전역 변수 'prev_is_timer_active'를 사용하여
  //   화면 진입 시 강제로 상태 불일치를 유발, 버튼이 무조건 그려지도록 수정함.
  if (is_timer_active != prev_is_timer_active || screenNeedsRedraw) {
     String actionLabel = is_timer_active ? "STOP" : "START";
     uint16_t color = is_timer_active ? COLOR_RED : COLOR_GREEN;
     
     // 버튼 영역 좌표: 230, 150, 80, 40 (다른 버튼과 겹치지 않음)
     tft.fillRoundRect(230, 150, 80, 40, 8, color);
     tft.drawRoundRect(230, 150, 80, 40, 8, COLOR_BUTTON_OUTLINE);
     tft.setTextColor(ILI9341_WHITE);
     tft.setTextSize(2);
     tft.setCursor(240, 162); 
     tft.print(actionLabel);
     
     // 상태 동기화
     prev_is_timer_active = is_timer_active;
  }
}