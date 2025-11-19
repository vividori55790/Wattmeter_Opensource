/*
 * ==============================================================================
 * 파일명: 3. View_Dynamic.ino
 * 버전: v205 (Dual Y-Axis Logic, Log Harmonics Calculation, Timer UI)
 * 설명: 
 * - 파형 화면: 좌측(Plot1), 우측(Plot2) Y축 라벨 그리기
 * - 고조파 화면: 기본값(RMS) * 비율 = 절대값 계산 후 로그 매핑, Source별 색상 적용
 * - 타이머 화면: 단일 버튼(Toggle Target) 및 통합 숫자 표시
 * - 네트워크 화면: 버튼 텍스트 OFF > WAIT > ON 로직 적용
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

    // 버튼 (기존 터치 영역 55~265, 45~105은 상태 표시바이고, 실제 토글 버튼은 터치 로직에 있음.
    // 여기서는 버튼이 상태바 역할도 하므로 텍스트만 업데이트)
    // 요구사항: "와이파이 연결 버튼의 색깔을 바꾸지 말고 표시되는 텍스트만 변경해서 출력해줘."
    // 하지만 현재 UI 구조상 상태바가 곧 버튼임. 
    
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

float findAxisStep(float peak, const float* steps, int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    if (peak <= steps[i]) return steps[i]; 
  }
  return steps[num_steps - 1]; 
}

// --- [Mod] 동적 Y축 라벨 (좌/우 표시) ---
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_CENTER - 15, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_CENTER + 5, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 

  // [Mod] 좌측(Plot1), 우측(Plot2) 모두 라벨링
  if (waveformPlotType == 0) { // V(Left)/I(Right)
    // Left Axis (V)
    tft.setTextColor(COLOR_BLUE);
    dtostrf(plot1_axis_max, 3, 0, buffer); 
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("V"); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("V");
    
    // Right Axis (I)
    tft.setTextColor(COLOR_ORANGE);
    if (plot2_axis_max < 1.0) { 
      dtostrf(plot2_axis_max * 1000, 3, 0, buffer);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("mA");
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("mA");
    } else { 
      dtostrf(plot2_axis_max, 3, 1, buffer);
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
      tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
    }

  } else if (waveformPlotType == 1) { // P/Q (Both W/Var)
    // Left (P)
    tft.setTextColor(COLOR_BLUE);
    dtostrf(plot1_axis_max, 3, 0, buffer); 
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("W"); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("W");
    
    // Right (Q)
    tft.setTextColor(COLOR_ORANGE);
    dtostrf(plot2_axis_max, 3, 0, buffer); 
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("Vr"); 
    tft.setCursor(PLOT_X_END + 2, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("Vr");
  } else { // I/I1/I2 (All Amps)
    tft.setTextColor(COLOR_TEXT_PRIMARY); 
    if (plot1_axis_max < 1.0) { 
      dtostrf(plot1_axis_max * 1000, 3, 0, buffer);
      tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("mA");
      tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("mA");
    } else { 
      dtostrf(plot1_axis_max, 3, 1, buffer);
      tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
      tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
    }
  }
}

void runCombinedWaveformLoop() {
  if (isWaveformFrozen) return;

  const int LAG_BUF_SIZE = 40; 
  static float v_lag_buffer[LAG_BUF_SIZE];
  static int buf_head = 0;

  float scale_p1, scale_p2, scale_p3; 
  float new_frame_p1_peak = 0.0, new_frame_p2_peak = 0.0, new_frame_p3_peak = 0.0;
  int new_y_plot1[PLOT_WIDTH], new_y_plot2[PLOT_WIDTH], new_y_plot3[PLOT_WIDTH];

  if (waveformPlotType == 0) { 
    scale_p1 = (plot1_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = (plot2_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  } else if (waveformPlotType == 1) { 
    scale_p1 = (plot1_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = (plot2_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  } else { 
    scale_p1 = (plot1_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = scale_p1; scale_p3 = scale_p1;
  }

  float eff_V_mult = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float eff_I_mult = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float eff_V_off = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float eff_I_off = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  if (waveformTriggerMode == 1) {
      long timeout = micros() + 16000; 
      int prev_v = analogRead(PIN_ADC_V) - ADC_MIDPOINT;
      while(micros() < timeout) {
          int curr_v = analogRead(PIN_ADC_V) - ADC_MIDPOINT;
          if (prev_v < 0 && curr_v >= 0) break; 
          prev_v = curr_v;
      }
  }

  for (int i = 0; i < PLOT_WIDTH; i++) {
    checkTouchInput(); 
    checkSerialInput(); 

    if (screenNeedsRedraw || warningActive) {
       if(screenNeedsRedraw) isWaveformFrozen = false;
       return;
    }

    int raw_v = analogRead(PIN_ADC_V);
    int raw_i = analogRead(PIN_ADC_I);
    int raw_i1 = analogRead(PIN_ADC_I1);
    int raw_i2 = analogRead(PIN_ADC_I2);

    float v_inst = (raw_v - ADC_MIDPOINT) * eff_V_mult + eff_V_off;
    float i_inst = (raw_i - ADC_MIDPOINT) * eff_I_mult - eff_I_off;
    float i1_inst = (raw_i1 - ADC_MIDPOINT) * eff_I_mult - eff_I_off;
    float i2_inst = (raw_i2 - ADC_MIDPOINT) * eff_I_mult - eff_I_off;

    float val1 = 0, val2 = 0, val3 = 0;

    if (waveformPlotType == 0) { // V/I
       val1 = v_inst;
       val2 = i_inst;
    } 
    else if (waveformPlotType == 1) { // P/Q
       val1 = v_inst * i_inst; 
       v_lag_buffer[buf_head] = v_inst;
       int lag_idx = (buf_head + 1) % LAG_BUF_SIZE;
       float v_lagged = v_lag_buffer[lag_idx];
       buf_head = lag_idx;
       val2 = v_lagged * i_inst; 
    } 
    else { // I/I1/I2
       val1 = i_inst;
       val2 = i1_inst;
       val3 = i2_inst;
    }

    if (abs(val1) > new_frame_p1_peak) new_frame_p1_peak = abs(val1);
    if (abs(val2) > new_frame_p2_peak) new_frame_p2_peak = abs(val2);
    if (abs(val3) > new_frame_p3_peak) new_frame_p3_peak = abs(val3);

    new_y_plot1[i] = constrain(PLOT_Y_CENTER - (int)(val1 * scale_p1), PLOT_Y_START, PLOT_Y_END);
    new_y_plot2[i] = constrain(PLOT_Y_CENTER - (int)(val2 * scale_p2), PLOT_Y_START, PLOT_Y_END);
    new_y_plot3[i] = constrain(PLOT_Y_CENTER - (int)(val3 * scale_p3), PLOT_Y_START, PLOT_Y_END);

    delayMicroseconds(WAVEFORM_DELAYS_US[waveformPeriodIndex]); 
  }

  tft.startWrite(); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  for (int i = 1; i < PLOT_WIDTH; i++) {
      int x_curr = PLOT_X_START + i;
      int x_prev = PLOT_X_START + i - 1;
      
      if(last_frame_y_plot1[i] != PLOT_Y_CENTER || last_frame_y_plot1[i-1] != PLOT_Y_CENTER) 
          tft.drawLine(x_prev, last_frame_y_plot1[i-1], x_curr, last_frame_y_plot1[i], COLOR_BACKGROUND); 
      tft.drawLine(x_prev, new_y_plot1[i-1], x_curr, new_y_plot1[i], COLOR_BLUE);
      
      if(last_frame_y_plot2[i] != PLOT_Y_CENTER || last_frame_y_plot2[i-1] != PLOT_Y_CENTER) 
          tft.drawLine(x_prev, last_frame_y_plot2[i-1], x_curr, last_frame_y_plot2[i], COLOR_BACKGROUND); 
      tft.drawLine(x_prev, new_y_plot2[i-1], x_curr, new_y_plot2[i], COLOR_ORANGE);

      if (waveformPlotType == 2) {
        if(last_frame_y_plot3[i] != PLOT_Y_CENTER || last_frame_y_plot3[i-1] != PLOT_Y_CENTER) 
            tft.drawLine(x_prev, last_frame_y_plot3[i-1], x_curr, last_frame_y_plot3[i], COLOR_BACKGROUND); 
        tft.drawLine(x_prev, new_y_plot3[i-1], x_curr, new_y_plot3[i], COLOR_RED);
      }
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
      last_frame_y_plot1[i] = new_y_plot1[i];
      last_frame_y_plot2[i] = new_y_plot2[i];
      last_frame_y_plot3[i] = new_y_plot3[i];
  }

  float new_p1_axis = plot1_axis_max;
  float new_p2_axis = plot2_axis_max;
  bool axisChanged = false;

  if (waveformPlotType == 0) { // V/I
    new_p1_axis = findAxisStep(new_frame_p1_peak, V_RANGES, NUM_V_RANGES);
    new_p2_axis = findAxisStep(new_frame_p2_peak, I_RANGES, NUM_I_RANGES);
  } else if (waveformPlotType == 1) { // P/Q
    new_p1_axis = findAxisStep(new_frame_p1_peak, P_RANGES, NUM_P_RANGES);
    new_p2_axis = findAxisStep(new_frame_p2_peak, P_RANGES, NUM_P_RANGES); 
  } else { // I/I1/I2
    float max_peak_all = max(new_frame_p1_peak, max(new_frame_p2_peak, new_frame_p3_peak));
    new_p1_axis = findAxisStep(max_peak_all, I_RANGES, NUM_I_RANGES);
    if (new_p1_axis != plot1_axis_max) axisChanged = true;
  }

  if (new_p1_axis != plot1_axis_max || new_p2_axis != plot2_axis_max || axisChanged) {
    plot1_axis_max = new_p1_axis;
    if (waveformPlotType != 2) plot2_axis_max = new_p2_axis;
    updateYAxisLabels();
  }
  
  if (waveformTriggerMode == 2 && !isWaveformFrozen) {
    isWaveformFrozen = true;
  }
}

// [Mod] HARMONICS 화면 값 업데이트 (로그 스케일 절대값, 색상 분리)
void displayHarmonicsScreenValues() {
  if (harmonicsSrcLabel != prev_harmonicsSrcLabel) {
     drawButton(10, 200, 90, 35, harmonicsSrcLabel);
     prev_harmonicsSrcLabel = harmonicsSrcLabel;
  }
  
  String currRunLabel = isHarmonicsFrozen ? "HOLD" : "RUN";
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
     int graph_x = 40;
     int graph_y = 45;
     int graph_h = 145;
     int bar_w = 270 / 15; 

     // [Mod] Y축 라벨 (Log Scale) 업데이트
     tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
     
     // 범위 설정: 전압(1V~1000V), 전류(0.01A~10A) - 둘 다 4 Decades
     float min_log = (harmonicsSource == 0) ? 0.0 : -2.0; // log10(1)=0, log10(0.01)=-2
     float max_log = (harmonicsSource == 0) ? 3.0 : 1.0;  // log10(1000)=3, log10(10)=1
     
     // 그리드 라벨 다시 그리기 (소스 변경 시 값 변경)
     if (screenNeedsRedraw || harmonicsSrcLabel != prev_harmonicsSrcLabel) {
        tft.fillRect(0, graph_y-10, 35, graph_h+20, COLOR_BACKGROUND); // 지우기
        int h_step = graph_h / 3; // 4 lines: 0, 1/3, 2/3, 1
        
        if (harmonicsSource == 0) { // Voltage: 1000, 100, 10, 1
           tft.setCursor(5, graph_y-3); tft.print("1kV");
           tft.setCursor(5, graph_y + h_step - 3); tft.print("100");
           tft.setCursor(5, graph_y + h_step*2 - 3); tft.print("10V");
           tft.setCursor(5, graph_y + graph_h - 3); tft.print("1V");
        } else { // Current: 10, 1, 0.1, 0.01
           tft.setCursor(5, graph_y-3); tft.print("10A");
           tft.setCursor(5, graph_y + h_step - 3); tft.print("1A");
           tft.setCursor(5, graph_y + h_step*2 - 3); tft.print("0.1");
           tft.setCursor(2, graph_y + graph_h - 3); tft.print(".01");
        }
     }

     for (int i = 1; i <= 15; i++) {
        // 절대값 계산 (Percent -> Abs)
        float val_abs = (dataPtr[i] / 100.0) * fundamental_rms;
        
        int bar_h = 0;
        if (val_abs > 0) {
            float val_log = log10(val_abs);
            // Normalize: (val_log - min) / (max - min)
            float normalized = (val_log - min_log) / (max_log - min_log);
            if (normalized > 1.0) normalized = 1.0;
            if (normalized < 0.0) normalized = 0.0;
            bar_h = (int)(normalized * graph_h);
        } else {
            bar_h = 1;
        }
        
        if (bar_h != prev_bar_heights[i]) {
            int x_pos = graph_x + (i - 1) * bar_w + 2;
            int prev_y = graph_y + graph_h - prev_bar_heights[i];
            int curr_y = graph_y + graph_h - bar_h;
            
            // [Mod] 색상 분리: 전압(파랑), 전류(주황)
            uint16_t color = (harmonicsSource == 0) ? COLOR_BLUE : COLOR_ORANGE;
            
            tft.fillRect(x_pos, graph_y, bar_w - 4, graph_h, COLOR_BACKGROUND);
            tft.fillRect(x_pos, curr_y, bar_w - 4, bar_h, color);
            
            prev_bar_heights[i] = bar_h;
        }
     }
  } else {
     // Text View
     tft.setTextSize(2);
     tft.setTextColor(COLOR_TEXT_PRIMARY);
     int col1_x = 10; int start_y = 80; int line_h = 20;
     
     for (int i = 1; i <= 7; i++) {
        if (abs(dataPtr[i] - prev_text_vals[i]) > 0.1) {
            int y = start_y + (i - 1) * line_h;
            char buff_pct[10]; dtostrf(dataPtr[i], 5, 1, buff_pct); 
            
            // 절대값 표시도 추가 (공간이 되면)
            float val_abs = (dataPtr[i] / 100.0) * fundamental_rms;
            char buff_abs[10]; dtostrf(val_abs, 4, 1, buff_abs);

            tft.fillRect(col1_x, y, 300, line_h, COLOR_BACKGROUND);
            tft.setCursor(col1_x, y);
            tft.print(i); tft.print(":"); tft.print(buff_pct); tft.print("% / ");
            tft.print(buff_abs); tft.print((harmonicsSource==0)?"V":"A");

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
     drawButton(90, 150, 60, 40, stepLabel); // 기존 STEP 버튼 위치 재활용
     // 하지만 STEP 버튼 폭이 좁으므로(60) 텍스트가 길면 안됨.
     // 요청: "설정된 스텝 시간을 버튼에만 표시해주고"
     // 공간 확보를 위해 버튼 폭을 늘리거나 텍스트를 짧게 줄여야 함.
     // 여기서는 버튼 폭을 조정하지 않고 짧게 표시: "10s"
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