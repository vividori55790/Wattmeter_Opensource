/*
 * ==============================================================================
 * 파일명: 3. View_Dynamic.ino
 * 버전: v78 (Harmonics Graph/List Implementation)
 * 최종 수정: 2025-11-21
 *
 * [변경 사항 - v78]
 * - displayTHDScreenValues() 삭제.
 * - displayHarmonicsScreenValues() 신규 구현:
 * : Graph Mode (Log Scale Bar Chart)
 * : List Mode (Text Table for 1st-7th harmonics)
 * ==============================================================================
 */

// --- 헬퍼: 값 출력 (Float) ---
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  if (abs(value - prev_value) < (pow(10, -precision) / 2.0)) {
    return; 
  }
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
  if (value == prev_value) {
    return; 
  }
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

// --- 네트워크 설정 화면 동적 갱신 ---
void runSettingsNetwork() {
  static bool prev_isWifiEnabled = !isWifiEnabled; 
  static bool prev_isWifiConnected = !isWifiConnected;
  
  // 1. WiFi Master Switch
  if (isWifiEnabled != prev_isWifiEnabled || isWifiConnected != prev_isWifiConnected || screenNeedsRedraw) {
    if (isWifiEnabled) {
       tft.fillRoundRect(60, 50, 200, 50, 10, COLOR_GREEN);
       tft.setTextColor(ILI9341_BLACK); 
       tft.setTextSize(2);
       
       String statusMsg = isWifiConnected ? "WiFi: ON (Conn)" : "WiFi: ON (Wait)";
       int16_t x1, y1; uint16_t w, h;
       tft.getTextBounds(statusMsg, 0, 0, &x1, &y1, &w, &h);
       tft.setCursor(60 + (200 - w)/2, 65);
       tft.print(statusMsg);
    } else {
       tft.fillRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON);
       tft.setTextColor(COLOR_BUTTON_TEXT);
       tft.setTextSize(2);
       
       String statusMsg = "WiFi Power: OFF";
       int16_t x1, y1; uint16_t w, h;
       tft.getTextBounds(statusMsg, 0, 0, &x1, &y1, &w, &h);
       tft.setCursor(60 + (200 - w)/2, 65);
       tft.print(statusMsg);
    }
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
    
    prev_isWifiEnabled = isWifiEnabled;
    prev_isWifiConnected = isWifiConnected;
  }

  // 2. Data Selection Checkboxes
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
  static float prev_PF = -1.0;
  static float prev_thd_v_main = -1.0; 
  static float prev_thd_i_main = -1.0;

  // V
  printTFTValue(col1_value_x, 40, V_rms, prev_V_rms, 1, COLOR_BLUE, " V");
  prev_V_rms = V_rms;
  // I
  tft.fillRect(col1_value_x, 65, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, 65);
  if (I_rms < 1.0) { 
    dtostrf(I_rms * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }
  // P
  tft.fillRect(col1_value_x, 90, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_DARKGREEN);
  tft.setCursor(col1_value_x, 90);
  if (P_real >= 1000.0) { 
    dtostrf(P_real / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kW");
  } else { 
    dtostrf(P_real, 4, 1, buffer);
    tft.print(buffer); tft.print(" W");
  }
  // PF
  printTFTValue(col2_value_x, 115, PF, prev_PF, 2, COLOR_MAGENTA, "");
  prev_PF = PF;
  // Q
  tft.fillRect(col1_value_x, 115, col_w_half, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_ORANGE);
  tft.setCursor(col1_value_x, 115);
  if (Q_reactive >= 1000.0) { 
    dtostrf(Q_reactive / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kvar");
  } else { 
    dtostrf(Q_reactive, 4, 1, buffer);
    tft.print(buffer); tft.print(" var");
  }
  // I-1
  tft.fillRect(col2_value_x, 40, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_RED);
  tft.setCursor(col2_value_x, 40); 
  if (I_rms_load1 < 1.0) { 
    dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load1, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }
  // I-2
  tft.fillRect(col2_value_x, 65, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_GREEN);
  tft.setCursor(col2_value_x, 65); 
  if (I_rms_load2 < 1.0) { 
    dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer);
    tft.print(buffer); tft.print(" mA");
  } else { 
    dtostrf(I_rms_load2, 4, 2, buffer);
    tft.print(buffer); tft.print(" A");
  }
  // S
  tft.fillRect(col2_value_x, 90, col_w_half_wide, 20, COLOR_BACKGROUND); 
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setCursor(col2_value_x, 90); 
  if (S_apparent >= 1000.0) { 
    dtostrf(S_apparent / 1000.0, 4, 2, buffer);
    tft.print(buffer); tft.print(" kVA");
  } else { 
    dtostrf(S_apparent, 4, 1, buffer);
    tft.print(buffer); tft.print(" VA");
  }
  // THD-V
  printTFTValue(col2_value_x, 140, thd_v_value * 100.0, prev_thd_v_main * 100.0, 1, COLOR_BLUE, " %");
  prev_thd_v_main = thd_v_value;
  // THD-I
  printTFTValue(col2_value_x, 165, thd_i_value * 100.0, prev_thd_i_main * 100.0, 1, COLOR_ORANGE, " %");
  prev_thd_i_main = thd_i_value;
}

// --- 위상차 화면 값 업데이트 ---
void displayPhaseScreenValues() {
  if (isPhaseFrozen) return; 

  phase_degrees = acos(abs(PF)) * (180.0 / M_PI);
  if (I_rms < 0.05) {
     phase_degrees = 0.0;
  }

  tft.setTextSize(2);
  printTFTValue(60, 50, PF, prev_phase_degrees, 2, COLOR_MAGENTA, "");
  printTFTValue(10, 100, lead_lag_status, prev_lead_lag_status, COLOR_TEXT_PRIMARY); 
  printTFTValue(60, 140, phase_main_deg, prev_phase_main_deg, 1, COLOR_ORANGE, "d"); 
  printTFTValue(60, 165, phase_load1_deg, prev_phase_load1_deg, 1, COLOR_RED, "d"); 
  printTFTValue(60, 190, phase_load2_deg, prev_phase_load2_deg, 1, COLOR_GREEN, "d"); 

  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, COLOR_BACKGROUND); 
  tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, COLOR_BACKGROUND); 

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

  tft.drawLine(PHASOR_CX, PHASOR_CY, v_x, v_y, COLOR_BLUE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, im_x, im_y, COLOR_ORANGE);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i1_x, i1_y, COLOR_RED);
  tft.drawLine(PHASOR_CX, PHASOR_CY, i2_x, i2_y, COLOR_GREEN);
  
  prev_phase_degrees = PF; 
  prev_lead_lag_status = lead_lag_status; 
  prev_phase_main_deg = phase_main_deg;
  prev_phase_load1_deg = phase_load1_deg;
  prev_phase_load2_deg = phase_load2_deg;
  
  prev_v_x = v_x; prev_v_y = v_y;
  prev_im_x = im_x; prev_im_y = im_y;
  prev_i1_x = i1_x; prev_i1_y = i1_y;
  prev_i2_x = i2_x; prev_i2_y = i2_y;
}

// --- Y축 범위 계산 헬퍼 ---
float findAxisStep(float peak, const float* steps, int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    if (peak <= steps[i]) return steps[i]; 
  }
  return steps[num_steps - 1]; 
}

// --- 동적 Y축 라벨 업데이트 ---
void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_CENTER - 15, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_CENTER + 5, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 

  if (waveformPlotType == 0) { // V/I
    tft.setTextColor(COLOR_BLUE);
    dtostrf(plot1_axis_max, 3, 0, buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("V"); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("V");
    
    tft.setTextColor(COLOR_ORANGE);
    if (plot2_axis_max < 1.0) { 
      dtostrf(plot2_axis_max * 1000, 3, 0, buffer);
      tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("mA");
      tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("mA");
    } else { 
      dtostrf(plot2_axis_max, 3, 1, buffer);
      tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
      tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
    }
  } else if (waveformPlotType == 1) { // P/V
    tft.setTextColor(COLOR_BLUE);
    dtostrf(plot1_axis_max, 3, 0, buffer); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("W"); 
    tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("W");
    
    tft.setTextColor(COLOR_ORANGE);
    dtostrf(plot2_axis_max, 3, 0, buffer); 
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("V"); 
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("V");
  } else { // I/I1/I2
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

// --- 파형 그리기 루프 (핵심) ---
void runCombinedWaveformLoop() {
  if (isWaveformFrozen) return;

  float scale_p1, scale_p2, scale_p3; 
  float new_frame_p1_peak = 0.0, new_frame_p2_peak = 0.0, new_frame_p3_peak = 0.0;
  int new_y_plot1[PLOT_WIDTH], new_y_plot2[PLOT_WIDTH], new_y_plot3[PLOT_WIDTH];

  if (waveformPlotType == 0) { // V/I
    scale_p1 = (plot1_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = (plot2_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  } else if (waveformPlotType == 1) { // P/V
    scale_p1 = (plot1_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = (plot2_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  } else { // I/I1/I2
    scale_p1 = (plot1_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
    scale_p2 = scale_p1;
    scale_p3 = scale_p1;
  }

  for (int i = 0; i < PLOT_WIDTH; i++) {
    String line = "";
    while(line.length() == 0) {
      if (Serial.available()) {
         line = Serial.readStringUntil('\n');
         line.trim();
      }
      checkTouchInput();
      if (screenNeedsRedraw || warningActive) {
        if(screenNeedsRedraw) isWaveformFrozen = false; 
        return; 
      }
    }
    
    float val1_inst = 0.0, val2_inst = 0.0, val3_inst = 0.0;
    int y_pos_p1, y_pos_p2, y_pos_p3;

    int comma1 = line.indexOf(',');
    if (comma1 != -1) {
      val1_inst = line.substring(0, comma1).toFloat();
      int comma2 = line.indexOf(',', comma1 + 1);
      
      if (comma2 == -1) { // 2 Values
        val2_inst = line.substring(comma1 + 1).toFloat();
        if (abs(val1_inst) > new_frame_p1_peak) new_frame_p1_peak = abs(val1_inst);
        if (abs(val2_inst) > new_frame_p2_peak) new_frame_p2_peak = abs(val2_inst);
        y_pos_p1 = PLOT_Y_CENTER - (int)(val1_inst * scale_p1);
        y_pos_p2 = PLOT_Y_CENTER - (int)(val2_inst * scale_p2);
        y_pos_p3 = PLOT_Y_CENTER;
      } else { // 3 Values
        val2_inst = line.substring(comma1 + 1, comma2).toFloat();
        val3_inst = line.substring(comma2 + 1).toFloat();
        if (abs(val1_inst) > new_frame_p1_peak) new_frame_p1_peak = abs(val1_inst);
        if (abs(val2_inst) > new_frame_p2_peak) new_frame_p2_peak = abs(val2_inst);
        if (abs(val3_inst) > new_frame_p3_peak) new_frame_p3_peak = abs(val3_inst);
        y_pos_p1 = PLOT_Y_CENTER - (int)(val1_inst * scale_p1);
        y_pos_p2 = PLOT_Y_CENTER - (int)(val2_inst * scale_p2);
        y_pos_p3 = PLOT_Y_CENTER - (int)(val3_inst * scale_p3);
      }
      new_y_plot1[i] = constrain(y_pos_p1, PLOT_Y_START, PLOT_Y_END);
      new_y_plot2[i] = constrain(y_pos_p2, PLOT_Y_START, PLOT_Y_END);
      new_y_plot3[i] = constrain(y_pos_p3, PLOT_Y_START, PLOT_Y_END);
    } else {
      new_y_plot1[i] = PLOT_Y_CENTER;
      new_y_plot2[i] = PLOT_Y_CENTER;
      new_y_plot3[i] = PLOT_Y_CENTER;
    }
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
  float new_p3_axis = plot3_axis_max;
  bool axisChanged = false;

  if (waveformPlotType == 0) { 
    new_p1_axis = findAxisStep(new_frame_p1_peak, V_AXIS_STEPS, NUM_V_STEPS);
    new_p2_axis = findAxisStep(new_frame_p2_peak, I_AXIS_STEPS, NUM_I_STEPS);
  } else if (waveformPlotType == 1) { 
    new_p1_axis = findAxisStep(new_frame_p1_peak, P_AXIS_STEPS, NUM_P_STEPS);
    new_p2_axis = findAxisStep(new_frame_p2_peak, V_AXIS_STEPS, NUM_V_STEPS);
  } else { // I/I1/I2
    float max_peak_all = max(new_frame_p1_peak, max(new_frame_p2_peak, new_frame_p3_peak));
    new_p1_axis = findAxisStep(max_peak_all, I_AXIS_STEPS, NUM_I_STEPS);
    if (new_p1_axis != plot1_axis_max) {
       axisChanged = true;
    }
  }

  if (new_p1_axis != plot1_axis_max || new_p2_axis != plot2_axis_max || axisChanged) {
    plot1_axis_max = new_p1_axis;
    if (waveformPlotType != 2) {
        plot2_axis_max = new_p2_axis;
    }
    updateYAxisLabels();
  }

  if (waveformTriggerMode == 2 && !isWaveformFrozen) {
    isWaveformFrozen = true;
  }
}

// --- [신규] HARMONICS 화면 값 업데이트 ---
void displayHarmonicsScreenValues() {
  if (isHarmonicsFrozen) return;

  // 1. 데이터 소스 포인터 설정 (V 또는 I)
  float* dataPtr = (harmonicsSource == 0) ? v_harmonics : i_harmonics;
  
  // 2. 뷰 모드에 따른 그리기
  if (harmonicsViewMode == 0) {
     // --- Graph View (Bar Chart) ---
     int graph_x = 40;
     int graph_y = 45;
     int graph_w = 270;
     int graph_h = 145;
     int bar_w = graph_w / 15; // 15개 막대

     for (int i = 1; i <= 15; i++) {
        float val = dataPtr[i]; // 퍼센트 값
        
        // 로그 스케일 높이 계산 (0.1% ~ 100% 기준)
        int bar_h = 0;
        if (val >= 0.1) {
           float log_val = log10(val);
           float normalized = (log_val + 1.0) / 3.0; 
           if (normalized > 1.0) normalized = 1.0;
           if (normalized < 0.0) normalized = 0.0;
           bar_h = (int)(normalized * graph_h);
        } else {
           bar_h = 1; // 최소 높이
        }

        int x_pos = graph_x + (i - 1) * bar_w + 2;
        int y_pos = graph_y + graph_h - bar_h;
        
        // 색상: 기본파(1차)는 파랑, 나머지는 소스에 따라 주황/초록
        uint16_t color = (i == 1) ? COLOR_BLUE : ((harmonicsSource == 0) ? COLOR_ORANGE : COLOR_GREEN);
        
        // 막대 갱신 (잔상 제거를 위해 해당 막대 영역 전체 배경색 칠한 후 다시 그림)
        tft.fillRect(x_pos, graph_y, bar_w - 4, graph_h, COLOR_BACKGROUND);
        tft.fillRect(x_pos, y_pos, bar_w - 4, bar_h, color);
     }

  } else {
     // --- List View (Text Table) ---
     tft.setTextSize(2);
     tft.setTextColor(COLOR_TEXT_PRIMARY);
     
     int col1_x = 10;  // 왼쪽 열 X
     int start_y = 80;
     int line_h = 20;
     
     // 1차 ~ 7차까지만 표시 (공간 제약)
     for (int i = 1; i <= 7; i++) {
        int y = start_y + (i - 1) * line_h;
        
        // 값 포맷팅
        char buff_pct[10]; dtostrf(dataPtr[i], 5, 1, buff_pct); // " 99.9"
        
        // 이전 영역 지우기 (배경색)
        tft.fillRect(col1_x, y, 150, line_h, COLOR_BACKGROUND);
        
        tft.setCursor(col1_x, y);
        tft.print(i); 
        tft.print(":");
        tft.print(buff_pct);
        tft.print("%");
     }
  }
}

// --- 캘리브레이션 값 업데이트 ---
void displaySettingsCalibValues() {
  int y_positions[] = {50, 85, 120};
  
  if (prev_calib_selection != calib_selection) {
    if (prev_calib_selection != -1) {
      tft.fillRect(10, y_positions[prev_calib_selection], 15, 18, COLOR_BACKGROUND); 
    }
    tft.setTextColor(COLOR_RED); 
    tft.setTextSize(2);
    tft.setCursor(10, y_positions[calib_selection]);
    tft.print(">");
    prev_calib_selection = calib_selection;
  }

  printTFTValue(190, 50, V_MULTIPLIER, prev_V_MULTIPLIER, 2, COLOR_BLUE, " x"); 
  printTFTValue(190, 85, I_MULTIPLIER, prev_I_MULTIPLIER, 2, COLOR_ORANGE, " x"); 
  printTFTValue(190, 120, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_V_MULTIPLIER = V_MULTIPLIER;
  prev_I_MULTIPLIER = I_MULTIPLIER;
  prev_setting_step_index = setting_step_index;
}

// --- 보호 설정 값 업데이트 ---
void displaySettingsProtectValues() {
  int y_positions[] = {70, 110};
  
  if (prev_protect_selection != protect_selection) {
    if (prev_protect_selection != -1) {
      tft.fillRect(10, y_positions[prev_protect_selection], 15, 18, COLOR_BACKGROUND); 
    }
    tft.setTextColor(COLOR_RED); 
    tft.setTextSize(2);
    tft.setCursor(10, y_positions[protect_selection]);
    tft.print(">");
    prev_protect_selection = protect_selection;
  }
  
  printTFTValue(190, 70, VOLTAGE_THRESHOLD, prev_VOLTAGE_THRESHOLD, 1, COLOR_RED, " V"); 
  printTFTValue(190, 110, setting_steps[setting_step_index], (prev_setting_step_index == -1) ? -1.0 : setting_steps[prev_setting_step_index], 4, COLOR_TEXT_PRIMARY, ""); 

  prev_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD;
  prev_setting_step_index = setting_step_index;
}

// --- 릴레이 상태 업데이트 ---
void runRelayControl() {
  if (relay1_state != prev_r1_state || relay2_state != prev_r2_state || screenNeedsRedraw) {
    String r1_status = relay1_state ? "ON" : "OFF";
    String r2_status = relay2_state ? "ON" : "OFF";
    
    drawButton(20, 70, 280, 40, "Relay 1: " + r1_status);
    drawButton(20, 130, 280, 40, "Relay 2: " + r2_status);
    
    prev_r1_state = relay1_state;
    prev_r2_state = relay2_state;
  }
}

// --- 테마 설정 (동적 부분) ---
void runSettingsTheme() {
  // 현재는 정적 화면만 사용하지만 확장성을 위해 유지
}

// --- 타이머 값 동적 표시 ---
void displaySettingsTimerValues() {
  
  if (temp_timer_setting_seconds != prev_temp_timer_setting_seconds) {
    uint32_t total_sec = temp_timer_setting_seconds;
    uint16_t hours = total_sec / 3600;
    uint16_t minutes = (total_sec % 3600) / 60;
    uint8_t seconds = total_sec % 60;
    
    char buffer[15];
    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
    
    tft.setTextSize(4); 
    int x_pos = 64;
    int y_pos = 70; 

    if (prev_temp_timer_setting_seconds != 0xFFFFFFFF) {
        uint32_t prev_total = prev_temp_timer_setting_seconds;
        uint16_t ph = prev_total / 3600;
        uint16_t pm = (prev_total % 3600) / 60;
        uint8_t ps = prev_total % 60;
        char prev_buffer[15];
        sprintf(prev_buffer, "%02d:%02d:%02d", ph, pm, ps);
        
        if (strcmp(buffer, prev_buffer) != 0) {
           tft.setTextColor(COLOR_BACKGROUND);
           tft.setCursor(x_pos, y_pos);
           tft.print(prev_buffer);
        }
    }
    
    tft.setTextColor(COLOR_ORANGE);
    tft.setCursor(x_pos, y_pos);
    tft.print(buffer);
    
    prev_temp_timer_setting_seconds = temp_timer_setting_seconds;
  }

  if (is_timer_active != prev_is_timer_active) {
    if (is_timer_active) {
       tft.fillRoundRect(230, 115, 80, 50, 8, COLOR_RED);
       tft.drawRoundRect(230, 115, 80, 50, 8, COLOR_BUTTON_OUTLINE);
       tft.setTextColor(ILI9341_WHITE);
       tft.setTextSize(2);
       tft.setCursor(245, 130); 
       tft.print("STOP");
    } else {
       tft.fillRoundRect(230, 115, 80, 50, 8, COLOR_BUTTON); 
       tft.drawRoundRect(230, 115, 80, 50, 8, COLOR_BUTTON_OUTLINE);
       tft.setTextColor(COLOR_BUTTON_TEXT);
       tft.setTextSize(2);
       tft.setCursor(240, 130); 
       tft.print("START");
    }
  }

  if (timer_step_index != prev_timer_step_index) {
    String current_label = String(TIMER_STEP_LABELS[timer_step_index]);
    String prev_label = (prev_timer_step_index == -1) ? "" : String(TIMER_STEP_LABELS[prev_timer_step_index]);
    printTFTValue(90, 185, current_label, prev_label, COLOR_TEXT_PRIMARY);
    prev_timer_step_index = timer_step_index;
  }

  String status_text = is_timer_active ? "ACTIVE" : "IDLE";
  String prev_status_text = prev_is_timer_active ? "ACTIVE" : "IDLE";
  uint16_t status_color = is_timer_active ? COLOR_GREEN : COLOR_RED;
  
  printTFTValue(100, 215, status_text, prev_status_text, status_color);
  prev_is_timer_active = is_timer_active; 
  
  if (timer_seconds_left != prev_timer_seconds_left) {
    uint16_t hours = timer_seconds_left / 3600;
    uint16_t minutes = (timer_seconds_left % 3600) / 60;
    uint8_t seconds = timer_seconds_left % 60;
    char buffer[15];
    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
    
    char prev_buffer[15];
    uint16_t ph = prev_timer_seconds_left / 3600;
    uint16_t pm = (prev_timer_seconds_left % 3600) / 60;
    uint8_t ps = prev_timer_seconds_left % 60;
    sprintf(prev_buffer, "%02d:%02d:%02d", ph, pm, ps);

    printTFTValue(220, 215, String(buffer), String(prev_buffer), COLOR_TEXT_PRIMARY);
    prev_timer_seconds_left = timer_seconds_left;
  }
}