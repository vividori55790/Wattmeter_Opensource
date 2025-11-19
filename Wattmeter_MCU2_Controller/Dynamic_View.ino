/*
 * ==============================================================================
 * 파일명: 3. View_Dynamic.ino
 * 버전: v212 (Waveform & Harmonics Update)
 * 설명: 
 * - [Mod] runCombinedWaveformLoop: Wattmeter.ino의 Trigger/Drawing 로직 완전 이식
 * - [Mod] 고조파 화면: 수치 표시(좌측) 및 그래프(하단) 구현
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

// [Mod] Wattmeter.ino 원본 스타일 복원 (Scale Finding)
float findAxisStep(float peak, const float* steps, int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    if (peak <= steps[i]) {
      return steps[i]; 
    }
  }
  return steps[num_steps - 1]; 
}

void updateYAxisLabels() {
  tft.setTextSize(1);
  char buffer[10]; 
  
  // Clear Labels Area
  tft.fillRect(0, PLOT_Y_START, PLOT_X_START - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(0, PLOT_Y_END - 10, PLOT_X_START - 1, 10, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_START, SCREEN_WIDTH - PLOT_X_END - 1, 20, COLOR_BACKGROUND); 
  tft.fillRect(PLOT_X_END + 1, PLOT_Y_END - 10, SCREEN_WIDTH - PLOT_X_END - 1, 10, COLOR_BACKGROUND); 
  
  tft.setTextColor(COLOR_ORANGE);
  if (I_axis_max < 1.0) { 
    dtostrf(I_axis_max * 1000, 3, 0, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("mA");
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("mA");
  } else { 
    dtostrf(I_axis_max, 3, 1, buffer);
    tft.setCursor(0, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("A");
    tft.setCursor(0, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("A");
  }

  tft.setTextColor(COLOR_BLUE);
  dtostrf(V_axis_max, 3, 0, buffer); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_START + 5); tft.print("+"); tft.print(buffer); tft.print("V"); 
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_END - 10); tft.print("-"); tft.print(buffer); tft.print("V");
}

// [Mod] Wattmeter.ino 원본 스타일 복원 (Drawing Loop)
void runCombinedWaveformLoop() {
  float volts_to_pixels_scale = (V_axis_max < 1.0) ? 0 : (PLOT_HEIGHT_HALF / V_axis_max);
  float amps_to_pixels_scale = (I_axis_max < 0.01) ? 0 : (PLOT_HEIGHT_HALF / I_axis_max);

  float new_frame_V_peak = 0.0;
  float new_frame_I_peak = 0.0;
  
  int new_y_v[PLOT_WIDTH];
  int new_y_i[PLOT_WIDTH];

  unsigned long startTime = micros();

  float effective_V_Calib = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float effective_I_Calib = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float effective_V_Offset = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float effective_I_Offset = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;

  for (int i = 0; i < PLOT_WIDTH; i++) {
    // 루프 중에도 터치/시리얼 체크 (응답성 유지)
    checkTouchInput();
    checkSerialInput();
    if(screenNeedsRedraw || warningActive) return;

    int V_raw = analogRead(PIN_ADC_V);
    int I_raw = analogRead(PIN_ADC_I);
    int V_ac_bits = V_raw - (int)ADC_MIDPOINT;
    int I_ac_bits = I_raw - (int)ADC_MIDPOINT;
    
    float V_mains_instant = V_ac_bits * effective_V_Calib + effective_V_Offset;
    float I_mains_instant = I_ac_bits * effective_I_Calib - effective_I_Offset;

    if (abs(V_mains_instant) > new_frame_V_peak) new_frame_V_peak = abs(V_mains_instant);
    if (abs(I_mains_instant) > new_frame_I_peak) new_frame_I_peak = abs(I_mains_instant);

    int y_pos_v = PLOT_Y_CENTER - (int)(V_mains_instant * volts_to_pixels_scale);
    new_y_v[i] = constrain(y_pos_v, PLOT_Y_START, PLOT_Y_END);
    int y_pos_i = PLOT_Y_CENTER - (int)(I_mains_instant * amps_to_pixels_scale);
    new_y_i[i] = constrain(y_pos_i, PLOT_Y_START, PLOT_Y_END);
    
    while(micros() - startTime < (i + 1) * WAVEFORM_SAMPLE_PERIOD_US);
  }

  tft.startWrite(); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  for (int i = 1; i < PLOT_WIDTH; i++) {
      int x_curr = PLOT_X_START + i;
      int x_prev = PLOT_X_START + i - 1;
      if(last_frame_y_v[i] != PLOT_Y_CENTER || last_frame_y_v[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_v[i-1], x_curr, last_frame_y_v[i], COLOR_BACKGROUND); 
      }
      if(last_frame_y_i[i] != PLOT_Y_CENTER || last_frame_y_i[i-1] != PLOT_Y_CENTER) {
          tft.drawLine(x_prev, last_frame_y_i[i-1], x_curr, last_frame_y_i[i], COLOR_BACKGROUND); 
      }
      tft.drawLine(x_prev, new_y_v[i-1], x_curr, new_y_v[i], COLOR_BLUE);
      tft.drawLine(x_prev, new_y_i[i-1], x_curr, new_y_i[i], COLOR_ORANGE);
  }
  tft.endWrite();

  for (int i = 0; i < PLOT_WIDTH; i++) {
      last_frame_y_v[i] = new_y_v[i];
      last_frame_y_i[i] = new_y_i[i];
  }

  // Auto Scaling
  float new_V_axis = findAxisStep(new_frame_V_peak, V_AXIS_STEPS, NUM_V_STEPS);
  float new_I_axis = findAxisStep(new_frame_I_peak, I_AXIS_STEPS, NUM_I_STEPS);

  if (new_V_axis != V_axis_max || new_I_axis != I_axis_max) {
    V_axis_max = new_V_axis;
    I_axis_max = new_I_axis;
    updateYAxisLabels();
  }
}

void displayHarmonicsScreenValues() {
  if (harmonicsSrcLabel != prev_harmonicsSrcLabel) {
     drawButton(10, 200, 90, 35, harmonicsSrcLabel);
     prev_harmonicsSrcLabel = harmonicsSrcLabel;
  }
  
  String currRunLabel = isHarmonicsFrozen ? "RUN" : "HOLD";
  if (currRunLabel != prev_harmonicsRunLabel) {
     drawButton(110, 200, 100, 35, currRunLabel);
     prev_harmonicsRunLabel = currRunLabel;
  }

  if (isHarmonicsFrozen) return;

  // [Mod] 데이터 포인터 설정 (MCU1에서 수신한 실제 데이터 사용)
  float* dataPtr = (harmonicsSource == 0) ? v_harmonics : i_harmonics;
  // 기본파 값(100%) 참조 (MCU1에서 수신한 RMS가 기본임)
  float rms_val = (harmonicsSource == 0) ? V_rms : I_rms;
  float thd_val = (harmonicsSource == 0) ? thd_v_value : thd_i_value;

  // [Mod] 좌측 수치 표시 (THD, RMS)
  static float prev_disp_thd = -1.0;
  static float prev_disp_rms = -1.0;
  
  printTFTValue(10, 75, thd_val * 100.0, prev_disp_thd * 100.0, 1, COLOR_RED, " %");
  printTFTValue(10, 125, rms_val, prev_disp_rms, 1, COLOR_TEXT_PRIMARY, (harmonicsSource == 0) ? " V" : " A");
  
  prev_disp_thd = thd_val;
  prev_disp_rms = rms_val;

  // [Mod] 그래프 표시
  int graph_x = 80;
  int graph_y = 45;
  int graph_h = 145;
  int bar_w = 230 / 15; 
  static int prev_bar_heights[16]; 
  
  // i=1(Fundamental) 부터 15차까지
  for (int i = 1; i <= 15; i++) {
      float val_pct = dataPtr[i]; // MCU1 sends percentage
      int bar_h = 0;
      
      // Log Scale or Linear? MCU1 sends percent. Linear is easier to read for THD.
      // 100% = Full Height.
      if (val_pct > 100.0) val_pct = 100.0;
      if (val_pct < 0.0) val_pct = 0.0;
      
      // 1차 고조파는 100%이므로 꽉 참. 나머지는 작게 나옴.
      // 로그 스케일 적용하여 작은 고조파도 보이게 함 (Log10)
      // 0.1% ~ 100% 매핑
      if (val_pct > 0.1) {
         float log_val = log10(val_pct); // log10(100)=2, log10(0.1)=-1
         // Map -1~2 to 0~Height
         float normalized = (log_val + 1.0) / 3.0; 
         if(normalized < 0) normalized = 0;
         bar_h = (int)(normalized * graph_h);
      } else {
         bar_h = 1;
      }
      
      if (bar_h != prev_bar_heights[i]) {
          int x_pos = graph_x + (i - 1) * bar_w + 2;
          int curr_y = graph_y + graph_h - bar_h;
          uint16_t color = (harmonicsSource == 0) ? COLOR_BLUE : COLOR_ORANGE;
          
          tft.fillRect(x_pos, graph_y, bar_w - 4, graph_h, COLOR_BACKGROUND); // Clear
          tft.fillRect(x_pos, curr_y, bar_w - 4, bar_h, color); // Draw
          
          prev_bar_heights[i] = bar_h;
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

  static bool prev_btn_active = false;
  if (is_timer_active != prev_btn_active || screenNeedsRedraw) {
     String actionLabel = is_timer_active ? "STOP" : "START";
     uint16_t color = is_timer_active ? COLOR_RED : COLOR_GREEN;
     
     tft.fillRoundRect(230, 150, 80, 40, 8, color);
     tft.drawRoundRect(230, 150, 80, 40, 8, COLOR_BUTTON_OUTLINE);
     tft.setTextColor(ILI9341_WHITE);
     tft.setTextSize(2);
     tft.setCursor(240, 162); 
     tft.print(actionLabel);
     
     prev_btn_active = is_timer_active;
  }
}