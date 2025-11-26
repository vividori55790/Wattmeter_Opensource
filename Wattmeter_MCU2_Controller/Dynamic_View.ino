/*
 * ==============================================================================
 * 파일명: 3. Dynamic_View.ino
 * 버전: v219 (Fix Graph Scale Mismatch)
 * 설명: 
 * - [Fix] 릴레이 상태 표시: 변수 상태를 그대로 반영하도록 로직 단순화
 * - [Fix] 페이서 작도: 선 이동 시 지워지는 배경 그리드 복구 로직 추가
 * - [Fix] 화면 갱신: 화면 진입 시(screenNeedsRedraw) 값이 같아도 강제 출력
 * - [Mod] 고조파 소스 변경 시 텍스트 리스트 강제 갱신 로직 추가
 * - [Mod] 페이서 다이어그램: 전체 삭제 -> 그리드 복구 -> 길이순(긴->짧은) 정렬 그리기 적용
 * - [Fix] Auto Calibration 재진입 시 값 갱신 안 되는 오류 수정 (resetViewStates 초기화 추가)
 * - [Fix] P/Q Waveform Stability (Continuous Mode) - 전역 버퍼 사용 및 초기화 로직
 * - [Fix] Waveform Screen Entry Scale Mismatch: Auto-ranging indices globalized & reset on entry
 * ==============================================================================
 */

// 외부 전역 변수 참조 (Controller에서 정의됨)
extern bool prev_is_timer_active; 
extern float prev_temp_true_v; // Controller에서 정의됨
extern float prev_temp_true_i; // Controller에서 정의됨
extern float V_ADC_MIDPOINT_CALIB; // [Mod] 추가된 외부 변수 참조
extern float I_ADC_MIDPOINT_CALIB; // [Mod] 추가된 외부 변수 참조
extern float I1_ADC_MIDPOINT_CALIB; // [Mod] 추가된 외부 변수 참조
extern float I2_ADC_MIDPOINT_CALIB; // [Mod] 추가된 외부 변수 참조

// [FIX] P/Q Waveform Stability Fix Externs
extern float P_Q_LAG_BUFFER[];
extern int P_Q_LAG_HEAD;
extern bool IS_LAG_BUFFER_INIT;

// [New] Globalized Auto-Ranging Indices (moved from runCombinedWaveformLoop)
// 초기값은 안전하게 0으로 설정하며, 실제 초기화는 resetViewStates()에서 수행됩니다.
int range_idx_v = 0; 
int range_idx_i = 0; 
int range_idx_p = 0; 

// [New] 전역으로 승격된 이전 값 저장 변수들 (화면 전환 시 초기화를 위함)
// 메인 전력 화면
float prev_disp_V_rms = -1.0;
float prev_disp_I_rms = -1.0;
float prev_disp_P_real = -1.0;
float prev_disp_PF = -1.0;
float prev_disp_Q_reactive = -1.0;
float prev_disp_I_rms_load1 = -1.0;
float prev_disp_I_rms_load2 = -1.0;
float prev_disp_S_apparent = -1.0;
float prev_disp_thd_v_main = -1.0;
float prev_disp_thd_i_main = -1.0;

// 타이머 화면
uint32_t prev_timer_display_time = 0xFFFFFFFF;
bool prev_timer_active_state = false;

// Auto Calibration 화면 (Step 5 검증용) - 전역으로 이동
float prev_disp_v = -1.0;
float prev_disp_i = -1.0;

// 고조파 그래프 이전 높이
static int prev_bar_heights[16]; 
static float prev_text_vals[16];  

// --- 헬퍼: 값 출력 (Float) ---
// [Mod] screenNeedsRedraw가 true면 값이 같아도 강제로 출력
void printTFTValue(int x, int y, float value, float prev_value, int precision, uint16_t color, String unit) {
  if (!screenNeedsRedraw && abs(value - prev_value) < (pow(10, -precision) / 2.0)) return; 
  
  char buffer[20];
  // --- [Mod] 이전 값 영역 지우기 시작 (Dynamic Clear) ---
  int16_t x1, y1; uint16_t w, h;
  dtostrf(prev_value, 4, precision, buffer);
  String prev_str = String(buffer) + unit;
  tft.getTextBounds(prev_str, x, y, &x1, &y1, &w, &h);
  tft.fillRect(x, y, w, h, COLOR_BACKGROUND);
  // --- [Mod] 이전 값 영역 지우기 종료 ---
  
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  dtostrf(value, 4, precision, buffer);
  tft.print(buffer); tft.print(unit); 
}

// --- 헬퍼: 값 출력 (Int) ---
// [Mod] screenNeedsRedraw가 true면 값이 같아도 강제로 출력
void printTFTValue(int x, int y, int value, int prev_value, uint16_t color, String unit) {
  if (!screenNeedsRedraw && value == prev_value) return; 
  
  char buffer[20];
  // --- [Mod] 이전 값 영역 지우기 시작 (Dynamic Clear) ---
  int16_t x1, y1; uint16_t w, h;
  sprintf(buffer, "%d%s", prev_value, unit.c_str());
  String prev_str = String(buffer);
  tft.getTextBounds(prev_str, x, y, &x1, &y1, &w, &h);
  tft.fillRect(x, y, w, h, COLOR_BACKGROUND);
  // --- [Mod] 이전 값 영역 지우기 종료 ---
  
  tft.setTextColor(color); 
  tft.setCursor(x, y);
  sprintf(buffer, "%d%s", value, unit.c_str());
  tft.print(buffer); 
}

// --- 헬퍼: 값 출력 (String) ---
// [Mod] screenNeedsRedraw가 true면 값이 같아도 강제로 출력
void printTFTValue(int x, int y, String value, String prev_value, uint16_t color) {
  if (!screenNeedsRedraw && value.equals(prev_value)) return; 
  
  // --- [Mod] 이전 값 영역 지우기 시작 (Dynamic Clear) ---
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(prev_value, x, y, &x1, &y1, &w, &h);
  tft.fillRect(x, y, w, h, COLOR_BACKGROUND);
  // --- [Mod] 이전 값 영역 지우기 종료 ---
  
  tft.setTextColor(color); 
  tft.setCursor(x, y); 
  tft.print(value);
}

// [New] 화면 상태 초기화 함수 (화면 진입 시 호출)
void resetViewStates() {
    // 메인 화면 변수 초기화
    prev_disp_V_rms = -1.0;
    prev_disp_I_rms = -1.0;
    prev_disp_P_real = -1.0;
    prev_disp_PF = -1.0;
    prev_disp_Q_reactive = -1.0;
    prev_disp_I_rms_load1 = -1.0;
    prev_disp_I_rms_load2 = -1.0;
    prev_disp_S_apparent = -1.0;
    prev_disp_thd_v_main = -1.0;
    prev_disp_thd_i_main = -1.0;

    // 페이서 화면 텍스트 변수 초기화
    prev_phase_degrees = -999.0;
    prev_lead_lag_status = "";
    prev_phase_main_deg = -999.0;
    prev_phase_load1_deg = -999.0;
    prev_phase_load2_deg = -999.0;
    
    // [New Fix] P/Q 버퍼 초기화 플래그 리셋
    IS_LAG_BUFFER_INIT = false;

    // [Fix] 페이서 작도(선) 좌표 초기화 (강제 갱신 유도)
    prev_v_x = -1; prev_v_y = -1;
    prev_im_x = -1; prev_im_y = -1;
    prev_i1_x = -1; prev_i1_y = -1;
    prev_i2_x = -1; prev_i2_y = -1;
    
    // 고조파 관련 초기화
    for(int i=0; i<16; i++) {
        prev_bar_heights[i] = -1;
        prev_text_vals[i] = -1.0;
    }
    prev_harmonicsSrcLabel = "";
    prev_harmonicsRunLabel = "";

    // 타이머 화면 초기화
    prev_timer_display_time = 0xFFFFFFFF;
    prev_timer_active_state = false;
    prev_timer_step_index = -1;
    prev_timer_target_relay = -1;

    // 설정 화면 초기화
    prev_calib_selection = -1;
    prev_V_MULTIPLIER = -1.0;
    prev_I_MULTIPLIER = -1.0;
    prev_setting_step_index = -1;
    prev_protect_selection = -1;
    prev_VOLTAGE_THRESHOLD = -1.0;
    
    // [Fix] Auto Calibration 화면 초기화
    prev_temp_true_v = -1.0;
    prev_temp_true_i = -1.0;
    prev_disp_v = -1.0;
    prev_disp_i = -1.0;

    // 릴레이 컨트롤 초기화
    prev_r1_state = !relay1_state; // 강제 불일치 유도
    prev_r2_state = !relay2_state;

    // [Fix] Waveform Scale Reset (Start at Max Range to match Labels)
    range_idx_v = NUM_V_RANGES - 1;
    range_idx_i = NUM_I_RANGES - 1;
    range_idx_p = NUM_P_RANGES - 1;
}

// --- [Mod] 네트워크 설정 화면 동적 갱신 ---
void runSettingsNetwork() {
  if (screenNeedsRedraw) {
    // 테두리는 유지하되, 데이터 선택 버튼 영역은 더 이상 그리지 않음
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
  }

  static WifiState prev_wifiState = (WifiState)-1;
  static unsigned long lastBlinkTime = 0;
  static bool blinkState = true;
  
  bool blinkNeeded = (wifiState == WIFI_WAIT);
  if (blinkNeeded) {
     if (millis() - lastBlinkTime > 500) {
         lastBlinkTime = millis();
         blinkState = !blinkState;
         prev_wifiState = (WifiState)-1; // 강제 갱신 트리거
     }
  }

  if (wifiState != prev_wifiState || screenNeedsRedraw) {
    String statusMsg = "";
    uint16_t statusColor = COLOR_BUTTON;
    uint16_t textColor = COLOR_BUTTON_TEXT;

    if (wifiState == WIFI_CONNECTED_STATE) {
       statusMsg = "WiFi: ON";
       statusColor = COLOR_GREEN;
       textColor = ILI9341_BLACK;
    } else if (wifiState == WIFI_WAIT) {
       statusMsg = "Connecting...";
       statusColor = blinkState ? COLOR_ORANGE : COLOR_BACKGROUND;
       textColor = blinkState ? ILI9341_BLACK : COLOR_TEXT_PRIMARY;
    } else {
       statusMsg = "WiFi: OFF";
       statusColor = COLOR_BUTTON; // 회색 or 테마색
       textColor = COLOR_BUTTON_TEXT;
    }

    tft.fillRoundRect(60, 50, 200, 50, 10, statusColor);
    tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
    tft.setTextColor(textColor); 
    tft.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(statusMsg, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(60 + (200 - w)/2, 65);
    tft.print(statusMsg);
    
    prev_wifiState = wifiState;
  }
}

// --- 메인 전력 화면 값 업데이트 ---
void displayMainScreenValues() {
  if (isMainPowerFrozen) return; 

  tft.setTextSize(2);
  char buffer[20];
  int col1_value_x = 60;
  int col2_value_x = 220; 
  // [Mod] 텍스트 겹침 방지를 위해 지움 영역 너비를 충분히 확보
  const int CLEAR_WIDTH = 110;
  int col_w_half = 90; 
  int col_w_half_wide = 100;
  
  // [Mod] static 제거 후 전역 변수 사용

  printTFTValue(col1_value_x, 40, V_rms, prev_disp_V_rms, 1, COLOR_BLUE, " V");
  prev_disp_V_rms = V_rms;
  
  if (abs(I_rms - prev_disp_I_rms) > 0.001 || screenNeedsRedraw) {
      tft.fillRect(col1_value_x, 65, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half -> CLEAR_WIDTH
      tft.setTextColor(COLOR_ORANGE);
      tft.setCursor(col1_value_x, 65);
      if (I_rms < 1.0) { 
        dtostrf(I_rms * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_disp_I_rms = I_rms;
  }
  
  if (abs(P_real - prev_disp_P_real) > 0.1 || screenNeedsRedraw) {
      tft.fillRect(col1_value_x, 90, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half -> CLEAR_WIDTH
      tft.setTextColor(COLOR_DARKGREEN);
      tft.setCursor(col1_value_x, 90);
      if (P_real >= 1000.0) { 
        dtostrf(P_real / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kW");
      } else { 
        dtostrf(P_real, 4, 1, buffer); tft.print(buffer); tft.print(" W");
      }
      prev_disp_P_real = P_real;
  }

  printTFTValue(col2_value_x, 115, PF, prev_disp_PF, 2, COLOR_MAGENTA, "");
  prev_disp_PF = PF;
  
  if (abs(Q_reactive - prev_disp_Q_reactive) > 0.1 || screenNeedsRedraw) {
      tft.fillRect(col1_value_x, 115, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half -> CLEAR_WIDTH
      tft.setTextColor(COLOR_ORANGE);
      tft.setCursor(col1_value_x, 115);
      if (Q_reactive >= 1000.0) { 
        dtostrf(Q_reactive / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVAR");
      } else { 
        dtostrf(Q_reactive, 4, 1, buffer); tft.print(buffer); tft.print(" VAR");
      }
      prev_disp_Q_reactive = Q_reactive;
  }

  if (abs(I_rms_load1 - prev_disp_I_rms_load1) > 0.001 || screenNeedsRedraw) {
      tft.fillRect(col2_value_x, 40, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half_wide -> CLEAR_WIDTH
      tft.setTextColor(COLOR_RED);
      tft.setCursor(col2_value_x, 40); 
      if (I_rms_load1 < 1.0) { 
        dtostrf(I_rms_load1 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms_load1, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_disp_I_rms_load1 = I_rms_load1;
  }

  if (abs(I_rms_load2 - prev_disp_I_rms_load2) > 0.001 || screenNeedsRedraw) {
      tft.fillRect(col2_value_x, 65, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half_wide -> CLEAR_WIDTH
      tft.setTextColor(COLOR_GREEN);
      tft.setCursor(col2_value_x, 65); 
      if (I_rms_load2 < 1.0) { 
        dtostrf(I_rms_load2 * 1000.0, 4, 0, buffer); tft.print(buffer); tft.print(" mA");
      } else { 
        dtostrf(I_rms_load2, 4, 2, buffer); tft.print(buffer); tft.print(" A");
      }
      prev_disp_I_rms_load2 = I_rms_load2;
  }

  if (abs(S_apparent - prev_disp_S_apparent) > 0.1 || screenNeedsRedraw) {
      tft.fillRect(col2_value_x, 90, CLEAR_WIDTH, 20, COLOR_BACKGROUND); // col_w_half_wide -> CLEAR_WIDTH
      tft.setTextColor(COLOR_TEXT_PRIMARY);
      tft.setCursor(col2_value_x, 90); 
      if (S_apparent >= 1000.0) { 
        dtostrf(S_apparent / 1000.0, 4, 2, buffer); tft.print(buffer); tft.print(" kVA");
      } else { 
        dtostrf(S_apparent, 4, 1, buffer); tft.print(buffer); tft.print(" VA");
      }
      prev_disp_S_apparent = S_apparent;
  }

  printTFTValue(col2_value_x, 140, thd_v_value, prev_disp_thd_v_main, 1, COLOR_BLUE, " %");
  prev_disp_thd_v_main = thd_v_value;
  printTFTValue(col2_value_x, 165, thd_i_value, prev_disp_thd_i_main, 1, COLOR_ORANGE, " %");
  prev_disp_thd_i_main = thd_i_value;
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

  // ================================================================================
  // [New] Improved Drawing Logic: Erase All -> Redraw Grid -> Sort & Draw Lines
  // ================================================================================
  
  // 1. Check for changes
  bool needsUpdate = screenNeedsRedraw || 
                     (v_x != prev_v_x || v_y != prev_v_y) ||
                     (im_x != prev_im_x || im_y != prev_im_y) ||
                     (i1_x != prev_i1_x || i1_y != prev_i1_y) ||
                     (i2_x != prev_i2_x || i2_y != prev_i2_y);

  if (needsUpdate) {
      // 2. Erase all existing lines (using previous coordinates)
      if (prev_v_x != -1) tft.drawLine(PHASOR_CX, PHASOR_CY, prev_v_x, prev_v_y, COLOR_BACKGROUND);
      if (prev_im_x != -1) tft.drawLine(PHASOR_CX, PHASOR_CY, prev_im_x, prev_im_y, COLOR_BACKGROUND);
      if (prev_i1_x != -1) tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i1_x, prev_i1_y, COLOR_BACKGROUND);
      if (prev_i2_x != -1) tft.drawLine(PHASOR_CX, PHASOR_CY, prev_i2_x, prev_i2_y, COLOR_BACKGROUND);

      // 3. Restore Grid (Circles & Axes)
      tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS, COLOR_GRID); 
      tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS / 2, COLOR_GRID); 
      tft.drawFastHLine(PHASOR_CX - PHASOR_RADIUS, PHASOR_CY, PHASOR_RADIUS * 2, COLOR_GRID); 
      tft.drawFastVLine(PHASOR_CX, PHASOR_CY - PHASOR_RADIUS, PHASOR_RADIUS * 2, COLOR_GRID); 

      // 4. Sort lines by length descending (Longest -> Shortest)
      // Using a simple struct and bubble sort for 4 items
      struct Phasor { int x; int y; float len; uint16_t col; };
      Phasor lines[4] = {
          {v_x, v_y, v_len, COLOR_BLUE},
          {im_x, im_y, im_len, COLOR_ORANGE},
          {i1_x, i1_y, i1_len, COLOR_RED},
          {i2_x, i2_y, i2_len, COLOR_GREEN}
      };

      for (int i = 0; i < 3; i++) {
          for (int j = i + 1; j < 4; j++) {
              if (lines[j].len > lines[i].len) {
                  Phasor temp = lines[i];
                  lines[i] = lines[j];
                  lines[j] = temp;
              }
          }
      }

      // 5. Draw sorted lines (Shortest will be drawn last, appearing on top)
      for (int i = 0; i < 4; i++) {
          tft.drawLine(PHASOR_CX, PHASOR_CY, lines[i].x, lines[i].y, lines[i].col);
      }

      // 6. Update previous coordinates
      prev_v_x = v_x; prev_v_y = v_y;
      prev_im_x = im_x; prev_im_y = im_y;
      prev_i1_x = i1_x; prev_i1_y = i1_y;
      prev_i2_x = i2_x; prev_i2_y = i2_y;
  }
}

// --- [Mod] Y-Axis Label Update: Internal Top-Left & Color Matched ---
void updateYAxisLabels() {
  // 기존 라벨 영역 지우기
  tft.fillRect(PLOT_X_START + 1, PLOT_Y_START + 1, 200, 20, COLOR_BACKGROUND);

  tft.setTextSize(1);
  char buffer[10]; 
  int x_pos = PLOT_X_START + 5;
  int y_pos = PLOT_Y_START + 5;

  // Type 2 (I/I1/I2): 단일 통합 라벨, 검정색
  if (waveformPlotType == 2) {
    tft.setTextColor(ILI9341_BLACK); // Inst 4: Black Color
    dtostrf(plot1_axis_max, 3, 0, buffer); 
    tft.setCursor(x_pos, y_pos);
    tft.print("+"); tft.print(buffer); tft.print("A");
  } 
  // Type 0 (V/I) & Type 1 (P/Q)
  else {
    // Label 1 (V or P) -> Blue
    tft.setTextColor(COLOR_BLUE);
    dtostrf(plot1_axis_max, 3, 0, buffer);
    String unit1 = (waveformPlotType == 1) ? "W" : "V";
    tft.setCursor(x_pos, y_pos);
    tft.print("+"); tft.print(buffer); tft.print(unit1); 
    
    // Spacing
    x_pos += 60; 

    // Label 2 (I or Q) -> Orange
    tft.setTextColor(COLOR_ORANGE);
    String unit2 = (waveformPlotType == 1) ? "VAR" : "A";
    
    if (plot2_axis_max < 1.0) { 
       dtostrf(plot2_axis_max * 1000, 3, 0, buffer);
       tft.setCursor(x_pos, y_pos);
       tft.print("+"); tft.print(buffer); tft.print("m"); tft.print(unit2);
    } else { 
       dtostrf(plot2_axis_max, 3, 0, buffer);
       tft.setCursor(x_pos, y_pos);
       tft.print("+"); tft.print(buffer); tft.print(unit2);
    }
  }
}

// --- [Fix] Harmonics Y-Axis Update Logic ---
void updateHarmonicsYAxisLabels() {
   // [Mod] Clear larger area for safety, including graph
   tft.fillRect(0, 45, 320, 150, COLOR_BACKGROUND);
   displayHarmonicsScreenStatic();
}

// --- [Mod] Triggered Scope & Rolling Logic ---
void runCombinedWaveformLoop() {
  if (isWaveformFrozen) return;

  static float v_buf[PLOT_WIDTH];
  static float p2_buf[PLOT_WIDTH];
  static float p3_buf[PLOT_WIDTH];
  
  // 60Hz에서 90도 위상 지연에 필요한 시간 (T/4 = 16666.67 / 4)
  const float TARGET_90DEG_DELAY_US = 4166.67f;
  
  // [Prompt 3] 센서별 유효 승수 계산 업데이트
  float eff_V_mult = BASE_V_CALIB_RMS * V_MULTIPLIER;
  float eff_I_mult_main = BASE_I_CALIB_RMS * I_MULTIPLIER;
  float eff_I_mult_load1 = BASE_I1_CALIB_RMS * I_MULTIPLIER;
  float eff_I_mult_load2 = BASE_I2_CALIB_RMS * I_MULTIPLIER;
  
  float eff_V_off = BASE_V_OFFSET_ADJUST * V_MULTIPLIER;
  float eff_I_off = BASE_I_OFFSET_ADJUST * I_MULTIPLIER;
  
  int delay_us = WAVEFORM_DELAYS_US[waveformPeriodIndex];
  
  // [새로운 Q 파형 LAG_SIZE 계산]
  int lag_size = 0;
  if (delay_us > 0) {
    lag_size = round(TARGET_90DEG_DELAY_US / (float)delay_us);
  }
  if (lag_size < 1) lag_size = 1; // 최소 1개 샘플
  // [Q 파형 LAG_SIZE 계산 끝]

  bool triggered = false;

  // ============================================================
  // 1. 샘플링 (Sampling) 영역
  // ============================================================
  if (waveformTriggerMode == 0) { // Cont. Mode
      // FIX: 지역 변수 선언 제거 (local_lag_buf, lag_head)
    
      // [FIX] P/Q 모드이면서 버퍼가 초기화되지 않았을 경우에만 Pre-fill 실행
      if (waveformPlotType == 1 && !IS_LAG_BUFFER_INIT) {
          P_Q_LAG_HEAD = 0; // [FIX] Use Global Head and reset
          for(int k=0; k<lag_size; k++) {
              int r_v = analogRead(PIN_ADC_V);
              float v = (r_v - V_ADC_MIDPOINT_CALIB) * eff_V_mult + eff_V_off;
              P_Q_LAG_BUFFER[P_Q_LAG_HEAD] = v; // [FIX] Use Global Buffer
              P_Q_LAG_HEAD = (P_Q_LAG_HEAD + 1) % lag_size; // [FIX] Use Global Head
              delayMicroseconds(delay_us);
          }
          IS_LAG_BUFFER_INIT = true; // [FIX] Set init flag
      }
    
      // [New] P/Q 모드가 아닐 경우, 다음 진입을 위해 플래그 리셋
      if (waveformPlotType != 1) {
          IS_LAG_BUFFER_INIT = false;
      }

      for (int i = 0; i < PLOT_WIDTH; i++) {
          unsigned long t0 = micros();
          
          int r_v = analogRead(PIN_ADC_V);
          int r_i = analogRead(PIN_ADC_I);
          int r_i1 = analogRead(PIN_ADC_I1);
          int r_i2 = analogRead(PIN_ADC_I2);
          
          float v = (r_v - V_ADC_MIDPOINT_CALIB) * eff_V_mult + eff_V_off;
          
          // [Prompt 3] 센서별 유효 승수 적용
          float cur = (r_i - I_ADC_MIDPOINT_CALIB) * eff_I_mult_main - eff_I_off;
          float cur1 = (r_i1 - I1_ADC_MIDPOINT_CALIB) * eff_I_mult_load1 - eff_I_off;
          float cur2 = (r_i2 - I2_ADC_MIDPOINT_CALIB) * eff_I_mult_load2 - eff_I_off;

          if (waveformPlotType == 0) { // V/I
             v_buf[i] = v;
             p2_buf[i] = cur;
          } 
          else if (waveformPlotType == 1) { // P/Q
             float p = v * cur;
             v_buf[i] = p; 
             
             // [FIX] Use Global Static Buffer
             P_Q_LAG_BUFFER[P_Q_LAG_HEAD] = v;
             int lag_read_idx = (P_Q_LAG_HEAD + 1) % lag_size;
             float v_lag = P_Q_LAG_BUFFER[lag_read_idx];
             P_Q_LAG_HEAD = lag_read_idx;
             
             float q = v_lag * cur;
             p2_buf[i] = q;
          } 
          else { // I/I1/I2
             v_buf[i] = cur;
             p2_buf[i] = cur1; 
             p3_buf[i] = cur2;
          }
          
          checkSerialInput();
          checkTouchInput();
          if (screenNeedsRedraw) return;

          while (micros() - t0 < delay_us);
      }
  } 
  else { // Trig. / Single Mode
      unsigned long trigger_start = millis();
      int hyst = 50;
      int trig_state = 0;
      
      while (millis() - trigger_start < 50) {
         checkTouchInput();
         if (screenNeedsRedraw) return;
         int raw_v = analogRead(PIN_ADC_V);
         if (trig_state == 0) { if (raw_v < (V_ADC_MIDPOINT_CALIB - hyst)) trig_state = 1; } 
         else if (trig_state == 1) { if (raw_v > (V_ADC_MIDPOINT_CALIB + hyst)) { triggered = true; break; } }
      }

      float local_lag_buf[lag_size];
      int lag_head = 0;
      for(int k=0; k<lag_size; k++) {
          int r_v = analogRead(PIN_ADC_V);
          float v = (r_v - V_ADC_MIDPOINT_CALIB) * eff_V_mult + eff_V_off;
          local_lag_buf[lag_head] = v;
          lag_head = (lag_head + 1) % lag_size;
          delayMicroseconds(delay_us);
      }
      
      for (int i = 0; i < PLOT_WIDTH; i++) {
        unsigned long t0 = micros();
        int r_v = analogRead(PIN_ADC_V);
        int r_i = analogRead(PIN_ADC_I);
        int r_i1 = analogRead(PIN_ADC_I1);
        int r_i2 = analogRead(PIN_ADC_I2);

        float v = (r_v - V_ADC_MIDPOINT_CALIB) * eff_V_mult + eff_V_off;
        
        // [Prompt 3] 센서별 유효 승수 적용
        float cur = (r_i - I_ADC_MIDPOINT_CALIB) * eff_I_mult_main - eff_I_off;
        float cur1 = (r_i1 - I1_ADC_MIDPOINT_CALIB) * eff_I_mult_load1 - eff_I_off;
        float cur2 = (r_i2 - I2_ADC_MIDPOINT_CALIB) * eff_I_mult_load2 - eff_I_off;

        if (waveformPlotType == 0) { 
           v_buf[i] = v;
           p2_buf[i] = cur;
        } 
        else if (waveformPlotType == 1) { 
           float p = v * cur;
           v_buf[i] = p; 
           
           local_lag_buf[lag_head] = v;
           int lag_read_idx = (lag_head + 1) % lag_size;
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
  // 2. 오토 레인징 (Auto-ranging)
  // ============================================================
  float max_val_p1 = 0.0, max_val_p2 = 0.0, max_val_p3 = 0.0;
  for(int i=0; i<PLOT_WIDTH; i++) {
     if (abs(v_buf[i]) > max_val_p1) max_val_p1 = abs(v_buf[i]);
     if (abs(p2_buf[i]) > max_val_p2) max_val_p2 = abs(p2_buf[i]);
     if (waveformPlotType == 2 && abs(p3_buf[i]) > max_val_p3) max_val_p3 = abs(p3_buf[i]);
  }

  // [Fix] Removed static declaration, using global range indices
  bool rangeChanged = false;

  auto updateRange = [&](int &idx, float peak, const float* ranges, int count) {
      bool changed = false;
      if (peak > ranges[idx]) {
          while (idx < count - 1 && peak > ranges[idx]) idx++;
          changed = true;
      } else if (idx > 0) {
          if (peak < (ranges[idx-1] * 0.6)) { idx--; changed = true; }
      }
      return changed;
  };

  if (waveformPlotType == 0) { // V/I
     if (updateRange(range_idx_v, max_val_p1, V_RANGES, NUM_V_RANGES)) rangeChanged = true;
     if (updateRange(range_idx_i, max_val_p2, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     plot1_axis_max = V_RANGES[range_idx_v]; 
     plot2_axis_max = I_RANGES[range_idx_i];
  } 
  else if (waveformPlotType == 1) { // P/Q
     float max_pq = max(max_val_p1, max_val_p2);
     if (updateRange(range_idx_p, max_pq, P_RANGES, NUM_P_RANGES)) rangeChanged = true;
     plot1_axis_max = P_RANGES[range_idx_p]; 
     plot2_axis_max = P_RANGES[range_idx_p];
  } 
  else { // I/I1/I2
     float global_max = max(max_val_p1, max(max_val_p2, max_val_p3));
     if (updateRange(range_idx_i, global_max, I_RANGES, NUM_I_RANGES)) rangeChanged = true;
     plot1_axis_max = I_RANGES[range_idx_i]; 
     plot2_axis_max = I_RANGES[range_idx_i]; 
     plot3_axis_max = I_RANGES[range_idx_i];
  }

  if (rangeChanged) {
     updateYAxisLabels();
     tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  }
  
  // ============================================================
  // 3. 그리기 (Drawing)
  // ============================================================
  float scale1 = (plot1_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot1_axis_max);
  float scale2 = (plot2_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot2_axis_max);
  float scale3 = (plot3_axis_max == 0) ? 0 : (PLOT_HEIGHT_HALF / plot3_axis_max); 

  int py1_old = PLOT_Y_CENTER, py2_old = PLOT_Y_CENTER, py3_old = PLOT_Y_CENTER; 
  int py1_new = PLOT_Y_CENTER, py2_new = PLOT_Y_CENTER, py3_new = PLOT_Y_CENTER;

  tft.startWrite();
  for (int i = 0; i < PLOT_WIDTH; i++) {
     int x = PLOT_X_START + i;
     
     int cy1_old = last_frame_y_plot1[i];
     int cy2_old = last_frame_y_plot2[i];
     int cy3_old = last_frame_y_plot3[i];

     int y1 = constrain(PLOT_Y_CENTER - (int)(v_buf[i] * scale1), PLOT_Y_START, PLOT_Y_END);
     int y2 = constrain(PLOT_Y_CENTER - (int)(p2_buf[i] * scale2), PLOT_Y_START, PLOT_Y_END);
     int y3 = PLOT_Y_CENTER;
     if (waveformPlotType == 2) y3 = constrain(PLOT_Y_CENTER - (int)(p3_buf[i] * scale3), PLOT_Y_START, PLOT_Y_END);

     if (i > 0) {
         int xp = x - 1;
         
         // [Clear] 이전 선 지우기
         if (cy1_old != PLOT_Y_CENTER || py1_old != PLOT_Y_CENTER)
            tft.drawLine(xp, py1_old, x, cy1_old, COLOR_BACKGROUND);
         if (cy2_old != PLOT_Y_CENTER || py2_old != PLOT_Y_CENTER)
            tft.drawLine(xp, py2_old, x, cy2_old, COLOR_BACKGROUND);
         if (waveformPlotType == 2) {
            if (cy3_old != PLOT_Y_CENTER || py3_old != PLOT_Y_CENTER)
                tft.drawLine(xp, py3_old, x, cy3_old, COLOR_BACKGROUND);
         }

         // [Draw] 새로운 선 그리기
         tft.drawLine(xp, py1_new, x, y1, COLOR_BLUE);
         tft.drawLine(xp, py2_new, x, y2, COLOR_ORANGE);
         if (waveformPlotType == 2) tft.drawLine(xp, py3_new, x, y3, COLOR_RED);
     }
     
     last_frame_y_plot1[i] = y1; 
     last_frame_y_plot2[i] = y2; 
     last_frame_y_plot3[i] = y3;

     py1_old = cy1_old; py2_old = cy2_old; py3_old = cy3_old;
     py1_new = y1;      py2_new = y2;      py3_new = y3;
  }
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID);
  tft.endWrite();
  
  if (waveformTriggerMode == 2 && triggered) isWaveformFrozen = true;
  
  checkSerialInput(); 
}

// HARMONICS 화면 업데이트
void displayHarmonicsScreenValues() {
  if (harmonicsSrcLabel != prev_harmonicsSrcLabel) {
     drawButton(10, 200, 90, 35, harmonicsSrcLabel);
     prev_harmonicsSrcLabel = harmonicsSrcLabel;
     updateHarmonicsYAxisLabels(); 
     // 소스 변경 시 전체 다시 그리기 위해 초기화 (그래프 및 텍스트)
     for(int i=0; i<16; i++) {
         prev_bar_heights[i] = -1;
         prev_text_vals[i] = -1.0; // [Mod] 텍스트 갱신 강제화
     }
  }
  
  String currRunLabel = isHarmonicsFrozen ? "RUN" : "HOLD";
  if (currRunLabel != prev_harmonicsRunLabel) {
     drawButton(110, 200, 100, 35, currRunLabel);
     prev_harmonicsRunLabel = currRunLabel;
  }

  if (isHarmonicsFrozen) return;

  float* dataPtr = (harmonicsSource == 0) ? v_harmonics : i_harmonics;
  float fundamental_rms = (harmonicsSource == 0) ? V_rms : I_rms;
  
  if (harmonicsViewMode == 0) {
     int graph_x = 55;
     int graph_y = 50; 
     int graph_h = 130;
     int graph_w = 255;
     int bar_w = graph_w / 8;
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

        int x_pos = graph_x + (i - 1) * bar_w + 2;
        uint16_t color = (harmonicsSource == 0) ? COLOR_BLUE : COLOR_ORANGE;
        
        int prev_h = prev_bar_heights[i];
        int curr_h = bar_h;

        // [Mod] 이전 막대 길이와 비교하여 부분 갱신 (지우개 현상 방지)
        if (curr_h != prev_h) {
            if (curr_h > prev_h) {
                // 1. 막대가 길어진 경우: 윗부분만 추가로 그리기
                // 좌표계: y값은 아래로 갈수록 증가. 바닥 = graph_y + graph_h
                // 이전 Top (prev_y) = graph_y + graph_h - prev_h
                // 현재 Top (curr_y) = graph_y + graph_h - curr_h (더 작음, 즉 더 위쪽)
                int curr_y = graph_y + graph_h - curr_h;
                // 그릴 높이: curr_h - prev_h
                tft.fillRect(x_pos, curr_y, bar_w - 4, curr_h - prev_h, color);
            } else {
                // 2. 막대가 짧아진 경우: 줄어든 부분만 지우고 그리드 복구
                int prev_y = graph_y + graph_h - prev_h; // 지워야 할 영역의 Top
                int curr_y = graph_y + graph_h - curr_h; // 지워야 할 영역의 Bottom (새 막대 Top)
                
                // 지우기
                tft.fillRect(x_pos, prev_y, bar_w - 4, prev_h - curr_h, COLOR_BACKGROUND);
                
                // 해당 영역에 그리드 선이 있으면 다시 그리기
                for (int j = 0; j <= 3; j++) {
                    int line_y = graph_y + (j * (graph_h / 3));
                    if (j == 3) line_y = graph_y + graph_h;
                    
                    // 선이 지워진 영역 [prev_y, curr_y] 사이에 있는지 확인
                    // 주의: prev_y가 더 작음(위쪽)
                    if (line_y >= prev_y && line_y <= curr_y) {
                        tft.drawFastHLine(x_pos, line_y, bar_w - 4, COLOR_GRID);
                    }
                }
                // 새 막대 상단 보정 (혹시 모를 픽셀 오차 방지용, 필요한 경우)
                // tft.fillRect(x_pos, curr_y, bar_w - 4, 1, color); 
            }
            prev_bar_heights[i] = curr_h;
        }
        // 초기 상태(-1)에서 바로 그리는 경우 처리
        else if (prev_h == -1) {
             int curr_y = graph_y + graph_h - curr_h;
             tft.fillRect(x_pos, curr_y, bar_w - 4, curr_h, color);
             prev_bar_heights[i] = curr_h;
        }
     }
  } else {
     // [Mod] Text Mode: Unified list for 8 odd harmonics
     tft.setTextSize(2); tft.setTextColor(COLOR_TEXT_PRIMARY);
     int col1_x = 20; int col2_x = 175; 
     int start_y = 85; int line_h = 25;
     
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
             
             // [Mod] Correct Labeling (1, 3, 5... 15)
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
      // [Prompt 4] 불필요한 eff_mult 계산 제거 및 직접 RMS 값 표시
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
    // [Fix] 강제 OFF 로직 제거, 실제 변수 상태만 반영
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
  // [Mod] 전역 변수 사용
  static uint32_t prev_display_time = 0xFFFFFFFF; // 함수 내 static은 제거하거나 초기값용으로 유지하되 아래 로직은 전역 변수 사용 권장
  // 여기서는 resetViewStates에서 초기화되는 prev_timer_display_time 사용

  if (display_time != prev_timer_display_time || is_timer_active != prev_timer_active_state || screenNeedsRedraw) {
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
     prev_timer_display_time = display_time;
     prev_timer_active_state = is_timer_active;
  }

  if (timer_step_index != prev_timer_step_index || screenNeedsRedraw) {
     drawButton(90, 150, 60, 40, TIMER_STEP_LABELS[timer_step_index]);
     prev_timer_step_index = timer_step_index;
  }

  if (is_timer_active != prev_is_timer_active || screenNeedsRedraw) {
     String actionLabel = is_timer_active ? "STOP" : "START";
     uint16_t color = is_timer_active ? COLOR_RED : COLOR_GREEN;
     
     tft.fillRoundRect(230, 150, 80, 40, 8, color);
     tft.drawRoundRect(230, 150, 80, 40, 8, COLOR_BUTTON_OUTLINE);
     tft.setTextColor(ILI9341_WHITE);
     tft.setTextSize(2);
     tft.setCursor(240, 162); 
     tft.print(actionLabel);
     
     prev_is_timer_active = is_timer_active;
  }
}