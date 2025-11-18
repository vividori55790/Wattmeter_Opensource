/*
 * ==============================================================================
 * 파일명: 2. View_Static.ino
 * 버전: v80 (Phasor Button Fix)
 * 최종 수정: 2025-11-21
 *
 * [변경 사항]
 * 1. displayPhaseScreenStatic(): "HOLD/RUN" 버튼 크기를 80x35 -> 60x25로 축소.
 * 2. drawWaveformGridAndLabels(): 그래프 그리드 그리기 로직 확인 (상수 변경 자동 반영).
 * ==============================================================================
 */

// --- 테마 설정 함수 ---
void setTheme() {
  if (isDarkMode) {
    COLOR_BACKGROUND = ILI9341_BLACK;
    COLOR_TEXT_PRIMARY = ILI9341_WHITE;
    COLOR_TEXT_SECONDARY = ILI9341_LIGHTGREY;
    COLOR_BUTTON = ILI9341_NAVY;
    COLOR_BUTTON_TEXT = ILI9341_WHITE;
    COLOR_BUTTON_OUTLINE = ILI9341_DARKGREY;
    COLOR_GRID = ILI9341_DARKGREY;
    COLOR_RED = ILI9341_PINK;       
    COLOR_GREEN = ILI9341_GREENYELLOW; 
    COLOR_BLUE = ILI9341_CYAN;       
    COLOR_ORANGE = ILI9341_YELLOW;     
    COLOR_MAGENTA = ILI9341_MAGENTA;   
    COLOR_DARKGREEN = ILI9341_GREEN;     
  } else {
    COLOR_BACKGROUND = ILI9341_WHITE;
    COLOR_TEXT_PRIMARY = ILI9341_BLACK;
    COLOR_TEXT_SECONDARY = ILI9341_DARKGREY;
    COLOR_BUTTON = ILI9341_BLUE;
    COLOR_BUTTON_TEXT = ILI9341_WHITE;
    COLOR_BUTTON_OUTLINE = ILI9341_BLACK;
    COLOR_GRID = ILI9341_LIGHTGREY;
    COLOR_RED = ILI9341_RED;
    COLOR_GREEN = ILI9341_GREEN;
    COLOR_BLUE = ILI9341_BLUE;
    COLOR_ORANGE = ILI9341_ORANGE;
    COLOR_MAGENTA = ILI9341_MAGENTA;
    COLOR_DARKGREEN = ILI9341_DARKGREEN;
  }
}

// --- 네트워크 상태 표시 (동적) ---
void displayNetworkStatus() {
  tft.setTextSize(1);
  if (isWifiConnected) {
    tft.setTextColor(COLOR_BLUE); 
    tft.setCursor(240, 5); 
    tft.print("NET: ON ");
  } else {
    tft.setTextColor(COLOR_TEXT_SECONDARY);
    tft.setCursor(240, 5); 
    tft.print("NET: OFF");
  }
}

// --- 뒤로 가기 버튼 그리기 ---
void drawBackButton() {
  tft.fillRoundRect(5, 5, 50, 30, 8, COLOR_TEXT_SECONDARY); 
  tft.setCursor(20, 12);
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setTextSize(2);
  tft.print("<");
}

// --- 공통 버튼 그리기 헬퍼 ---
void drawButton(int x, int y, int w, int h, String text) {
  tft.fillRoundRect(x, y, w, h, 8, COLOR_BUTTON); 
  tft.drawRoundRect(x, y, w, h, 8, COLOR_BUTTON_OUTLINE); 
  tft.setTextColor(COLOR_BUTTON_TEXT); 
  tft.setTextSize(2);
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(x + (w - w1) / 2, y + (h - h1) / 2);
  tft.print(text);
}

// --- 홈 화면 ---
void displayHomeScreenStatic() {
  tft.setCursor(50, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(3);
  tft.print("KW WATT-METER");
  
  drawButton(20, 50, 130, 40, "MAIN POWER");
  drawButton(170, 50, 130, 40, "PHASOR");
  drawButton(20, 110, 130, 40, "WAVEFORM");
  drawButton(170, 110, 130, 40, "HARMONICS"); 
  drawButton(20, 170, 130, 40, "SETTINGS");
  drawButton(170, 170, 130, 40, "RELAY CTRL");
}

// --- 메인 전력 화면 ---
void displayMainScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("MAIN POWER (Live)");
  drawBackButton(); 
  tft.setTextSize(2);
  
  int col1_label_x = 10;
  int col2_label_x = 170;
  
  tft.setCursor(col1_label_x, 40); tft.setTextColor(COLOR_BLUE); tft.println("V:");
  tft.setCursor(col1_label_x, 65); tft.setTextColor(COLOR_ORANGE); tft.println("I:");
  tft.setCursor(col1_label_x, 90); tft.setTextColor(COLOR_DARKGREEN); tft.println("P:");
  tft.setCursor(col2_label_x, 115); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:");
  tft.setCursor(col1_label_x, 115); tft.setTextColor(COLOR_ORANGE); tft.println("Q:"); 
  tft.setCursor(col2_label_x, 40); tft.setTextColor(COLOR_RED); tft.println("I-1:");
  tft.setCursor(col2_label_x, 65); tft.setTextColor(COLOR_GREEN); tft.println("I-2:");
  tft.setCursor(col2_label_x, 90); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("S:");
  tft.setCursor(col1_label_x, 140); tft.setTextColor(COLOR_BLUE); tft.println("THD-V:"); 
  tft.setCursor(col1_label_x, 165); tft.setTextColor(COLOR_ORANGE); tft.println("THD-I:");

  String btnText = isMainPowerFrozen ? "RUN" : "HOLD";
  drawButton(20, 195, 80, 35, btnText);

  tft.setTextSize(1); 
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(115, 200);
  tft.print("V Mult: x"); tft.print(V_MULTIPLIER, 1);
  tft.setCursor(115, 215);
  tft.print("I Mult: x"); tft.print(I_MULTIPLIER, 1);
}

// --- 위상차 화면 ---
void displayPhaseScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("PHASOR DIAGRAM"); 
  drawBackButton(); 
  
  tft.setTextSize(2);
  tft.setCursor(10, 50); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 75); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("Status:"); 
  
  tft.setTextSize(1);
  tft.setCursor(10, 125); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("--- Phase (deg) ---"); 
  tft.setTextSize(2);
  tft.setCursor(10, 140); tft.setTextColor(COLOR_ORANGE); tft.println("I-M:"); 
  tft.setCursor(10, 165); tft.setTextColor(COLOR_RED); tft.println("I-1:"); 
  tft.setCursor(10, 190); tft.setTextColor(COLOR_GREEN); tft.println("I-2:"); 

  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS, COLOR_GRID); 
  tft.drawCircle(PHASOR_CX, PHASOR_CY, PHASOR_RADIUS / 2, COLOR_GRID); 
  tft.drawFastHLine(PHASOR_CX - PHASOR_RADIUS, PHASOR_CY, PHASOR_RADIUS * 2, COLOR_GRID); 
  tft.drawFastVLine(PHASOR_CX, PHASOR_CY - PHASOR_RADIUS, PHASOR_RADIUS * 2, COLOR_GRID); 
  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY); 
  tft.setCursor(PHASOR_CX + PHASOR_RADIUS + 3, PHASOR_CY - 3);
  tft.print("0");                                                           
  int legendY = 200; 
  tft.drawRect(155, legendY, 160, 35, COLOR_TEXT_SECONDARY); 
  tft.drawFastHLine(160, legendY + 17, 10, COLOR_BLUE); 
  tft.setCursor(175, legendY + 4); tft.print("V");
  tft.drawFastHLine(200, legendY + 17, 10, COLOR_ORANGE); 
  tft.setCursor(215, legendY + 4); tft.print("I-M");
  tft.drawFastHLine(240, legendY + 17, 10, COLOR_RED); 
  tft.setCursor(255, legendY + 4); tft.print("I-1");
  tft.drawFastHLine(280, legendY + 17, 10, COLOR_GREEN); 
  tft.setCursor(295, legendY + 4); tft.print("I-2");
  
  // [수정] 버튼 크기 축소 (80x35 -> 60x25) 및 위치 조정 (Y: 200 -> 205)
  String btnText = isPhaseFrozen ? "RUN" : "HOLD";
  drawButton(20, 205, 60, 25, btnText);
}

// --- HARMONICS 화면 ---
void displayHarmonicsScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("HARMONICS ANALYSIS"); 
  drawBackButton(); 
  
  drawButton(10, 200, 90, 35, "Src:V"); 
  drawButton(110, 200, 100, 35, "RUN"); 
  drawButton(220, 200, 90, 35, "View"); 

  if (harmonicsViewMode == 0) { 
     int graph_x = 40;
     int graph_y = 45;
     int graph_w = 270;
     int graph_h = 145; 
     
     tft.drawRect(graph_x, graph_y, graph_w, graph_h, COLOR_GRID);
     
     tft.drawFastHLine(graph_x, graph_y, graph_w, COLOR_GRID);
     tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
     tft.setCursor(5, graph_y-3); tft.print("100%");

     int y_10 = graph_y + graph_h * 1 / 3;
     tft.drawFastHLine(graph_x, y_10, graph_w, COLOR_GRID);
     tft.setCursor(10, y_10-3); tft.print("10%");

     int y_1 = graph_y + graph_h * 2 / 3;
     tft.drawFastHLine(graph_x, y_1, graph_w, COLOR_GRID);
     tft.setCursor(15, y_1-3); tft.print("1%");

     tft.setCursor(5, graph_y + graph_h - 3); tft.print("0.1%");

     int bar_w = graph_w / 15;
     for(int i=1; i<=15; i+=2) { 
        int x_pos = graph_x + (i-1)*bar_w + bar_w/2 - 3;
        tft.setCursor(x_pos, graph_y + graph_h + 2);
        tft.print(i);
     }

  } else { 
     tft.setTextSize(2);
     tft.setTextColor(COLOR_TEXT_PRIMARY);
     tft.setCursor(10, 50); tft.print("Ord");
     tft.setCursor(60, 50); tft.print("%");
     tft.setCursor(120, 50); tft.print("Val");
     
     tft.setCursor(170, 50); tft.print("Ord");
     tft.setCursor(220, 50); tft.print("%");
     tft.setCursor(280, 50); tft.print("Val");
     
     tft.drawFastHLine(10, 70, 300, COLOR_GRID);
     tft.drawFastVLine(160, 50, 140, COLOR_GRID); 
  }
}

// --- 설정 메인 화면 ---
void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("SETTINGS");
  displayNetworkStatus(); 
  drawBackButton();
  
  drawButton(20, 50, 280, 35, "CALIBRATION");
  drawButton(20, 90, 280, 35, "PROTECTION");
  drawButton(20, 130, 280, 35, "NETWORK"); 
  drawButton(20, 170, 280, 35, "ADVANCED");
}

// --- 네트워크 설정 화면 ---
void displaySettingsNetworkStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("NETWORK SETTINGS");
  drawBackButton();

  // 1. WiFi Toggle Switch
  tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
  
  // 2. Data Selection Header
  tft.setCursor(20, 115);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.print("Data to Send (ThingSpeak):");
}

// --- 캘리브레이션 설정 화면 ---
void displaySettingsCalibStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("CALIBRATION SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(30, 50); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("V Mult:"); 
  tft.setCursor(30, 85); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("I Mult:"); 
  tft.setCursor(30, 120); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Step:"); 

  drawButton(20, 180, 60, 40, "UP");
  drawButton(90, 180, 60, 40, "DOWN");
  drawButton(180, 180, 60, 40, "-");
  drawButton(250, 180, 60, 40, "+");
  
  prev_calib_selection = -1; 
  prev_V_MULTIPLIER = -1.0;
  prev_I_MULTIPLIER = -1.0;
  prev_setting_step_index = -1;
}

// --- 보호 설정 화면 ---
void displaySettingsProtectStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("PROTECTION SETTINGS");
  drawBackButton();

  tft.setTextSize(2);
  tft.setCursor(30, 70); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Over V:"); 
  tft.setCursor(30, 110); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.print("Step:"); 

  drawButton(20, 180, 60, 40, "UP");
  drawButton(90, 180, 60, 40, "DOWN");
  drawButton(180, 180, 60, 40, "-");
  drawButton(250, 180, 60, 40, "+");
  
  prev_protect_selection = -1;
  prev_VOLTAGE_THRESHOLD = -1.0;
  prev_setting_step_index = -1;
}

// --- 릴레이 제어 화면 ---
void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton();
  prev_r1_state = !relay1_state;
  prev_r2_state = !relay2_state;
}

// --- 테마 설정 화면 ---
void displaySettingsThemeStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("THEME SETTINGS");
  drawBackButton();
  drawButton(20, 70, 280, 40, "LIGHT MODE");
  drawButton(20, 130, 280, 40, "DARK MODE");
  tft.setTextColor(COLOR_RED);
  tft.setTextSize(2);
  if (isDarkMode) {
    tft.setCursor(5, 140);
  } else {
    tft.setCursor(5, 80);
  }
  tft.print(">");
}

// --- 설정 초기화 화면 ---
void displaySettingsResetStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RESET SETTINGS");
  drawBackButton();
  tft.setCursor(20, 60);
  tft.print("Reset all values to");
  tft.setCursor(20, 80);
  tft.print("factory defaults?");
  drawButton(20, 100, 130, 40, "RESET");
  drawButton(170, 100, 130, 40, "CANCEL");
}

// --- 저장 확인 화면 ---
void displayConfirmSaveStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("SAVE CHANGES?");
  drawBackButton();
  tft.setCursor(20, 70);
  tft.print("Save the new settings?");
  drawButton(20, 100, 130, 40, "SAVE");
  drawButton(170, 100, 130, 40, "DISCARD");
}

// --- 고급 설정 화면 ---
void displaySettingsAdvancedStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("ADVANCED SETTINGS");
  drawBackButton();
  drawButton(20, 60, 280, 35, "THEME");
  drawButton(20, 105, 280, 35, "TIMER");
  drawButton(20, 150, 280, 35, "RESET");
}

// --- 타이머 설정 화면 ---
void displaySettingsTimerStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RELAY TIMER SETTINGS");
  drawBackButton();
  
  tft.setTextSize(2);
  tft.setCursor(20, 45); tft.print("Set Duration:");
  
  drawButton(20, 115, 60, 50, "-");    
  drawButton(90, 115, 60, 50, "STEP"); 
  drawButton(160, 115, 60, 50, "+");   
  
  String actionLabel = is_timer_active ? "STOP" : "START";
  drawButton(230, 115, 80, 50, actionLabel); 
  
  tft.setTextSize(2);
  tft.setCursor(20, 185); tft.print("Step:");
  
  tft.setCursor(20, 215); tft.print("Status:");
  tft.setCursor(160, 215); tft.print("Left:");
  
  prev_temp_timer_setting_seconds = 0xFFFFFFFF;
  prev_timer_step_index = -1;
}

// --- 경고 화면 ---
void displayWarningScreenStatic() {
  tft.fillScreen(ILI9341_RED);
  tft.drawRect(5, 5, SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10, ILI9341_WHITE);
  tft.drawRect(6, 6, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 12, ILI9341_WHITE);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3); 
  tft.setCursor(75, 40); 
  tft.print("WARNING!"); 
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2); 
  
  int16_t x1, y1;
  uint16_t w1, h1;
  tft.getTextBounds(warningMessage, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 85); 
  tft.print(warningMessage);
  
  if (warningMessage.startsWith("TIMER")) {
    // 값 표시 안 함
  } else {
    tft.setTextSize(3); 
    char buffer[20];
    if (warningMessage.startsWith("OVER VOLTAGE")) {
      dtostrf(V_rms, 4, 1, buffer);
      strcat(buffer, " V");
    } else {
      dtostrf(I_rms, 4, 2, buffer);
      strcat(buffer, " A");
    }
    tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w1, &h1);
    tft.setCursor((SCREEN_WIDTH - w1) / 2, 125); 
    tft.print(buffer);
  }
  
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2); 
  String resetMsg = "Tap screen to reset";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); 
  tft.print(resetMsg); 
}

// --- 파형 화면 (그리드 및 라벨, 버튼) ---
void drawWaveformGridAndLabels() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("WAVEFORM (60Hz)");
  drawBackButton(); 
  
  // [수정] PLOT_X_START 등은 Controller.ino에 정의된 상수를 따름 (30으로 변경됨)
  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), COLOR_GRID); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 
  tft.drawFastVLine(PLOT_X_END, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 

  tft.setTextSize(1);
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setCursor(PLOT_X_END + 5, PLOT_Y_CENTER - 4); 
  tft.print("0"); 
  tft.setCursor(0, PLOT_Y_CENTER - 4); 
  tft.print("0"); 
  
  drawButton(10, 195, 100, 35, WAVEFORM_TYPE_LABELS[waveformPlotType]);
  drawButton(115, 195, 100, 35, WAVEFORM_MODE_LABELS[waveformTriggerMode]); 
  drawButton(220, 195, 90, 35, WAVEFORM_PERIOD_LABELS[waveformPeriodIndex]);
}