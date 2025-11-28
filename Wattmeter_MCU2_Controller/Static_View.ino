/*
 * ==============================================================================
 * 파일명: 2. Static_View.ino
 * 버전: v214_Mod_UI (Credit UI Improved)
 * 설명: 
 * - [Mod] Harmonics Screen: 그래프 영역 최적화 및 Y축 단위 즉시 반영 구조
 * - [Mod] X축 라벨: 1, 3, 5 ... 15 홀수 차수 표시
 * - [Mod] Legend Color Fix 유지
 * - [Mod] Settings UI 변경 (5버튼 배열) 및 Advanced UI 변경 (2x2 그리드)
 * - [New] Credit Splash Screen & Member Profile Card UI Design
 * ==============================================================================
 */

// 광운대학교 상징색 (Deep Red/Burgundy) 정의
#define COLOR_KW_BURGUNDY 0x8000 

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

void displayNetworkStatus() {
  tft.setTextSize(1);
  if (wifiState == WIFI_CONNECTED_STATE) {
    tft.setTextColor(COLOR_BLUE); 
    tft.setCursor(240, 5); 
    tft.print("NET: ON ");
  } else if (wifiState == WIFI_WAIT) {
    tft.setTextColor(COLOR_ORANGE);
    tft.setCursor(240, 5);
    tft.print("NET: WAIT");
  } else {
    tft.setTextColor(COLOR_TEXT_SECONDARY);
    tft.setCursor(240, 5); 
    tft.print("NET: OFF");
  }
}

void drawBackButton() {
  tft.fillRoundRect(5, 5, 50, 30, 8, COLOR_TEXT_SECONDARY); 
  tft.setCursor(20, 12);
  tft.setTextColor(COLOR_BACKGROUND); 
  tft.setTextSize(2);
  tft.print("<");
}

void drawButton(int x, int y, int w, int h, String text) {
  // 1. 애니메이션 효과 (중앙에서 확장)
  int centerX = x + w / 2;
  int centerY = y + h / 2;
  int steps = 4; // 애니메이션 단계 (숫자가 클수록 부드럽지만 느림)

  for (int i = 1; i <= steps; i++) {
    int cw = (w * i) / steps;
    int ch = (h * i) / steps;
    
    // 확장되는 테두리 그리기 (버튼 외곽선 색상 사용)
    tft.drawRoundRect(centerX - cw / 2, centerY - ch / 2, cw, ch, 8, COLOR_BUTTON_OUTLINE);
    
    delay(10); // 애니메이션 속도 조절 (너무 느리면 반응성 저하)
    
    if (i < steps) {
      // 다음 프레임을 위해 이전 테두리 지우기 (현재 배경색으로 덮어쓰기)
      tft.drawRoundRect(centerX - cw / 2, centerY - ch / 2, cw, ch, 8, COLOR_BACKGROUND);
    }
  }

  // 2. 최종 버튼 그리기 (기존 로직)
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

void displayPhaseScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("PHASOR DIAGRAM"); 
  drawBackButton(); 
  
  tft.setTextSize(2);
  tft.setCursor(10, 50); tft.setTextColor(COLOR_MAGENTA); tft.println("PF:"); 
  tft.setCursor(10, 75); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.println("Status(I_M):"); 
  
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
  
  String btnText = isPhaseFrozen ? "RUN" : "HOLD";
  drawButton(20, 205, 60, 25, btnText);
}

// --- [Mod] Improved Harmonics Static Display ---
void displayHarmonicsScreenStatic() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("HARMONICS (Log)"); 
  drawBackButton(); 
  
  drawButton(10, 200, 90, 35, harmonicsSrcLabel); 
  drawButton(110, 200, 100, 35, isHarmonicsFrozen ? "RUN" : "HOLD"); 
  drawButton(220, 200, 90, 35, "View"); 

  if (harmonicsViewMode == 0) {
     // [Mod] Adjusted Graph Area for Better Y-Axis Label Visibility
     int graph_x = 55;  // Left margin increased for Y-axis labels
     int graph_y = 50;  
     int graph_w = 255; 
     int graph_h = 130; // Height adjusted to avoid overlapping buttons
     
     tft.drawRect(graph_x, graph_y, graph_w, graph_h, COLOR_GRID);
     
     tft.setTextSize(1); tft.setTextColor(COLOR_TEXT_SECONDARY);
     
     // Draw Grid Lines (3 equal divisions for Log scale)
     for (int i=0; i<=3; i++) {
         int line_y = graph_y + (i * (graph_h / 3));
         // Ensure last line is exactly at bottom
         if (i == 3) line_y = graph_y + graph_h;
         
         tft.drawFastHLine(graph_x, line_y, graph_w, COLOR_GRID);
         
         // Y-Axis Labels based on Source
         if (harmonicsSource == 0) { // Voltage
             if(i==0) { tft.setCursor(15, line_y-3); tft.print("1000V"); }
             else if(i==1) { tft.setCursor(20, line_y-3); tft.print("100V"); }
             else if(i==2) { tft.setCursor(25, line_y-3); tft.print("10V"); }
             else if(i==3) { tft.setCursor(30, line_y-3); tft.print("1V"); }
         } else { // Current
             if(i==0) { tft.setCursor(25, line_y-3); tft.print("10A"); }
             else if(i==1) { tft.setCursor(30, line_y-3); tft.print("1A"); }
             else if(i==2) { tft.setCursor(25, line_y-3); tft.print(".1A"); }
             else if(i==3) { tft.setCursor(20, line_y-3); tft.print(".01A"); }
         }
     }
     
     // [Mod] X-Axis Labels (Odd Harmonics: 1, 3, 5 ... 15)
     int bar_w = graph_w / 8; 
     for(int i=1; i<=8; i++) { 
        int x_pos = graph_x + (i-1)*bar_w + bar_w/2 - 3; 
        tft.setCursor(x_pos, graph_y + graph_h + 5); 
        
        // Display 1, 3, 5 ... 15
        int harmonic_order = (i == 1) ? 1 : (2 * i - 1);
        tft.print(harmonic_order); 
     }
  } else { 
     // Text Mode Layout
     tft.setTextSize(2); tft.setTextColor(COLOR_TEXT_PRIMARY);
     tft.setCursor(30, 55); tft.print("Odd Harmonics (1-15)");
     tft.drawFastHLine(30, 75, 260, COLOR_GRID);
  }
}

void displaySettingsScreenStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("SETTINGS");
  displayNetworkStatus(); 
  drawBackButton();
  
  // 1행 (Mod: Full Width)
  drawButton(20, 50, 280, 40, "CALIBRATION");
  
  // 2행
  drawButton(20, 110, 130, 40, "PROTECTION");
  drawButton(170, 110, 130, 40, "NETWORK"); 
  
  // 3행
  drawButton(20, 170, 130, 40, "TIMER");
  drawButton(170, 170, 130, 40, "ADVANCED");
}

// --- [New] Credit Splash Screen ---
void displayCreditSplashStatic() {
  // 1. Background Fill with KW Burgundy
  tft.fillScreen(COLOR_KW_BURGUNDY);
  
  // 2. Draw University Name
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  
  String title = "KWANGWOON UNIV.";
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  
  // Center alignment
  tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
  tft.print(title);
  
  // 3. Auto-transition logic
  delay(2000); // 2 seconds delay
  currentScreen = SCREEN_SETTINGS_CREDIT;
  screenNeedsRedraw = true;
}

// --- [New] Credit Screen & Sub Screens ---
void displaySettingsCreditStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("CREDIT");
  displayNetworkStatus();
  drawBackButton();
  
  drawButton(20, 50, 280, 40, "MEMBER 1");
  drawButton(20, 110, 280, 40, "MEMBER 2");
  drawButton(20, 170, 280, 40, "MEMBER 3");
}

// --- Helper for Profile Card ---
void drawProfileCard(String title, uint16_t themeColor, String name, String subTitle, String comment) {
  // Title Header
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println(title);
  drawBackButton();

  // Card Dimension
  int cardX = 30;
  int cardY = 50;
  int cardW = 260;
  int cardH = 160;

  // Draw Card Border
  tft.drawRoundRect(cardX, cardY, cardW, cardH, 15, themeColor);
  tft.drawRoundRect(cardX+1, cardY+1, cardW-2, cardH-2, 15, themeColor); // Double Border for effect

  // Calculate Center for Text
  int16_t x1, y1; uint16_t w, h;
  int centerX = cardX + cardW / 2;

  // 1. Name (Large, Theme Color)
  tft.setTextColor(themeColor);
  tft.setTextSize(3); // Large Font
  tft.getTextBounds(name, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(centerX - w / 2, cardY + 35);
  tft.print(name);

  // 2. SubTitle/Role (Medium, Primary Text)
  if (subTitle.length() > 0) {
      tft.setTextColor(COLOR_TEXT_PRIMARY);
      tft.setTextSize(2);
      tft.getTextBounds(subTitle, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(centerX - w / 2, cardY + 70);
      tft.print(subTitle);
  }

  // Divider Line
  tft.drawFastHLine(cardX + 40, cardY + 100, cardW - 80, COLOR_TEXT_SECONDARY);

  // 3. Comment (Small/Italic feel, Secondary Text)
  tft.setTextColor(COLOR_TEXT_SECONDARY);
  tft.setTextSize(1);
  // Simple word wrap or just centering (assuming short quotes)
  tft.getTextBounds(comment, 0, 0, &x1, &y1, &w, &h);
  if (w > cardW - 20) {
     // Very simple wrap logic if needed, but for now assuming it fits or simple split
     tft.setCursor(cardX + 10, cardY + 120);
     tft.print(comment);
  } else {
     tft.setCursor(centerX - w / 2, cardY + 120);
     tft.print(comment);
  }
}

void displayCreditMember1Static() {
  drawProfileCard(
    "MEMBER 1", 
    COLOR_ORANGE, 
    "PARK RAEWON", 
    "(Leader)", 
    "\"Couldnt be better.\""
  );
}

void displayCreditMember2Static() {
  drawProfileCard(
    "MEMBER 2", 
    COLOR_BLUE, 
    "PARK YESEONG", 
    "", 
    "\"Extraordinary claims require \next raordinary evidence.\""
  );
}

void displayCreditMember3Static() {
  drawProfileCard(
    "MEMBER 3", 
    COLOR_GREEN, 
    "JOO HWIJAE", 
    "", 
    "\"Butterflysoup.\""
  );
}

void displaySettingsNetworkStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("NETWORK SETTINGS");
  drawBackButton();
  tft.drawRoundRect(60, 50, 200, 50, 10, COLOR_BUTTON_OUTLINE);
  tft.setCursor(20, 115);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.print("Send to Thingspeak");
}

void displaySettingsCalibMenuStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("CALIBRATION MODE");
  drawBackButton();

  drawButton(20, 60, 280, 40, "MULTIPLIER (MANUAL)");
  drawButton(20, 120, 280, 40, "AUTO CALIBRATION");
}

void displayAutoCalibStatic() {
  tft.setCursor(65, 10); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2);
  tft.println("AUTO CALIBRATION");
  drawBackButton();

  switch(auto_calib_step) {
    case 0:
      tft.setCursor(20, 60); tft.print("Disconnect ALL power"); tft.setCursor(20, 80); tft.print("and loads.");
      drawButton(80, 120, 160, 40, "START");
      break;
    case 1:
      tft.setCursor(20, 80); tft.print("Measuring Offsets...");
      break;
    case 2:
      tft.setCursor(20, 60); tft.print("Offsets Measured."); tft.setCursor(20, 80); tft.print("Connect known Load.");
      drawButton(80, 130, 160, 40, "NEXT");
      break;
    case 3:
      tft.setCursor(30, 50); tft.print("True V:"); tft.setCursor(30, 85); tft.print("True I:"); tft.setCursor(30, 120); tft.print("Step:");
      drawButton(20, 180, 60, 40, "UP"); drawButton(90, 180, 60, 40, "DOWN"); drawButton(180, 180, 60, 40, "-"); drawButton(250, 180, 60, 40, "+");
      tft.fillRoundRect(260, 5, 55, 30, 8, COLOR_GREEN); tft.setCursor(270, 12); tft.setTextColor(COLOR_BACKGROUND); tft.print("SAVE");
      break;
    case 4:
      tft.setCursor(20, 80); tft.print("Calculated."); tft.setCursor(20, 100); tft.print("Disconnect to Verify.");
      drawButton(80, 130, 160, 40, "VERIFY");
      break;
    case 5:
      tft.setCursor(20, 50); tft.print("Verification:"); tft.setCursor(20, 80); tft.print("V:"); tft.setCursor(20, 110); tft.print("I:");
      drawButton(20, 180, 180, 40, "APPLY & SAVE"); drawButton(210, 180, 100, 40, "RESTART");
      break;
  }
}

void displaySettingsCalibManualStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("MANUAL CALIBRATION");
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

void displayRelayControlStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("RELAY CONTROL");
  drawBackButton();
  prev_r1_state = !relay1_state;
  prev_r2_state = !relay2_state;
}

void displaySettingsThemeStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("THEME SETTINGS");
  drawBackButton();
  
  drawButton(20, 70, 280, 40, "DARK THEME");
  drawButton(20, 130, 280, 40, "LIGHT THEME");
  
  tft.setTextColor(COLOR_RED);
  tft.setTextSize(2);
  if (!isDarkMode) { tft.setCursor(5, 80); } else { tft.setCursor(5, 140); }
  tft.print(">");
}

void displaySettingsResetStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RESET SETTINGS");
  drawBackButton();
  tft.setCursor(20, 60); tft.print("Reset all values to"); tft.setCursor(20, 80); tft.print("factory defaults?");
  drawButton(20, 100, 130, 40, "RESET");
  drawButton(170, 100, 130, 40, "CANCEL");
}

void displayConfirmSaveStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("SAVE CHANGES?");
  drawBackButton();
  tft.setCursor(20, 70); tft.print("Save the new settings?");
  
  drawButton(20, 100, 130, 40, "DISCARD"); 
  drawButton(170, 100, 130, 40, "SAVE");    
}

void displaySettingsAdvancedStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("ADVANCED SETTINGS");
  drawBackButton();
  
  // 1행
  drawButton(20, 50, 130, 40, "THEME");
  drawButton(170, 50, 130, 40, "PRESETS");
  
  // 2행
  drawButton(20, 110, 130, 40, "RESET");
  drawButton(170, 110, 130, 40, "CREDIT");
}

void displayPresetScreenStatic() {
  tft.setCursor(65, 10); tft.setTextColor(COLOR_TEXT_PRIMARY); tft.setTextSize(2); tft.println("PRESETS (EEPROM)");
  drawBackButton();
  
  String modeStr = isPresetSaveMode ? "MODE: SAVE" : "MODE: LOAD";
  drawButton(80, 50, 160, 40, modeStr);
  
  drawButton(20, 110, 130, 40, "SLOT 1");
  drawButton(170, 110, 130, 40, "SLOT 2");
  drawButton(20, 170, 130, 40, "SLOT 3");
  drawButton(170, 170, 130, 40, "SLOT 4");
  
  tft.setTextColor(COLOR_TEXT_SECONDARY); tft.setTextSize(1);
  tft.setCursor(20, 220); tft.print("Tap Slot to Load/Save");
}

void displaySettingsTimerStatic() {
  tft.setCursor(65, 10);
  tft.setTextColor(COLOR_TEXT_PRIMARY);
  tft.setTextSize(2);
  tft.println("RELAY TIMER SETTINGS");
  drawBackButton();
  
  drawButton(20, 150, 60, 40, "-");    
  drawButton(160, 150, 60, 40, "+");   
  
  prev_temp_timer_setting_seconds = 0xFFFFFFFF;
  prev_timer_step_index = -1;
  prev_timer_target_relay = -1;
}

void displayWarningScreenStatic() {
  uint16_t bgColor;
  if (warningMessage.indexOf("TIMER") != -1) {
    bgColor = ILI9341_BLUE; 
  } else {
    bgColor = ILI9341_RED;  
  }

  tft.fillScreen(bgColor);
  tft.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, ILI9341_WHITE);
  tft.drawRect(12, 12, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 24, ILI9341_WHITE);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3); 
  int16_t x1, y1;
  uint16_t w1, h1;
  String title = "SYSTEM TRIP";
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 40); 
  tft.print(title); 
  
  tft.setTextSize(3); 
  tft.getTextBounds(warningMessage, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 90); 
  tft.print(warningMessage);
  
  tft.setTextSize(2);
  String relayMsg = "Relays Cut-off";
  tft.getTextBounds(relayMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 140);
  tft.print(relayMsg);
  
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2); 
  String resetMsg = "Tap to Check Status >";
  tft.getTextBounds(resetMsg, 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor((SCREEN_WIDTH - w1) / 2, 190); 
  tft.print(resetMsg); 
}

void drawWaveformGridAndLabels() {
  tft.setCursor(65, 10); 
  tft.setTextColor(COLOR_TEXT_PRIMARY); 
  tft.setTextSize(2);
  tft.println("WAVEFORM (60Hz)");
  drawBackButton(); 
  
  // [Mod] 그리드 그리기: PLOT_X_START(0) ~ PLOT_X_END(320) 전체 영역 사용
  tft.drawRect(PLOT_X_START, PLOT_Y_START, PLOT_WIDTH, (PLOT_Y_END - PLOT_Y_START), COLOR_GRID); 
  tft.drawFastHLine(PLOT_X_START, PLOT_Y_CENTER, PLOT_WIDTH, COLOR_GRID); 
  tft.drawFastVLine(PLOT_X_START, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 
  // 오른쪽 끝 라인 (화면 밖으로 나가지 않도록 -1)
  tft.drawFastVLine(PLOT_X_END - 1, PLOT_Y_START, (PLOT_Y_END - PLOT_Y_START), COLOR_TEXT_SECONDARY); 

  drawButton(10, 195, 100, 35, WAVEFORM_TYPE_LABELS[waveformPlotType]);
  drawButton(115, 195, 100, 35, WAVEFORM_MODE_LABELS[waveformTriggerMode]); 
  drawButton(220, 195, 90, 35, WAVEFORM_PERIOD_LABELS[waveformPeriodIndex]);

  // [Mod] Type 2 (Current Mode) Legend Colors Matched
  if (waveformPlotType == 2) {
     tft.setTextSize(1);
     tft.setCursor(200, 35); tft.setTextColor(COLOR_BLUE); tft.print("I ");
     tft.setCursor(220, 35); tft.setTextColor(COLOR_ORANGE); tft.print("I1 ");
     tft.setCursor(245, 35); tft.setTextColor(COLOR_RED); tft.print("I2");
  }
}