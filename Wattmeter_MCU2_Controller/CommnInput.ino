/*
 * ==============================================================================
 * 파일명: 4. Comm_Input.ino
 * 버전: v106 (Timer Relay Cycle Logic)
 * 설명: 
 * - [Mod] Timer Target Select: Cycle 1 -> 2 -> Both(3) -> 1 on single button touch
 * - 터치 영역 확대 (+/- 10px padding 적용)
 * - 고조파 화면 버튼 토글 로직 유지
 * ==============================================================================
 */

// --- CSV 파싱 헬퍼 ---
void parseCSV(String data, float* arr, int maxLen) {
  int idx = 0;
  int start = 0;
  int end = data.indexOf(',');
  
  while (end != -1 && idx < maxLen) {
    arr[idx++] = data.substring(start, end).toFloat();
    start = end + 1;
    end = data.indexOf(',', start);
  }
  if (idx < maxLen) {
    arr[idx] = data.substring(start).toFloat();
  }
}

// --- JSON 명령 전송 헬퍼 (Serial1 사용) ---
void sendJsonCommand(String jsonString) {
  Serial1.print(jsonString); 
  Serial1.println(); 
}

// --- 시리얼 입력 확인 (Serial1 사용) ---
void checkSerialInput() {
  if (Serial1.available() == 0) return;
  
  String line = Serial1.readStringUntil('\n');
  if (line.length() == 0) return;

  DeserializationError error = deserializeJson(rxJsonDoc, line);
  if (error) {
    return;
  }

  const char* type = rxJsonDoc["TYPE"];
  if (type == NULL || strcmp(type, "DATA") != 0) {
    return; 
  }
  
  // RMS 데이터 업데이트
  V_rms = rxJsonDoc["V"] | V_rms;
  I_rms = rxJsonDoc["I"] | I_rms;
  I_rms_load1 = rxJsonDoc["I1"] | I_rms_load1;
  I_rms_load2 = rxJsonDoc["I2"] | I_rms_load2;
  P_real = rxJsonDoc["P"] | P_real;
  Q_reactive = rxJsonDoc["Q"] | Q_reactive;
  S_apparent = rxJsonDoc["S"] | S_apparent;
  PF = rxJsonDoc["PF"] | PF;
  
  phase_main_deg = rxJsonDoc["PH_M"] | phase_main_deg;
  phase_load1_deg = rxJsonDoc["PH_1"] | phase_load1_deg;
  phase_load2_deg = rxJsonDoc["PH_2"] | phase_load2_deg;
  lead_lag_status = rxJsonDoc["LL"] | "---";
  
  thd_v_value = rxJsonDoc["THD_V"] | thd_v_value;
  thd_i_value = rxJsonDoc["THD_I"] | thd_i_value;
  
  String hv = rxJsonDoc["H_V_STR"];
  String hi = rxJsonDoc["H_I_STR"];
  if (hv.length() > 0) {
    float temp_h[8];
    parseCSV(hv, temp_h, 8);
    for(int k=0; k<7; k++) v_harmonics[k+1] = temp_h[k]; 
  }
  if (hi.length() > 0) {
    float temp_h[8];
    parseCSV(hi, temp_h, 8);
    for(int k=0; k<7; k++) i_harmonics[k+1] = temp_h[k];
  }

  V_MULTIPLIER = rxJsonDoc["V_MULT"] | V_MULTIPLIER;
  I_MULTIPLIER = rxJsonDoc["I_MULT"] | I_MULTIPLIER;
  VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"] | VOLTAGE_THRESHOLD;
  setting_step_index = rxJsonDoc["STEP_IDX"] | setting_step_index;

  relay1_state = rxJsonDoc["R1"]; 
  relay2_state = rxJsonDoc["R2"];
  
  // MCU1에서 타이머 상태 수신하지 않음 (MCU2가 Master)
  // is_timer_active = rxJsonDoc["T_ACTIVE"];
  // timer_seconds_left = rxJsonDoc["T_LEFT_S"] | 0;
  // timer_setting_seconds = rxJsonDoc["T_SEC"] | 0;
  
  // [Mod] Strict Warning Trigger Logic
  // 오직 JSON "WARN" 필드가 true일 때만 경고 활성화
  bool newWarning = rxJsonDoc["WARN"];
  if (newWarning && !warningActive) {
    warningMessage = rxJsonDoc["MSG"] | "WARNING!";
    warningActive = true;
    screenNeedsRedraw = true; 
  } 
  // 해제는 MCU2 화면 터치로만 수행하므로, else if (!newWarning) 블록 불필요
}

// --- 설정 값 변경 헬퍼 (Calib Manual) ---
void adjustCalibValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; 
  
  txJsonDoc.clear();
  txJsonDoc["CMD"] = "SET_CALIB";

  float targetValue;

  switch (calib_selection) {
    case 0: 
      targetValue = V_MULTIPLIER + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["V_MULT"] = targetValue;
      V_MULTIPLIER = targetValue;
      break;
    case 1: 
      targetValue = I_MULTIPLIER + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["I_MULT"] = targetValue;
      I_MULTIPLIER = targetValue;
      break;
    case 2: 
      txJsonDoc.clear(); 
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index; 
        txJsonDoc["CMD"] = "SET_STEP";
        txJsonDoc["IDX"] = new_index;
      } else {
        return; 
      }
      break;
  }
  
  serializeJson(txJsonDoc, Serial1); 
  Serial1.println();
}

// --- [New] Auto Calib 값 조정 ---
void adjustAutoCalibValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  switch (calib_selection) {
    case 0: temp_true_v += (increase ? step_to_apply : -step_to_apply); if (temp_true_v < 0) temp_true_v = 0; break;
    case 1: temp_true_i += (increase ? step_to_apply : -step_to_apply); if (temp_true_i < 0) temp_true_i = 0; break;
    case 2: 
       int new_index = setting_step_index + (increase ? 1 : -1);
       if (new_index >= 0 && new_index < NUM_SETTING_STEPS) setting_step_index = new_index;
       break;
  }
}

// --- 설정 값 변경 헬퍼 (Protect) ---
void adjustProtectValue(bool increase) {
  float step_to_apply = setting_steps[setting_step_index];
  settingsChanged = true; 

  txJsonDoc.clear();
  float targetValue;

  switch (protect_selection) {
    case 0: 
      targetValue = VOLTAGE_THRESHOLD + (increase ? step_to_apply : -step_to_apply);
      if (targetValue < 0) targetValue = 0;
      txJsonDoc["CMD"] = "SET_PROTECT";
      txJsonDoc["V_THR"] = targetValue;
      VOLTAGE_THRESHOLD = targetValue;
      break;
    case 1: 
      int new_index = setting_step_index + (increase ? 1 : -1);
      if (new_index >= 0 && new_index < NUM_SETTING_STEPS) {
        setting_step_index = new_index; 
        txJsonDoc["CMD"] = "SET_STEP";
        txJsonDoc["IDX"] = new_index;
      } else {
        return; 
      }
      break;
  }

  serializeJson(txJsonDoc, Serial1); 
  Serial1.println();
}

// --- EEPROM 프리셋 저장/로드 ---
void savePreset(int slot) {
  int addr = EEPROM_BASE_ADDR + (slot * PRESET_SIZE);
  Preset p = {V_MULTIPLIER, I_MULTIPLIER, VOLTAGE_THRESHOLD, true};
  EEPROM.put(addr, p);
}

void loadPreset(int slot) {
  int addr = EEPROM_BASE_ADDR + (slot * PRESET_SIZE);
  Preset p;
  EEPROM.get(addr, p);
  if (p.valid) {
    V_MULTIPLIER = p.v_mult;
    I_MULTIPLIER = p.i_mult;
    VOLTAGE_THRESHOLD = p.v_thresh;
    
    txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_CALIB"; txJsonDoc["V_MULT"] = V_MULTIPLIER; txJsonDoc["I_MULT"] = I_MULTIPLIER; serializeJson(txJsonDoc, Serial1); Serial1.println();
    txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_PROTECT"; txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; serializeJson(txJsonDoc, Serial1); Serial1.println();
  }
}

// --- 터치 입력 확인 ---
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  static bool prev_touched_flag = false;

  if (!touched) {
     prev_touched_flag = false;
     return; // 터치가 없으면 리턴
  }
  
  // 터치 좌표 읽기
  SPI.beginTransaction(spiSettingsTouch);
  TS_Point p = ts.getPoint();
  SPI.endTransaction();

  p.x = map(p.x, TS_RAW_X1, TS_RAW_X2, SCREEN_WIDTH, 0);
  p.y = map(p.y, TS_RAW_Y1, TS_RAW_Y2, SCREEN_HEIGHT, 0);
  
  // [Mod] Relay Control 화면 One-shot 로직
  bool is_new_touch = !prev_touched_flag;
  prev_touched_flag = true; // 플래그 갱신
  
  // [Mod] Warning Screen 해제 로직 변경 (Safe Release)
  if (currentScreen == SCREEN_WARNING) { 
    // 1. 경고 상태만 해제 (릴레이 자동 ON 금지)
    warningActive = false;
    
    // 2. 사용자가 직접 확인하도록 Relay Control 화면으로 이동
    currentScreen = SCREEN_RELAY_CONTROL;
    screenNeedsRedraw = true; 
    delay(200); // Debounce
    return; 
  }

  if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
    if (p.x >= 0 && p.x <= 60 && p.y >= 0 && p.y <= 40) { // Back Button
      if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
        isWaveformFrozen = false; 
      }
      if (currentScreen == SCREEN_SETTINGS_CALIB_AUTO) {
         auto_calib_step = 0; 
      }

      isMainPowerFrozen = false;
      isPhaseFrozen = false;
      isHarmonicsFrozen = false;
      
      // Back Navigation Logic
      if (currentScreen == SCREEN_SETTINGS_CALIB_MENU || currentScreen == SCREEN_SETTINGS_CALIB_MANUAL || currentScreen == SCREEN_SETTINGS_CALIB_AUTO) {
        if (currentScreen == SCREEN_SETTINGS_CALIB_MANUAL && settingsChanged) { 
           previousScreen = currentScreen; currentScreen = SCREEN_CONFIRM_SAVE;
        } else if (currentScreen == SCREEN_SETTINGS_CALIB_AUTO && settingsChanged) {
           previousScreen = currentScreen; currentScreen = SCREEN_CONFIRM_SAVE;
        } else if (currentScreen == SCREEN_SETTINGS_CALIB_MENU) {
           currentScreen = SCREEN_SETTINGS; 
        } else {
           currentScreen = SCREEN_SETTINGS_CALIB_MENU; 
        }
      } 
      else if (currentScreen == SCREEN_SETTINGS_PROTECT) {
           if (settingsChanged) { previousScreen = currentScreen; currentScreen = SCREEN_CONFIRM_SAVE; }
           else currentScreen = SCREEN_SETTINGS;
      }
      else if (currentScreen == SCREEN_SETTINGS_THEME || 
               currentScreen == SCREEN_SETTINGS_RESET || 
               currentScreen == SCREEN_SETTINGS_TIMER || 
               currentScreen == SCREEN_SETTINGS_PRESETS || 
               currentScreen == SCREEN_SETTINGS_NETWORK ||
               currentScreen == SCREEN_CONFIRM_SAVE) {
        if (currentScreen == SCREEN_CONFIRM_SAVE) {
           currentScreen = SCREEN_SETTINGS; 
        } else if (currentScreen == SCREEN_SETTINGS_NETWORK) {
           currentScreen = SCREEN_SETTINGS;
        } else if (currentScreen == SCREEN_SETTINGS_PRESETS) {
           currentScreen = SCREEN_SETTINGS_ADVANCED;
        } else if (currentScreen == SCREEN_SETTINGS_THEME || currentScreen == SCREEN_SETTINGS_RESET) {
           currentScreen = SCREEN_SETTINGS_ADVANCED;
        } else {
           // Timer is now directly under Settings
           if (currentScreen == SCREEN_SETTINGS_TIMER) currentScreen = SCREEN_SETTINGS;
           else currentScreen = SCREEN_SETTINGS_ADVANCED;
        }
      }
      else if (currentScreen == SCREEN_SETTINGS_ADVANCED) {
        currentScreen = SCREEN_SETTINGS;
      }
      else if (currentScreen == SCREEN_SETTINGS || currentScreen == SCREEN_RELAY_CONTROL) {
        currentScreen = SCREEN_HOME;
      }
      else {
        currentScreen = SCREEN_HOME;
      }
      
      screenNeedsRedraw = true;
      delay(100); 
      return;
    }
  }

  switch (currentScreen) {
    case SCREEN_HOME:
      // 터치 영역 넉넉하게 (+/- 5~10px)
      if (p.x >= 15 && p.x <= 155 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_MAIN_POWER; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_PHASE_DIFFERENCE; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_COMBINED_WAVEFORM; screenNeedsRedraw = true; sendJsonCommand("{\"CMD\":\"REQ_WAVEFORM\"}"); }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_HARMONICS; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_RELAY_CONTROL; screenNeedsRedraw = true; }
      break;

    case SCREEN_MAIN_POWER: 
       if (p.x >= 15 && p.x <= 105 && p.y >= 190 && p.y <= 235) {
          isMainPowerFrozen = !isMainPowerFrozen;
          String text = isMainPowerFrozen ? "RUN" : "HOLD";
          drawButton(20, 195, 80, 35, text); delay(200);
       }
       break;

    case SCREEN_PHASE_DIFFERENCE:
       if (p.x >= 15 && p.x <= 85 && p.y >= 200 && p.y <= 235) {
          isPhaseFrozen = !isPhaseFrozen;
          String text = isPhaseFrozen ? "RUN" : "HOLD";
          drawButton(20, 205, 60, 25, text); delay(200);
       }
       break;

    case SCREEN_HARMONICS: 
      if (p.x >= 5 && p.x <= 105 && p.y >= 195 && p.y <= 240) { 
          harmonicsSource = !harmonicsSource; 
          harmonicsSrcLabel = (harmonicsSource == 0) ? "Src: V" : "Src: I";
          screenNeedsRedraw = false; 
          delay(200);
      }
      else if (p.x >= 110 && p.x <= 215 && p.y >= 195 && p.y <= 240) { 
          isHarmonicsFrozen = !isHarmonicsFrozen; 
          delay(200);
      }
      else if (p.x >= 220 && p.x <= 315 && p.y >= 195 && p.y <= 240) { 
          harmonicsViewMode = !harmonicsViewMode; 
          screenNeedsRedraw = true; 
          delay(200);
      }
      break;

    case SCREEN_SETTINGS:
      // [Mod] Settings 터치 좌표 재배치 (Grid)
      // 1. Top: CALIB (Y: 45~95, Wide)
      if (p.x >= 15 && p.x <= 305 && p.y >= 45 && p.y <= 95) { 
        temp_V_MULTIPLIER = V_MULTIPLIER; temp_I_MULTIPLIER = I_MULTIPLIER; temp_setting_step_index = setting_step_index; settingsChanged = false; 
        currentScreen = SCREEN_SETTINGS_CALIB_MENU; screenNeedsRedraw = true; 
      } 
      // 2. Row 1 Left: PROTECT (Y: 105~155)
      else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { 
        temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD; temp_setting_step_index = setting_step_index; settingsChanged = false; currentScreen = SCREEN_SETTINGS_PROTECT; screenNeedsRedraw = true;
      } 
      // 3. Row 1 Right: NETWORK (Y: 105~155)
      else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { 
        currentScreen = SCREEN_SETTINGS_NETWORK; screenNeedsRedraw = true;
      } 
      // 4. Row 2 Left: TIMER (Y: 165~215) [Advanced에서 이동됨]
      else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) { 
        temp_timer_setting_seconds = timer_setting_seconds; currentScreen = SCREEN_SETTINGS_TIMER; screenNeedsRedraw = true; 
      }
      // 5. Row 2 Right: ADVANCED (Y: 165~215)
      else if (p.x >= 165 && p.x <= 305 && p.y >= 165 && p.y <= 215) { 
        currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true;
      }
      break;

    case SCREEN_SETTINGS_CALIB_MENU: 
      if (p.x >= 15 && p.x <= 305 && p.y >= 55 && p.y <= 105) { currentScreen = SCREEN_SETTINGS_CALIB_MANUAL; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 305 && p.y >= 115 && p.y <= 165) { 
         currentScreen = SCREEN_SETTINGS_CALIB_AUTO; 
         auto_calib_step = 0; 
         temp_true_v = 220.0; temp_true_i = 1.0;
         screenNeedsRedraw = true; 
      }
      break;

    case SCREEN_SETTINGS_NETWORK: 
      if (p.x >= 55 && p.x <= 265 && p.y >= 45 && p.y <= 105) {
        isWifiEnabled = !isWifiEnabled;
        if (isWifiEnabled) connectWiFi(); else { sendAT("AT+CWQAP\r\n", 1000, true); isWifiConnected = false; }
        delay(200);
      } else if (p.y >= 130 && p.y <= 180) {
         if (p.x >= 15 && p.x <= 85) send_V = !send_V; else if (p.x >= 85 && p.x <= 155) send_I = !send_I; else if (p.x >= 155 && p.x <= 225) send_P = !send_P;
      }
      break;

    case SCREEN_SETTINGS_ADVANCED:
      // [Mod] Advanced 터치 좌표 재배치 (세로 3개)
      // Top: THEME (Y: 45~95)
      if (p.x >= 15 && p.x <= 305 && p.y >= 45 && p.y <= 95) { 
          currentScreen = SCREEN_SETTINGS_THEME; screenNeedsRedraw = true; 
      } 
      // Mid: PRESETS (Y: 105~155)
      else if (p.x >= 15 && p.x <= 305 && p.y >= 105 && p.y <= 155) { 
          currentScreen = SCREEN_SETTINGS_PRESETS; screenNeedsRedraw = true; 
      } 
      // Bot: RESET (Y: 165~215)
      else if (p.x >= 15 && p.x <= 305 && p.y >= 165 && p.y <= 215) { 
          currentScreen = SCREEN_SETTINGS_RESET; screenNeedsRedraw = true; 
      }
      break;
      
    case SCREEN_SETTINGS_PRESETS: 
      if (p.x >= 75 && p.x <= 245 && p.y >= 45 && p.y <= 95) { isPresetSaveMode = !isPresetSaveMode; screenNeedsRedraw = true; } 
      else {
         int slot = -1;
         if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) slot = 0;
         else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) slot = 1;
         else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) slot = 2;
         else if (p.x >= 165 && p.x <= 305 && p.y >= 165 && p.y <= 215) slot = 3;
         
         if (slot != -1) {
            if (isPresetSaveMode) { savePreset(slot); drawButton(60, 220, 200, 20, "Saved to Slot " + String(slot+1)); }
            else { loadPreset(slot); drawButton(60, 220, 200, 20, "Loaded Slot " + String(slot+1)); }
            delay(500);
         }
      }
      break;

    case SCREEN_SETTINGS_TIMER:
      // [Mod] 릴레이 선택 (Target 설정) - Cycle Logic
      // 버튼 위치: 60, 45, 200, 40 (Dynamic View)
      // 터치 영역 통합: x:60~260, y:40~90
      if (p.x >= 60 && p.x <= 260 && p.y >= 40 && p.y <= 90) { 
         timer_target_relay++;
         if (timer_target_relay > 3) timer_target_relay = 1; // 1 -> 2 -> 3 -> 1
         delay(100); 
      }
      
      // 시간 조절
      else if (p.x >= 15 && p.x <= 85 && p.y >= 135 && p.y <= 185) { // -
        if (temp_timer_setting_seconds >= TIMER_STEP_VALUES[timer_step_index]) temp_timer_setting_seconds -= TIMER_STEP_VALUES[timer_step_index]; else temp_timer_setting_seconds = 0;
      } else if (p.x >= 85 && p.x <= 155 && p.y >= 135 && p.y <= 185) { // STEP
        timer_step_index = (timer_step_index + 1) % 6; 
      } else if (p.x >= 155 && p.x <= 225 && p.y >= 135 && p.y <= 185) { // +
        if (temp_timer_setting_seconds + TIMER_STEP_VALUES[timer_step_index] <= 359999) temp_timer_setting_seconds += TIMER_STEP_VALUES[timer_step_index];
      } 
      // [Mod] START/STOP 로직 수정
      // MCU1으로 SET_TIMER 명령을 보내지 않고, MCU2 내부 변수만 설정
      else if (p.x >= 225 && p.x <= 315 && p.y >= 135 && p.y <= 185) {
        if (is_timer_active) {
           // STOP
           is_timer_active = false;
        } else {
           // START
           if (temp_timer_setting_seconds > 0) {
              timer_seconds_left = temp_timer_setting_seconds;
              is_timer_active = true;
              last_timer_tick = millis();
              timer_setting_seconds = temp_timer_setting_seconds; // Save setting
           }
        }
        delay(200);
      }
      break;

    case SCREEN_SETTINGS_CALIB_MANUAL:
      if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS;
      else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS;
      else if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) adjustCalibValue(false); 
      else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) adjustCalibValue(true); 
      break;
      
    case SCREEN_SETTINGS_CALIB_AUTO:
      if (auto_calib_step == 0) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 115 && p.y <= 165) { auto_calib_step = 1; screenNeedsRedraw = true; }
      } else if (auto_calib_step == 2) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 125 && p.y <= 175) { 
           auto_calib_step = 3; calib_selection = 0; setting_step_index = 2; screenNeedsRedraw = true; 
         }
      } else if (auto_calib_step == 3) {
         if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) calib_selection = (calib_selection - 1 + NUM_AUTOCALIB_INPUTS) % NUM_AUTOCALIB_INPUTS;
         else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) calib_selection = (calib_selection + 1) % NUM_AUTOCALIB_INPUTS;
         else if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) adjustAutoCalibValue(false); 
         else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) adjustAutoCalibValue(true); 
         else if (p.x >= 255 && p.x <= 320 && p.y >= 0 && p.y <= 40) { calculateNewGains(temp_true_v, temp_true_i); auto_calib_step = 4; screenNeedsRedraw = true; }
      } else if (auto_calib_step == 4) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 125 && p.y <= 175) { auto_calib_step = 5; screenNeedsRedraw = true; }
      } else if (auto_calib_step == 5) {
         if (p.x >= 15 && p.x <= 205 && p.y >= 175 && p.y <= 225) { previousScreen = currentScreen; currentScreen = SCREEN_CONFIRM_SAVE; } 
         else if (p.x >= 205 && p.x <= 315 && p.y >= 175 && p.y <= 225) { auto_calib_step = 0; screenNeedsRedraw = true; }
      }
      break;

    case SCREEN_SETTINGS_PROTECT:
      if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS;
      else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS;
      else if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) adjustProtectValue(false); 
      else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) adjustProtectValue(true); 
      break;

    case SCREEN_RELAY_CONTROL:
      // [Mod] One-shot Logic Applied (is_new_touch check above)
      if (is_new_touch) {
        if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":2}");
        else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":2}");
      }
      break;

    case SCREEN_SETTINGS_RESET:
      if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { sendJsonCommand("{\"CMD\":\"RESET_SETTINGS\"}"); currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
      break;

    case SCREEN_CONFIRM_SAVE:
      // [Mod] Save / Discard 텍스트 스왑에 따른 로직 매핑 확인
      // Display: Left=SAVE, Right=DISCARD
      if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { // Left Button (Now SAVE)
        if (previousScreen == SCREEN_SETTINGS_CALIB_MANUAL) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; } 
        else if (previousScreen == SCREEN_SETTINGS_CALIB_AUTO) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; }
        else if (previousScreen == SCREEN_SETTINGS_PROTECT) { VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD; }
        
        setting_step_index = temp_setting_step_index;
        
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_CALIB"; txJsonDoc["V_MULT"] = V_MULTIPLIER; txJsonDoc["I_MULT"] = I_MULTIPLIER; serializeJson(txJsonDoc, Serial1); Serial1.println();
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_PROTECT"; txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; serializeJson(txJsonDoc, Serial1); Serial1.println();
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_STEP"; txJsonDoc["IDX"] = setting_step_index; serializeJson(txJsonDoc, Serial1); Serial1.println();
        
        settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
      }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { // Right Button (Now DISCARD)
        settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
      }
      break;

    case SCREEN_SETTINGS_THEME:
      // [Mod] Theme 텍스트 스왑에 따른 로직 매핑 확인
      // Display: Top=LIGHT, Bot=DARK
      if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) { // Top Button (Light)
         isDarkMode = false; setTheme(); screenNeedsRedraw = true; 
      }
      else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) { // Bot Button (Dark)
         isDarkMode = true; setTheme(); screenNeedsRedraw = true; 
      }
      break;

    case SCREEN_COMBINED_WAVEFORM:
      if (p.x >= 5 && p.x <= 110 && p.y >= 190 && p.y <= 235) {
        waveformPlotType = (waveformPlotType + 1) % 3;
        drawButton(10, 195, 100, 35, WAVEFORM_TYPE_LABELS[waveformPlotType]);
        
        screenNeedsRedraw = true; 
      }
      else if (p.x >= 110 && p.x <= 215 && p.y >= 190 && p.y <= 235) {
        if (waveformTriggerMode == 2 && isWaveformFrozen) isWaveformFrozen = false; 
        else { waveformTriggerMode = (waveformTriggerMode + 1) % 3; isWaveformFrozen = false; }
        drawButton(115, 195, 100, 35, String(WAVEFORM_MODE_LABELS[waveformTriggerMode]));
      }
      else if (p.x >= 215 && p.x <= 315 && p.y >= 190 && p.y <= 235) {
        waveformPeriodIndex = (waveformPeriodIndex + 1) % 3;
        drawButton(220, 195, 90, 35, WAVEFORM_PERIOD_LABELS[waveformPeriodIndex]);
      }
      break;
  }
  delay(100); 
}