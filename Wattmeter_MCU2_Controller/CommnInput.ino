/*
 * ==============================================================================
 * 파일명: 4. Comm_Input.ino
 * 버전: v218_TouchMod (Phase One-Shot Added) + Timer Logic Improved
 * 설명: 
 * - [System] 공통 터치 딜레이 상수(TOUCH_REPEAT_DELAY) 정의 및 적용
 * - [Mod] One-Shot(한 번만 인식) 적용: Main, Phase(Run/Hold), Harmonics(All), Waveform(Type, Period)
 * - [Mod] Continuous(연속 인식) 적용: Timer/Calib/Protect 화면의 (+, -) 버튼
 * - [Fix] Waveform Trigger Mode 버튼은 연속 입력 가능하도록 예외 처리
 * - [Legacy] CALIB_AUTO 등은 기존 로직 유지
 * - [Mod] Timer 5초 이상 누름 리셋 기능 제거
 * - [Fix] Timer 설정 값 변경 시 전체 화면 갱신(screenNeedsRedraw) 제거 -> 부분 갱신 유도
 * - [Mod] BASE 교정 값 수신 로직 제거 (하드코딩으로 변경됨)
 * - [Update] EEPROM Preset Save/Load includes ADC Midpoints (Offsets)
 * - [Update] Reset Settings includes ADC Midpoints (8192.0)
 * ==============================================================================
 */

// --- 공통 딜레이 상수 정의 ---
const int TOUCH_REPEAT_DELAY = 150;

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
    Serial.print("[RX ERROR] JSON Parsing Failed: ");
    Serial.println(error.c_str());
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
  
  // [Mod] Receive Base Calibration Values Removed (Now hardcoded in Controller)
  
  String hv = rxJsonDoc["H_V_STR"];
  String hi = rxJsonDoc["H_I_STR"];
  if (hv.length() > 0) {
    float temp_h[8];
    parseCSV(hv, temp_h, 8);
    for(int k=0; k<8; k++) v_harmonics[k+1] = temp_h[k]; 
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
  
  // Warning Logic
  bool newWarning = rxJsonDoc["WARN"];
  static bool prevWarningLog = false;
  if (newWarning != prevWarningLog) {
      Serial.print("[DEBUG] WARN Signal Changed: ");
      Serial.println(newWarning ? "TRUE (Active)" : "FALSE (Clear)");
      prevWarningLog = newWarning;
  }

  if (newWarning) {
     if (!warningActive) {
        String t_type = rxJsonDoc["T_TYPE"] | "";
        String t_rly = rxJsonDoc["T_RLY"] | "";
        if (t_rly == "R1") {
               relay1_state = true;
           } 
           else if (t_rly == "R2") {
               relay2_state = true;
           } 
           else if (t_rly == "ALL") {
               relay1_state = true;
               relay2_state = true;
           } 
        if (t_type.length() > 0 && t_rly.length() > 0) {
           warningMessage = t_type + " (" + t_rly + ")";
        } else {
           warningMessage = rxJsonDoc["MSG"] | "WARNING!";
        }

        warningActive = true;

        // [Mod] Reset All Settings except Offsets on Warning
        V_MULTIPLIER = DEFAULT_V_MULTIPLIER;
        I_MULTIPLIER = DEFAULT_I_MULTIPLIER;
        VOLTAGE_THRESHOLD = DEFAULT_VOLTAGE_THRESHOLD;
        setting_step_index = DEFAULT_SETTING_STEP_INDEX;
        
        // [User Request] ADC Midpoint Reset Code REMOVED Here

        txJsonDoc.clear();
        txJsonDoc["CMD"] = "RESET_SETTINGS";
        serializeJson(txJsonDoc, Serial1);
        Serial1.println();

        screenNeedsRedraw = true; 
     }
  } 
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

// --- Auto Calib 값 조정 ---
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
  // [Mod] Save Midpoint (Offset) values to Preset
  // Note: 'Preset' struct in MCU2_Controller.ino must have fields: v_mid, i_mid, i1_mid, i2_mid
  Preset p;
  p.v_mult = V_MULTIPLIER;
  p.i_mult = I_MULTIPLIER;
  p.v_thresh = VOLTAGE_THRESHOLD;
  p.v_mid = V_ADC_MIDPOINT_CALIB;
  p.i_mid = I_ADC_MIDPOINT_CALIB;
  p.i1_mid = I1_ADC_MIDPOINT_CALIB;
  p.i2_mid = I2_ADC_MIDPOINT_CALIB;
  p.valid = true;
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
    
    // [Mod] Load Midpoint (Offset) values to Global Variables
    V_ADC_MIDPOINT_CALIB = p.v_mid;
    I_ADC_MIDPOINT_CALIB = p.i_mid;
    I1_ADC_MIDPOINT_CALIB = p.i1_mid;
    I2_ADC_MIDPOINT_CALIB = p.i2_mid;
    
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

  // [Legacy] Long Press Logic (CALIB_AUTO를 위해 유지)
  static unsigned long touchDownTime = 0;       
  static bool isAdjustingValue = false;        
  static bool repeatDirection = false;         
  static unsigned long lastRepeatTime = 0;      
  const unsigned long INITIAL_DELAY = 1000;    
  const unsigned long REPEAT_INTERVAL = 100;   

  if (!touched) {
     touchDownTime = 0;
     isAdjustingValue = false;
     prev_touched_flag = false;
     return; 
  }
  
  // 터치 좌표 읽기
  SPI.beginTransaction(spiSettingsTouch);
  TS_Point p = ts.getPoint();
  SPI.endTransaction();

  p.x = map(p.x, TS_RAW_X1, TS_RAW_X2, SCREEN_WIDTH, 0);
  p.y = map(p.y, TS_RAW_Y1, TS_RAW_Y2, SCREEN_HEIGHT, 0);
  
  // [Legacy] Long Press Logic (CALIB_AUTO를 위해 유지)
  if (isAdjustingValue) {
     if (millis() - touchDownTime > INITIAL_DELAY) {
        if (millis() - lastRepeatTime > REPEAT_INTERVAL) {
           lastRepeatTime = millis();
           if (currentScreen == SCREEN_SETTINGS_CALIB_AUTO) {
              adjustAutoCalibValue(repeatDirection);
           }
           screenNeedsRedraw = true;
        }
     }
     return;
  }

  // One-Shot Flag
  bool is_new_touch = !prev_touched_flag;
  prev_touched_flag = true; 
  
  if (currentScreen == SCREEN_WARNING) { 
    warningActive = false;
    currentScreen = SCREEN_RELAY_CONTROL;
    screenNeedsRedraw = true; 
    delay(200); 
    return; 
  }

  // Common Back Button
  if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
    if (p.x >= 0 && p.x <= 60 && p.y >= 0 && p.y <= 40) { 
      if (currentScreen == SCREEN_COMBINED_WAVEFORM) { isWaveformFrozen = false; }
      if (currentScreen == SCREEN_SETTINGS_CALIB_AUTO) { auto_calib_step = 0; }

      isMainPowerFrozen = false;
      isPhaseFrozen = false;
      isHarmonicsFrozen = false;
      
      // [Mod] WIFI CONFIG 화면에서 뒤로가기 시 Station 모드 복구 로직 추가
      if (currentScreen == SCREEN_WIFI_CONFIG) {
          sendAT("AT+CWMODE=1\r\n", 1000, true); // Station 모드로 복귀
          currentScreen = SCREEN_SETTINGS_NETWORK;
          screenNeedsRedraw = true;
          delay(100);
          return;
      }

      if (currentScreen == SCREEN_SETTINGS_CALIB_MENU || currentScreen == SCREEN_SETTINGS_CALIB_MANUAL || currentScreen == SCREEN_SETTINGS_CALIB_AUTO) {
        if ((currentScreen == SCREEN_SETTINGS_CALIB_MANUAL) && settingsChanged) { // AUTO 화면 뒤로가기 시 저장/버리기 팝업 비활성화 [User Request]
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
               currentScreen == SCREEN_SETTINGS_CREDIT ||
               currentScreen == SCREEN_CONFIRM_SAVE) {
        if (currentScreen == SCREEN_CONFIRM_SAVE) currentScreen = SCREEN_SETTINGS; 
        else if (currentScreen == SCREEN_SETTINGS_NETWORK) currentScreen = SCREEN_SETTINGS;
        else if (currentScreen == SCREEN_SETTINGS_PRESETS) currentScreen = SCREEN_SETTINGS_ADVANCED;
        else if (currentScreen == SCREEN_SETTINGS_THEME || currentScreen == SCREEN_SETTINGS_RESET) currentScreen = SCREEN_SETTINGS_ADVANCED;
        else if (currentScreen == SCREEN_SETTINGS_CREDIT) currentScreen = SCREEN_SETTINGS_ADVANCED; 
        else {
           if (currentScreen == SCREEN_SETTINGS_TIMER) currentScreen = SCREEN_SETTINGS;
           else currentScreen = SCREEN_SETTINGS_ADVANCED;
        }
      }
      else if (currentScreen == SCREEN_CREDIT_MEMBER_1 || currentScreen == SCREEN_CREDIT_MEMBER_2 || currentScreen == SCREEN_CREDIT_MEMBER_3) {
        currentScreen = SCREEN_SETTINGS_CREDIT;
      }
      else if (currentScreen == SCREEN_SETTINGS_ADVANCED) currentScreen = SCREEN_SETTINGS;
      else if (currentScreen == SCREEN_SETTINGS || currentScreen == SCREEN_RELAY_CONTROL) currentScreen = SCREEN_HOME;
      else currentScreen = SCREEN_HOME;
      
      screenNeedsRedraw = true;
      delay(100); 
      return;
    }
  }

  switch (currentScreen) {
    case SCREEN_HOME:
      if (p.x >= 15 && p.x <= 155 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_MAIN_POWER; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_PHASE_DIFFERENCE; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_COMBINED_WAVEFORM; screenNeedsRedraw = true; sendJsonCommand("{\"CMD\":\"REQ_WAVEFORM\"}"); }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_HARMONICS; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_RELAY_CONTROL; screenNeedsRedraw = true; }
      break;

    case SCREEN_MAIN_POWER: 
       // [Mod] One-Shot applied
       if (is_new_touch) {
           if (p.x >= 15 && p.x <= 105 && p.y >= 190 && p.y <= 235) {
              isMainPowerFrozen = !isMainPowerFrozen;
              String text = isMainPowerFrozen ? "RUN" : "HOLD";
              drawButton(20, 195, 80, 35, text); delay(200);
           }
       }
       break;

    case SCREEN_PHASE_DIFFERENCE:
       // [Mod] One-Shot applied (Added per request)
       if (is_new_touch) {
           if (p.x >= 15 && p.x <= 85 && p.y >= 200 && p.y <= 235) {
              isPhaseFrozen = !isPhaseFrozen;
              String text = isPhaseFrozen ? "RUN" : "HOLD";
              drawButton(20, 205, 60, 25, text); delay(200);
           }
       }
       break;

    case SCREEN_HARMONICS: 
      // [Mod] One-Shot applied to all buttons
      if (is_new_touch) {
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
      }
      break;

    case SCREEN_SETTINGS:
      if (p.x >= 15 && p.x <= 305 && p.y >= 45 && p.y <= 95) { 
        temp_V_MULTIPLIER = V_MULTIPLIER; temp_I_MULTIPLIER = I_MULTIPLIER; temp_setting_step_index = setting_step_index; settingsChanged = false; 
        currentScreen = SCREEN_SETTINGS_CALIB_MENU; screenNeedsRedraw = true; 
      } 
      else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { 
        temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD; temp_setting_step_index = setting_step_index; settingsChanged = false; currentScreen = SCREEN_SETTINGS_PROTECT; screenNeedsRedraw = true;
      } 
      else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { 
        currentScreen = SCREEN_SETTINGS_NETWORK; screenNeedsRedraw = true;
      } 
      else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) { 
        temp_timer_setting_seconds = timer_setting_seconds; currentScreen = SCREEN_SETTINGS_TIMER; screenNeedsRedraw = true; 
      }
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
      if (is_new_touch) {
          // Existing "Send to Thingspeak" Button
          if (p.x >= 55 && p.x <= 265 && p.y >= 45 && p.y <= 105) {
            if (wifiState == WIFI_OFF) { wifiState = WIFI_WAIT; lastWifiRetryTime = 0; screenNeedsRedraw = true; } 
            else if (wifiState == WIFI_WAIT) { wifiState = WIFI_OFF; screenNeedsRedraw = true; } 
            else if (wifiState == WIFI_CONNECTED_STATE) { sendAT("AT+CWQAP\r\n", 1000, true); wifiState = WIFI_OFF; screenNeedsRedraw = true; }
            delay(150); 
          }
          // [New] WEB CONFIG MODE Button
          else if (p.x >= 20 && p.x <= 300 && p.y >= 140 && p.y <= 180) {
            currentScreen = SCREEN_WIFI_CONFIG;
            enableSoftAP(); // SoftAP 모드 활성화
            screenNeedsRedraw = true;
            delay(150);
          }
      }
      break;

    case SCREEN_SETTINGS_ADVANCED:
      if (p.x >= 15 && p.x <= 155 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_SETTINGS_THEME; screenNeedsRedraw = true; } 
      else if (p.x >= 165 && p.x <= 305 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_SETTINGS_PRESETS; screenNeedsRedraw = true; } 
      else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_SETTINGS_RESET; screenNeedsRedraw = true; }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_CREDIT_SPLASH; screenNeedsRedraw = true; }
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
      // [Mod] Target Relay, Step, Start -> One-Shot
      if (is_new_touch) {
          if (p.x >= 60 && p.x <= 260 && p.y >= 40 && p.y <= 90) { 
             timer_target_relay++;
             if (timer_target_relay > 3) timer_target_relay = 1; 
             delay(150); 
          }
          else if (p.x >= 85 && p.x <= 155 && p.y >= 135 && p.y <= 185) { 
            timer_step_index = (timer_step_index + 1) % 6; 
            delay(150);
          } 
          else if (p.x >= 225 && p.x <= 315 && p.y >= 135 && p.y <= 185) {
            // [Mod] Long Press Start Time Record
            touchDownTime = millis();
            
            if (is_timer_active) { is_timer_active = false; } 
            else {
               if (temp_timer_setting_seconds > 0) {
                  timer_seconds_left = temp_timer_setting_seconds;
                  is_timer_active = true;
                  last_timer_tick = millis();
                  timer_setting_seconds = temp_timer_setting_seconds; 
               }
            }
            delay(150);
          }
      }
      // [Mod] (+, -) Button -> Continuous (Loop + Delay)
      if (p.x >= 15 && p.x <= 85 && p.y >= 135 && p.y <= 185) { 
        // [-] Button
        if (temp_timer_setting_seconds >= TIMER_STEP_VALUES[timer_step_index]) temp_timer_setting_seconds -= TIMER_STEP_VALUES[timer_step_index]; else temp_timer_setting_seconds = 0;
        // screenNeedsRedraw = true; // [Fix] Partial Update only
        delay(TOUCH_REPEAT_DELAY);
      } else if (p.x >= 155 && p.x <= 225 && p.y >= 135 && p.y <= 185) { 
        // [+] Button
        if (temp_timer_setting_seconds + TIMER_STEP_VALUES[timer_step_index] <= 359999) temp_timer_setting_seconds += TIMER_STEP_VALUES[timer_step_index];
        // screenNeedsRedraw = true; // [Fix] Partial Update only
        delay(TOUCH_REPEAT_DELAY);
      } 
      break;

    case SCREEN_SETTINGS_CALIB_MANUAL:
      // [Mod] Selection -> One-Shot
      if (is_new_touch) {
          if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) { calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS; delay(150); }
          else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) { calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS; delay(150); }
      }
      // [Mod] (+, -) Button -> Continuous (Loop + Delay)
      if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) { 
          adjustCalibValue(false); 
          delay(TOUCH_REPEAT_DELAY); 
      }
      else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) { 
          adjustCalibValue(true); 
          delay(TOUCH_REPEAT_DELAY); 
      }
      break;
      
    case SCREEN_SETTINGS_CALIB_AUTO:
      // [Legacy] 기존 로직 유지 (isAdjustingValue 방식)
      if (auto_calib_step == 0) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 115 && p.y <= 165) { auto_calib_step = 1; screenNeedsRedraw = true; }
      } else if (auto_calib_step == 2) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 125 && p.y <= 175) { 
           auto_calib_step = 3; calib_selection = 0; setting_step_index = 2; screenNeedsRedraw = true; 
         }
      } else if (auto_calib_step == 3) {
         if (is_new_touch) {
             if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) { calib_selection = (calib_selection - 1 + NUM_AUTOCALIB_INPUTS) % NUM_AUTOCALIB_INPUTS; delay(150); }
             else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) { calib_selection = (calib_selection + 1) % NUM_AUTOCALIB_INPUTS; delay(150); }
             else if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) { 
                 adjustAutoCalibValue(false); 
                 isAdjustingValue = true; repeatDirection = false; touchDownTime = millis(); lastRepeatTime = millis();
                 delay(TOUCH_REPEAT_DELAY); 
             }
             else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) { 
                 adjustAutoCalibValue(true); 
                 isAdjustingValue = true; repeatDirection = true; touchDownTime = millis(); lastRepeatTime = millis();
                 delay(TOUCH_REPEAT_DELAY); 
             }
             else if (p.x >= 255 && p.x <= 320 && p.y >= 0 && p.y <= 40) { calculateNewGains(temp_true_v, temp_true_i); auto_calib_step = 4; screenNeedsRedraw = true; delay(150); }
         }
      } else if (auto_calib_step == 4) {
         if (p.x >= 75 && p.x <= 245 && p.y >= 125 && p.y <= 175) { auto_calib_step = 5; screenNeedsRedraw = true; }
      } else if (auto_calib_step == 5) {
         if (p.x >= 15 && p.x <= 205 && p.y >= 175 && p.y <= 225) { 
           // [Mod] APPLY & SAVE: 즉시 저장 및 화면 이동 (CONFIRM 화면 생략)
           V_MULTIPLIER = temp_V_MULTIPLIER; 
           I_MULTIPLIER = temp_I_MULTIPLIER;
           setting_step_index = temp_setting_step_index;
           
           txJsonDoc.clear(); 
           txJsonDoc["CMD"] = "SET_CALIB"; 
           txJsonDoc["V_MULT"] = V_MULTIPLIER; 
           txJsonDoc["I_MULT"] = I_MULTIPLIER; 
           serializeJson(txJsonDoc, Serial1); 
           Serial1.println();
           
           txJsonDoc.clear(); 
           txJsonDoc["CMD"] = "SET_PROTECT"; 
           txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; 
           serializeJson(txJsonDoc, Serial1); 
           Serial1.println();
           
           txJsonDoc.clear(); 
           txJsonDoc["CMD"] = "SET_STEP"; 
           txJsonDoc["IDX"] = setting_step_index; 
           serializeJson(txJsonDoc, Serial1); 
           Serial1.println();
           
           settingsChanged = false; 
           currentScreen = SCREEN_SETTINGS; 
           screenNeedsRedraw = true;
         } 
         else if (p.x >= 205 && p.x <= 315 && p.y >= 175 && p.y <= 225) { auto_calib_step = 0; screenNeedsRedraw = true; }
      }
      break;

    case SCREEN_SETTINGS_PROTECT:
      // [Mod] Selection -> One-Shot
      if (is_new_touch) {
          if (p.x >= 15 && p.x <= 85 && p.y >= 175 && p.y <= 225) { protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS; delay(150); }
          else if (p.x >= 85 && p.x <= 155 && p.y >= 175 && p.y <= 225) { protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS; delay(150); }
      }
      // [Mod] (+, -) Button -> Continuous (Loop + Delay)
      if (p.x >= 175 && p.x <= 245 && p.y >= 175 && p.y <= 225) { 
          adjustProtectValue(false); 
          delay(TOUCH_REPEAT_DELAY); 
      }
      else if (p.x >= 245 && p.x <= 315 && p.y >= 175 && p.y <= 225) { 
          adjustProtectValue(true); 
          delay(TOUCH_REPEAT_DELAY); 
      }
      break;

    case SCREEN_RELAY_CONTROL:
      if (is_new_touch) {
        if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) { sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":2}"); delay(150); }
        else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) { sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":2}"); delay(150); }
      }
      break;

    case SCREEN_SETTINGS_RESET:
      if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { 
        // [User Request] ADC Midpoint Reset Code REMOVED Here
        
        sendJsonCommand("{\"CMD\":\"RESET_SETTINGS\"}"); 
        currentScreen = SCREEN_SETTINGS_ADVANCED; 
        screenNeedsRedraw = true; 
      }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
      break;

    case SCREEN_CONFIRM_SAVE:
      if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { // SAVE
        if (previousScreen == SCREEN_SETTINGS_CALIB_MANUAL) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; } 
        else if (previousScreen == SCREEN_SETTINGS_CALIB_AUTO) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; }
        else if (previousScreen == SCREEN_SETTINGS_PROTECT) { VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD; }
        
        setting_step_index = temp_setting_step_index;
        
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_CALIB"; txJsonDoc["V_MULT"] = V_MULTIPLIER; txJsonDoc["I_MULT"] = I_MULTIPLIER; serializeJson(txJsonDoc, Serial1); Serial1.println();
        
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_PROTECT"; txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; serializeJson(txJsonDoc, Serial1); Serial1.println();
        txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_STEP"; txJsonDoc["IDX"] = setting_step_index; serializeJson(txJsonDoc, Serial1); Serial1.println();
        
        settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
      }
      else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { // DISCARD
        settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
      }
      break;

    case SCREEN_SETTINGS_THEME:
      if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) { // Light
         isDarkMode = false; setTheme(); screenNeedsRedraw = true; 
      }
      else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) { // Dark
         isDarkMode = true; setTheme(); screenNeedsRedraw = true; 
      }
      break;

    case SCREEN_COMBINED_WAVEFORM:
      // [Mod] One-Shot: Type(Left), Period(Right)
      if (is_new_touch) {
          if (p.x >= 5 && p.x <= 110 && p.y >= 190 && p.y <= 235) {
            waveformPlotType = (waveformPlotType + 1) % 3;
            drawButton(10, 195, 100, 35, WAVEFORM_TYPE_LABELS[waveformPlotType]);
            screenNeedsRedraw = true; 
          }
          else if (p.x >= 215 && p.x <= 315 && p.y >= 190 && p.y <= 235) {
            waveformPeriodIndex = (waveformPeriodIndex + 1) % 3;
            drawButton(220, 195, 90, 35, WAVEFORM_PERIOD_LABELS[waveformPeriodIndex]);
          }
      }
      // [Fix] Trigger Mode(Center) -> Continuous allowed (Trigger check during press)
      if (p.x >= 110 && p.x <= 215 && p.y >= 190 && p.y <= 235) {
        if (waveformTriggerMode == 2 && isWaveformFrozen) isWaveformFrozen = false; 
        else { waveformTriggerMode = (waveformTriggerMode + 1) % 3; isWaveformFrozen = false; }
        drawButton(115, 195, 100, 35, String(WAVEFORM_MODE_LABELS[waveformTriggerMode]));
        delay(TOUCH_REPEAT_DELAY); // Simple debounce
      }
      break;
      
    case SCREEN_SETTINGS_CREDIT:
      if (p.x >= 15 && p.x <= 305 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_CREDIT_MEMBER_1; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 305 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_CREDIT_MEMBER_2; screenNeedsRedraw = true; }
      else if (p.x >= 15 && p.x <= 305 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_CREDIT_MEMBER_3; screenNeedsRedraw = true; }
      break;

    case SCREEN_CREDIT_MEMBER_1:
    case SCREEN_CREDIT_MEMBER_2:
    case SCREEN_CREDIT_MEMBER_3:
      break;
  }
  delay(100); 
}