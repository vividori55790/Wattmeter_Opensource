/*
 * ==============================================================================
 * 파일명: 4. Comm_Input.ino
 * 버전: v212 (Trip Logic & Harmonics Parsing)
 * 설명: 
 * - [Mod] 트립 화면 터치 시 Relay Control 화면으로 이동 (릴레이 자동 해제 안함)
 * - [Mod] 고조파 데이터 파싱 강화 및 실제 변수 매핑
 * - [Mod] Advanced 설정 화면 3버튼 터치 영역 수정
 * - [Mod] Warning 화면 오작동 방지 로직 추가
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
  
  // [Mod] 고조파 데이터 파싱 (String -> Array)
  String hv = rxJsonDoc["H_V_STR"];
  String hi = rxJsonDoc["H_I_STR"];
  if (hv.length() > 0) {
    float temp_h[16]; // 넉넉하게
    parseCSV(hv, temp_h, 16);
    for(int k=0; k<=15; k++) v_harmonics[k] = temp_h[k]; 
  }
  if (hi.length() > 0) {
    float temp_h[16];
    parseCSV(hi, temp_h, 16);
    for(int k=0; k<=15; k++) i_harmonics[k] = temp_h[k];
  }

  V_MULTIPLIER = rxJsonDoc["V_MULT"] | V_MULTIPLIER;
  I_MULTIPLIER = rxJsonDoc["I_MULT"] | I_MULTIPLIER;
  VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"] | VOLTAGE_THRESHOLD;
  setting_step_index = rxJsonDoc["STEP_IDX"] | setting_step_index;

  relay1_state = rxJsonDoc["R1"]; 
  relay2_state = rxJsonDoc["R2"];
  
  // [Mod] Warning Logic 강화
  // 프로세서로부터 WARN 플래그가 True일 때만 경고 활성화
  bool mcu1_warning = rxJsonDoc["WARN"];
  if (mcu1_warning && !warningActive) {
    warningMessage = rxJsonDoc["MSG"] | "SYSTEM TRIP";
    tripReason = warningMessage; // 상세 사유 저장
    trippedRelayInfo = "ALL"; // MCU1 트립은 보통 전체 차단
    warningActive = true;
    screenNeedsRedraw = true; 
  } else if (!mcu1_warning && warningActive) {
    // MCU1 경고가 해제되었지만, 타이머 트립일 수 있으므로 구분
    if (warningMessage != "TIMER FINISHED") {
        warningActive = false;
        // 여기서 화면 전환은 하지 않음 (사용자가 확인 후 닫아야 함)
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
  static bool wasTouched = false; 
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  if (touched && !wasTouched) { 
    wasTouched = true; 
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    p.x = map(p.x, TS_RAW_X1, TS_RAW_X2, SCREEN_WIDTH, 0);
    p.y = map(p.y, TS_RAW_Y1, TS_RAW_Y2, SCREEN_HEIGHT, 0);
    
    // [Mod] 경고 화면 터치 로직 수정
    if (currentScreen == SCREEN_WARNING) { 
      // 경고 활성 상태만 해제 (릴레이는 제어하지 않음)
      warningActive = false;
      sendJsonCommand("{\"CMD\":\"RESET_WARNING\"}"); // MCU1에도 알림 (소리 끄기 등 목적)
      // 화면은 릴레이 제어 화면으로 이동하여 사용자가 직접 복구하도록 함
      currentScreen = SCREEN_RELAY_CONTROL;
      screenNeedsRedraw = true;
      delay(200); 
      return; 
    }

    if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
      if (p.x >= 0 && p.x <= 60 && p.y >= 0 && p.y <= 40) { 
        
        if (currentScreen == SCREEN_COMBINED_WAVEFORM) isMainPowerFrozen = false; // Var reuse fix
        if (currentScreen == SCREEN_SETTINGS_CALIB_AUTO) auto_calib_step = 0; 

        isMainPowerFrozen = false;
        isPhaseFrozen = false;
        isHarmonicsFrozen = false;
        
        if (currentScreen == SCREEN_SETTINGS_CALIB_MENU || currentScreen == SCREEN_SETTINGS_CALIB_MANUAL || currentScreen == SCREEN_SETTINGS_CALIB_AUTO) {
          if ((currentScreen == SCREEN_SETTINGS_CALIB_MANUAL || currentScreen == SCREEN_SETTINGS_CALIB_AUTO) && settingsChanged) { 
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
                 currentScreen == SCREEN_SETTINGS_PRESETS || 
                 currentScreen == SCREEN_SETTINGS_NETWORK || 
                 currentScreen == SCREEN_SETTINGS_TIMER || 
                 currentScreen == SCREEN_CONFIRM_SAVE) {
          if (currentScreen == SCREEN_CONFIRM_SAVE) {
             currentScreen = SCREEN_SETTINGS; 
          } else if (currentScreen == SCREEN_SETTINGS_NETWORK) {
             currentScreen = SCREEN_SETTINGS;
          } else if (currentScreen == SCREEN_SETTINGS_TIMER) {
             currentScreen = SCREEN_SETTINGS; 
          } else if (currentScreen == SCREEN_SETTINGS_PRESETS) {
             currentScreen = SCREEN_SETTINGS_ADVANCED;
          } else {
             currentScreen = SCREEN_SETTINGS_ADVANCED;
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
        if (p.x >= 15 && p.x <= 155 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_MAIN_POWER; screenNeedsRedraw = true; }
        else if (p.x >= 165 && p.x <= 305 && p.y >= 45 && p.y <= 95) { currentScreen = SCREEN_PHASE_DIFFERENCE; screenNeedsRedraw = true; }
        else if (p.x >= 15 && p.x <= 155 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_COMBINED_WAVEFORM; screenNeedsRedraw = true; }
        else if (p.x >= 165 && p.x <= 305 && p.y >= 105 && p.y <= 155) { currentScreen = SCREEN_HARMONICS; screenNeedsRedraw = true; }
        else if (p.x >= 15 && p.x <= 155 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true; }
        else if (p.x >= 165 && p.x <= 305 && p.y >= 165 && p.y <= 215) { currentScreen = SCREEN_RELAY_CONTROL; screenNeedsRedraw = true; }
        break;

      case SCREEN_MAIN_POWER: 
         if (p.x >= 15 && p.x <= 105 && p.y >= 190 && p.y <= 235) {
            isMainPowerFrozen = !isMainPowerFrozen;
            screenNeedsRedraw = true;
         }
         break;

      case SCREEN_PHASE_DIFFERENCE:
         if (p.x >= 15 && p.x <= 85 && p.y >= 200 && p.y <= 235) {
            isPhaseFrozen = !isPhaseFrozen;
            screenNeedsRedraw = true;
         }
         break;

      case SCREEN_HARMONICS: 
        if (p.x >= 10 && p.x <= 100 && p.y >= 200 && p.y <= 235) { 
            harmonicsSource = !harmonicsSource; 
            harmonicsSrcLabel = (harmonicsSource == 0) ? "Src: V" : "Src: I";
            screenNeedsRedraw = true;
        }
        else if (p.x >= 110 && p.x <= 210 && p.y >= 200 && p.y <= 235) { 
            isHarmonicsFrozen = !isHarmonicsFrozen; 
            screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS:
        if (p.x >= 20 && p.x <= 300 && p.y >= 50 && p.y <= 90) { 
          temp_V_MULTIPLIER = V_MULTIPLIER; temp_I_MULTIPLIER = I_MULTIPLIER; temp_setting_step_index = setting_step_index; settingsChanged = false; 
          currentScreen = SCREEN_SETTINGS_CALIB_MENU; screenNeedsRedraw = true; 
        } 
        else if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) { 
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD; temp_setting_step_index = setting_step_index; settingsChanged = false; currentScreen = SCREEN_SETTINGS_PROTECT; screenNeedsRedraw = true;
        } 
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) { 
          currentScreen = SCREEN_SETTINGS_NETWORK; screenNeedsRedraw = true;
        } 
        else if (p.x >= 20 && p.x <= 150 && p.y >= 150 && p.y <= 190) { 
            temp_timer_setting_seconds = timer_setting_seconds; 
            timer_target_relay = (timer_target_relay > 0) ? timer_target_relay : 3; 
            currentScreen = SCREEN_SETTINGS_TIMER; screenNeedsRedraw = true; 
        } 
        else if (p.x >= 170 && p.x <= 300 && p.y >= 150 && p.y <= 190) { 
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
        if (p.x >= 60 && p.x <= 260 && p.y >= 50 && p.y <= 100) {
           if (wifiState == WIFI_OFF) {
               wifiState = WIFI_WAIT;
               lastWifiRetryTime = 0; 
           } else {
               wifiState = WIFI_OFF;
               isWifiConnected = false;
               sendAT("AT+CWQAP\r\n", 1000, true);
           }
           screenNeedsRedraw = true; 
           delay(300);
        } 
        else if (p.y >= 135 && p.y <= 175) {
           if (p.x >= 20 && p.x <= 80) send_V = !send_V; 
           else if (p.x >= 90 && p.x <= 150) send_I = !send_I; 
           else if (p.x >= 160 && p.x <= 220) send_P = !send_P;
           delay(200); 
        }
        break;

      case SCREEN_SETTINGS_ADVANCED:
        // [Mod] Advanced Screen 3 Long Buttons Layout
        if (p.x >= 20 && p.x <= 300 && p.y >= 50 && p.y <= 90) { 
            currentScreen = SCREEN_SETTINGS_THEME; screenNeedsRedraw = true; 
        } else if (p.x >= 20 && p.x <= 300 && p.y >= 100 && p.y <= 140) { 
            currentScreen = SCREEN_SETTINGS_PRESETS; screenNeedsRedraw = true; 
        } else if (p.x >= 20 && p.x <= 300 && p.y >= 150 && p.y <= 190) { 
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
              if (isPresetSaveMode) { savePreset(slot); screenNeedsRedraw = true; }
              else { loadPreset(slot); screenNeedsRedraw = true; }
              delay(300);
           }
        }
        break;

      case SCREEN_SETTINGS_TIMER:
        if (p.x >= 60 && p.x <= 260 && p.y >= 45 && p.y <= 85) {
           timer_target_relay = (timer_target_relay % 3) + 1; 
           delay(200);
        }
        else if (p.y >= 150 && p.y <= 190) {
           if (p.x >= 20 && p.x <= 80) { 
              if (temp_timer_setting_seconds >= TIMER_STEP_VALUES[timer_step_index]) temp_timer_setting_seconds -= TIMER_STEP_VALUES[timer_step_index]; 
              else temp_timer_setting_seconds = 0;
           } 
           else if (p.x >= 90 && p.x <= 150) { 
              timer_step_index = (timer_step_index + 1) % 6; 
           } 
           else if (p.x >= 160 && p.x <= 220) { 
              if (temp_timer_setting_seconds + TIMER_STEP_VALUES[timer_step_index] <= 359999) temp_timer_setting_seconds += TIMER_STEP_VALUES[timer_step_index];
           }
           else if (p.x >= 230 && p.x <= 310) { 
              if (is_timer_active) {
                 is_timer_active = false;
                 timer_seconds_left = 0;
              } else {
                 is_timer_active = true;
                 timer_setting_seconds = temp_timer_setting_seconds;
                 timer_seconds_left = temp_timer_setting_seconds;
                 lastTimerTick = millis();
              }
              delay(200);
           }
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
        if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":2}");
        else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":2}");
        break;

      case SCREEN_SETTINGS_RESET:
        if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { sendJsonCommand("{\"CMD\":\"RESET_SETTINGS\"}"); currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
        else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
        break;

      case SCREEN_CONFIRM_SAVE:
        if (p.x >= 15 && p.x <= 155 && p.y >= 95 && p.y <= 145) { 
          if (previousScreen == SCREEN_SETTINGS_CALIB_MANUAL) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; } 
          else if (previousScreen == SCREEN_SETTINGS_CALIB_AUTO) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; }
          else if (previousScreen == SCREEN_SETTINGS_PROTECT) { VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD; }
          
          setting_step_index = temp_setting_step_index;
          
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_CALIB"; txJsonDoc["V_MULT"] = V_MULTIPLIER; txJsonDoc["I_MULT"] = I_MULTIPLIER; serializeJson(txJsonDoc, Serial1); Serial1.println();
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_PROTECT"; txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; serializeJson(txJsonDoc, Serial1); Serial1.println();
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_STEP"; txJsonDoc["IDX"] = setting_step_index; serializeJson(txJsonDoc, Serial1); Serial1.println();
          
          settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        else if (p.x >= 165 && p.x <= 305 && p.y >= 95 && p.y <= 145) { 
          settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_THEME:
        if (p.x >= 15 && p.x <= 305 && p.y >= 65 && p.y <= 115) { isDarkMode = false; setTheme(); screenNeedsRedraw = true; }
        else if (p.x >= 15 && p.x <= 305 && p.y >= 125 && p.y <= 175) { isDarkMode = true; setTheme(); screenNeedsRedraw = true; }
        break;
    }
  } 
  else if (!touched) {
    wasTouched = false; 
  }
}