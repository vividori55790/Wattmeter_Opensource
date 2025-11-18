/*
 * ==============================================================================
 * 파일명: 4. Comm_Input.ino
 * 버전: v101 (Serial1 for Inter-MCU Communication)
 * 설명: MCU1과의 데이터 송수신을 Serial1(하드웨어 UART)로 처리합니다.
 * ==============================================================================
 */

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
    // JSON 파싱 실패는 무시 (노이즈 등)
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

  V_MULTIPLIER = rxJsonDoc["V_MULT"] | V_MULTIPLIER;
  I_MULTIPLIER = rxJsonDoc["I_MULT"] | I_MULTIPLIER;
  VOLTAGE_THRESHOLD = rxJsonDoc["V_THR"] | VOLTAGE_THRESHOLD;
  setting_step_index = rxJsonDoc["STEP_IDX"] | setting_step_index;

  relay1_state = rxJsonDoc["R1"]; 
  relay2_state = rxJsonDoc["R2"];
  is_timer_active = rxJsonDoc["T_ACTIVE"];
  timer_seconds_left = rxJsonDoc["T_LEFT_S"] | 0;
  timer_setting_seconds = rxJsonDoc["T_SEC"] | 0;
  
  bool newWarning = rxJsonDoc["WARN"];
  if (newWarning && !warningActive) {
    warningMessage = rxJsonDoc["MSG"] | "WARNING!";
    warningActive = true;
    screenNeedsRedraw = true; 
  } else if (!newWarning && warningActive) {
    warningActive = false;
    currentScreen = SCREEN_HOME;
    screenNeedsRedraw = true; 
  }
}

// --- 설정 값 변경 헬퍼 (Calib) ---
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
  
  serializeJson(txJsonDoc, Serial1); // Serial1 사용
  Serial1.println();
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

  serializeJson(txJsonDoc, Serial1); // Serial1 사용
  Serial1.println();
}

// --- 터치 입력 확인 ---
void checkTouchInput() {
  SPI.beginTransaction(spiSettingsTouch);
  bool touched = ts.touched(); 
  SPI.endTransaction();
  
  if (touched) {
    SPI.beginTransaction(spiSettingsTouch);
    TS_Point p = ts.getPoint();
    SPI.endTransaction();

    p.x = map(p.x, TS_RAW_X1, TS_RAW_X2, SCREEN_WIDTH, 0);
    p.y = map(p.y, TS_RAW_Y1, TS_RAW_Y2, SCREEN_HEIGHT, 0);
    
    if (currentScreen == SCREEN_WARNING) { 
      sendJsonCommand("{\"CMD\":\"RESET_WARNING\"}");
      delay(100); 
      return; 
    }

    if (currentScreen != SCREEN_HOME && currentScreen != SCREEN_WARNING) {
      if (p.x >= 5 && p.x <= 55 && p.y >= 5 && p.y <= 35) {
        if (currentScreen == SCREEN_COMBINED_WAVEFORM) {
          isWaveformFrozen = false; 
        }

        isMainPowerFrozen = false;
        isPhaseFrozen = false;
        isHarmonicsFrozen = false;
        
        if (currentScreen == SCREEN_SETTINGS_CALIB || currentScreen == SCREEN_SETTINGS_PROTECT) {
          if (settingsChanged) { 
            previousScreen = currentScreen;
            currentScreen = SCREEN_CONFIRM_SAVE;
          } else { 
             currentScreen = SCREEN_SETTINGS;
          }
        } 
        else if (currentScreen == SCREEN_SETTINGS_THEME || 
                 currentScreen == SCREEN_SETTINGS_RESET || 
                 currentScreen == SCREEN_SETTINGS_TIMER || 
                 currentScreen == SCREEN_SETTINGS_NETWORK ||
                 currentScreen == SCREEN_CONFIRM_SAVE) {
          if (currentScreen == SCREEN_CONFIRM_SAVE) {
             currentScreen = SCREEN_SETTINGS; 
          } else if (currentScreen == SCREEN_SETTINGS_NETWORK) {
             currentScreen = SCREEN_SETTINGS;
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
        if (p.x >= 20 && p.x <= 150 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_MAIN_POWER;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 50 && p.y <= 90) {
          currentScreen = SCREEN_PHASE_DIFFERENCE;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_COMBINED_WAVEFORM;
          screenNeedsRedraw = true;
          sendJsonCommand("{\"CMD\":\"REQ_WAVEFORM\"}");
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 110 && p.y <= 150) {
          currentScreen = SCREEN_HARMONICS; 
          screenNeedsRedraw = true;
        }
        else if (p.x >= 20 && p.x <= 150 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_SETTINGS;
          screenNeedsRedraw = true;
        }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 170 && p.y <= 210) {
          currentScreen = SCREEN_RELAY_CONTROL; 
          screenNeedsRedraw = true;
        }
        break;

      case SCREEN_MAIN_POWER: 
         if (p.x >= 20 && p.x <= 100 && p.y >= 195 && p.y <= 230) {
            isMainPowerFrozen = !isMainPowerFrozen;
            String text = isMainPowerFrozen ? "RUN" : "HOLD";
            drawButton(20, 195, 80, 35, text); delay(200);
         }
         break;

      case SCREEN_PHASE_DIFFERENCE:
         if (p.x >= 20 && p.x <= 80 && p.y >= 205 && p.y <= 230) {
            isPhaseFrozen = !isPhaseFrozen;
            String text = isPhaseFrozen ? "RUN" : "HOLD";
            drawButton(20, 205, 60, 25, text); delay(200);
         }
         break;

      case SCREEN_HARMONICS: 
        if (p.x >= 10 && p.x <= 100 && p.y >= 200 && p.y <= 235) {
           harmonicsSource = !harmonicsSource; screenNeedsRedraw = true; 
        }
        else if (p.x >= 110 && p.x <= 210 && p.y >= 200 && p.y <= 235) {
           isHarmonicsFrozen = !isHarmonicsFrozen;
        }
        else if (p.x >= 220 && p.x <= 310 && p.y >= 200 && p.y <= 235) {
           harmonicsViewMode = !harmonicsViewMode; screenNeedsRedraw = true; 
        }
        break;

      case SCREEN_SETTINGS:
        if (p.x >= 20 && p.x <= 300 && p.y >= 50 && p.y <= 85) { 
          temp_V_MULTIPLIER = V_MULTIPLIER; temp_I_MULTIPLIER = I_MULTIPLIER; temp_setting_step_index = setting_step_index; settingsChanged = false; currentScreen = SCREEN_SETTINGS_CALIB; screenNeedsRedraw = true;
        } else if (p.x >= 20 && p.x <= 300 && p.y >= 90 && p.y <= 125) { 
          temp_VOLTAGE_THRESHOLD = VOLTAGE_THRESHOLD; temp_setting_step_index = setting_step_index; settingsChanged = false; currentScreen = SCREEN_SETTINGS_PROTECT; screenNeedsRedraw = true;
        } else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 165) { 
          currentScreen = SCREEN_SETTINGS_NETWORK; screenNeedsRedraw = true;
        } else if (p.x >= 20 && p.x <= 300 && p.y >= 170 && p.y <= 205) { 
          currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_NETWORK: 
        if (p.x >= 60 && p.x <= 260 && p.y >= 50 && p.y <= 100) {
          isWifiEnabled = !isWifiEnabled;
          if (isWifiEnabled) connectWiFi(); else { sendAT("AT+CWQAP\r\n", 1000, true); isWifiConnected = false; }
          delay(200);
        } else if (p.y >= 135 && p.y <= 175) {
           if (p.x >= 20 && p.x <= 80) send_V = !send_V; else if (p.x >= 90 && p.x <= 150) send_I = !send_I; else if (p.x >= 160 && p.x <= 220) send_P = !send_P;
        }
        break;

      case SCREEN_SETTINGS_ADVANCED:
        if (p.x >= 20 && p.x <= 300 && p.y >= 60 && p.y <= 95) { currentScreen = SCREEN_SETTINGS_THEME; screenNeedsRedraw = true; }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 105 && p.y <= 140) { temp_timer_setting_seconds = timer_setting_seconds; currentScreen = SCREEN_SETTINGS_TIMER; screenNeedsRedraw = true; }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 150 && p.y <= 185) { currentScreen = SCREEN_SETTINGS_RESET; screenNeedsRedraw = true; }
        break;

      case SCREEN_SETTINGS_TIMER:
        if (p.x >= 20 && p.x <= 80 && p.y >= 115 && p.y <= 165) {
          if (temp_timer_setting_seconds >= TIMER_STEP_VALUES[timer_step_index]) temp_timer_setting_seconds -= TIMER_STEP_VALUES[timer_step_index]; else temp_timer_setting_seconds = 0;
        } else if (p.x >= 90 && p.x <= 150 && p.y >= 115 && p.y <= 165) {
          timer_step_index = (timer_step_index + 1) % 6; 
        } else if (p.x >= 160 && p.x <= 220 && p.y >= 115 && p.y <= 165) {
          if (temp_timer_setting_seconds + TIMER_STEP_VALUES[timer_step_index] <= 359999) temp_timer_setting_seconds += TIMER_STEP_VALUES[timer_step_index];
        } else if (p.x >= 230 && p.x <= 310 && p.y >= 115 && p.y <= 165) {
          if (is_timer_active) sendJsonCommand("{\"CMD\":\"SET_TIMER\", \"SEC\":0}"); else sendJsonCommand("{\"CMD\":\"SET_TIMER\", \"SEC\":" + String(temp_timer_setting_seconds) + "}");
          delay(200);
        }
        break;

      case SCREEN_SETTINGS_CALIB:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) calib_selection = (calib_selection - 1 + NUM_CALIB_SETTINGS) % NUM_CALIB_SETTINGS;
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) calib_selection = (calib_selection + 1) % NUM_CALIB_SETTINGS;
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) adjustCalibValue(false); 
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) adjustCalibValue(true); 
        break;

      case SCREEN_SETTINGS_PROTECT:
        if (p.x >= 20 && p.x <= 80 && p.y >= 180 && p.y <= 220) protect_selection = (protect_selection - 1 + NUM_PROTECT_SETTINGS) % NUM_PROTECT_SETTINGS;
        else if (p.x >= 90 && p.x <= 150 && p.y >= 180 && p.y <= 220) protect_selection = (protect_selection + 1) % NUM_PROTECT_SETTINGS;
        else if (p.x >= 180 && p.x <= 240 && p.y >= 180 && p.y <= 220) adjustProtectValue(false); 
        else if (p.x >= 250 && p.x <= 310 && p.y >= 180 && p.y <= 220) adjustProtectValue(true); 
        break;

      case SCREEN_RELAY_CONTROL:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":1, \"STATE\":2}");
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) sendJsonCommand("{\"CMD\":\"SET_RELAY\", \"ID\":2, \"STATE\":2}");
        break;

      case SCREEN_SETTINGS_RESET:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) { sendJsonCommand("{\"CMD\":\"RESET_SETTINGS\"}"); currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) { currentScreen = SCREEN_SETTINGS_ADVANCED; screenNeedsRedraw = true; }
        break;

      case SCREEN_CONFIRM_SAVE:
        if (p.x >= 20 && p.x <= 150 && p.y >= 100 && p.y <= 140) { settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true; }
        else if (p.x >= 170 && p.x <= 300 && p.y >= 100 && p.y <= 140) {
          if (previousScreen == SCREEN_SETTINGS_CALIB) { V_MULTIPLIER = temp_V_MULTIPLIER; I_MULTIPLIER = temp_I_MULTIPLIER; } 
          else if (previousScreen == SCREEN_SETTINGS_PROTECT) { VOLTAGE_THRESHOLD = temp_VOLTAGE_THRESHOLD; }
          setting_step_index = temp_setting_step_index;
          
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_CALIB"; txJsonDoc["V_MULT"] = V_MULTIPLIER; txJsonDoc["I_MULT"] = I_MULTIPLIER; serializeJson(txJsonDoc, Serial1); Serial1.println();
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_PROTECT"; txJsonDoc["V_THR"] = VOLTAGE_THRESHOLD; serializeJson(txJsonDoc, Serial1); Serial1.println();
          txJsonDoc.clear(); txJsonDoc["CMD"] = "SET_STEP"; txJsonDoc["IDX"] = setting_step_index; serializeJson(txJsonDoc, Serial1); Serial1.println();
          
          settingsChanged = false; currentScreen = SCREEN_SETTINGS; screenNeedsRedraw = true;
        }
        break;

      case SCREEN_SETTINGS_THEME:
        if (p.x >= 20 && p.x <= 300 && p.y >= 70 && p.y <= 110) { isDarkMode = false; setTheme(); screenNeedsRedraw = true; }
        else if (p.x >= 20 && p.x <= 300 && p.y >= 130 && p.y <= 170) { isDarkMode = true; setTheme(); screenNeedsRedraw = true; }
        break;

      case SCREEN_COMBINED_WAVEFORM:
        if (p.x >= 10 && p.x <= 110 && p.y >= 195 && p.y <= 230) {
          waveformPlotType = (waveformPlotType + 1) % 3;
          drawButton(10, 195, 100, 35, WAVEFORM_TYPE_LABELS[waveformPlotType]);
          
          if (waveformPlotType == 0) { plot1_axis_max = V_AXIS_STEPS[4]; plot2_axis_max = I_AXIS_STEPS[2]; } 
          else if (waveformPlotType == 1) { plot1_axis_max = P_AXIS_STEPS[3]; plot2_axis_max = P_AXIS_STEPS[3]; } 
          else { plot1_axis_max = I_AXIS_STEPS[2]; plot2_axis_max = I_AXIS_STEPS[2]; plot3_axis_max = I_AXIS_STEPS[2]; }
          updateYAxisLabels(); screenNeedsRedraw = true; 
        }
        else if (p.x >= 115 && p.x <= 215 && p.y >= 195 && p.y <= 230) {
          if (waveformTriggerMode == 2 && isWaveformFrozen) isWaveformFrozen = false; 
          else { waveformTriggerMode = (waveformTriggerMode + 1) % 3; isWaveformFrozen = false; }
          drawButton(115, 195, 100, 35, String(WAVEFORM_MODE_LABELS[waveformTriggerMode]));
        }
        else if (p.x >= 220 && p.x <= 310 && p.y >= 195 && p.y <= 230) {
          waveformPeriodIndex = (waveformPeriodIndex + 1) % 3;
          drawButton(220, 195, 90, 35, WAVEFORM_PERIOD_LABELS[waveformPeriodIndex]);
        }
        break;
    }
    delay(100); 
  }
}