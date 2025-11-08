MQTT 게이트웨이 설정 및 연동 가이드 (수정본)

이 가이드는 ESP-01을 MQTT 게이트웨이로 설정하고, PC의 웹 브라우저에서 MQTT 데이터를 실시간으로 확인하는 방법을 설명합니다.

이전 방식과 달리, 웹페이지 파일(data.html)은 ESP-01에 업로드할 필요가 없습니다.

1단계: ESP-01 코드 업로드

Arduino IDE 환경 설정:

"ESP8266" 보드 매니저가 설치되어 있는지 확인합니다.

툴 > 보드에서 "Generic ESP8266 Module"을 선택합니다.

라이브러리 매니저에서 PubSubClient 라이브러리를 검색하여 설치합니다.

게이트웨이 코드 업로드:

ESP01_MQTT_Gateway.ino 스케치를 엽니다.

코드 상단의 ssid (WiFi 이름)와 password (WiFi 비밀번호)를 실제 환경에 맞게 수정합니다.

mqtt_server는 테스트용 공개 브로커(broker.hivemq.com)로 설정되어 있습니다. (이대로 사용 가능)

스케치를 ESP-01에 업로드합니다.

업로드 후, 시리얼 모니터를 열어 ESP-01이 WiFi와 MQTT 브로커에 성공적으로 접속하는지 확인합니다.

2단계: Nano R4와 ESP-01 하드웨어 연결 (필수)

경고: 5V ↔ 3.3V 레벨 변환이 필수입니다. ESP-01은 3.3V로만 동작합니다.

필요 부품: 양방향 로직 레벨 컨버터 (e.g., 1324248_1.jpg[cite: 1478-1481])

전원 연결:

Nano R4 5V → 레벨 컨버터 HV

Nano R4 3.3V → 레벨 컨버터 LV 및 ESP-01 VCC 및 CH_PD

Nano R4 GND → 레벨 컨버터 GND (양쪽) 및 ESP-01 GND

시리얼(UART) 신호 연결:

Nano R4 D1 (TX) (5V 신호) → 레벨 컨버터 HV1

레벨 컨버터 LV1 → ESP-01 RXD (3.3V 입력)

ESP-01 TXD (3.3V 신호) → 레벨 컨버터 LV2

레벨 컨버터 HV2 → Nano R4 D0 (RX) (5V 입력)

3단계: Nano R4 코드 수정 (JSON 전송)

Nano R4가 ESP-01로 데이터를 전송하도록 Wattmeter_NanoR4_MultiLoad_Fuzzy.ino 코드의 setup()과 loop() 함수에 시리얼 전송 로직을 추가합니다.

// ... (Nano R4 코드의 setup 함수에 추가) ...
void setup() {
  Serial.begin(115200); // ESP-01과 통신 속도
  // ... (기존 setup 코드) ...
}


// ... (Nano R4 코드의 loop 함수 끝, apply_relay_states() 호출 직후에 추가) ...
void loop() {
  // ... (기존 loop 코드) ...

  if (calculationReady) {
    // ... (기존 계산 로직) ...
    // ... (display_power_values 호출) ...
    // ... (check_pfc_relay 호출) ...
    // ... (check_load_shedding 호출) ...
    // ... (apply_relay_states 호출) ...

    // --- [신규] ESP-01로 전송할 JSON 데이터 생성 ---
    String json = "{";
    json += "\"v_rms\":" + String(V_rms_real, 1);
    json += ",\"thd\":" + String(THD_value, 1);
    json += ",\"p_total\":" + String(Total_P_real, 0);
    json += ",\"pf_total\":" + String(Total_PF, 2);

    // 시스템 상태 문자열 전송
    String sys_status = "NORMAL";
    if (protectionTripped) sys_status = "FAULT";
    else if (timerExpired) sys_status = "TIMER_EXPIRED";
    else if (loadShedLatch[0] || loadShedLatch[1] || loadShedLatch[2]) sys_status = "SHEDDING";
    json += ",\"status\":\"" + sys_status + "\"";

    json += ",\"loads\":[";
    for (int i = 0; i < NUM_LOADS; i++) {
      json += "{";
      json += "\"id\":" + String(i + 1);
      json += ",\"p_real\":" + String(P_real[i], 0);
      json += ",\"i_rms\":" + String(I_rms[i], 1);
      json += ",\"pf\":" + String(PF[i], 2);
      
      // 부하 상태 문자열 전송
      String load_state = "OFF";
      if (protectionTripped) load_state = "FAULT";
      else if (loadShedLatch[i]) load_state = "SHED";
      else if (loadRelayState[i]) load_state = "ON"; // 타이머 만료는 OFF로 처리됨
      
      json += ",\"state\":\"" + load_state + "\"";
      json += "}";
      if (i < NUM_LOADS - 1) json += ",";
    }
    json += "]";
    json += "}\n"; // JSON 종료 및 줄바꿈 문자(필수)

    // ESP-01로 JSON 전송 (Serial.print 사용)
    Serial.print(json); 
    
    // 9. FFT 카운터 초기화 (기존 코드)
    fft_count = 0;
  }
}


4단계: 웹페이지에서 확인하기

PC에 iot_wattmeter_mqtt.html 파일을 저장합니다.

Nano R4와 ESP-01의 전원을 켭니다.

PC에서 iot_wattmeter_mqtt.html 파일을 웹 브라우저(크롬, 엣지 등)로 열기만 하면 됩니다.

웹페이지가 자동으로 broker.hivemq.com에 접속하여, ESP-01이 발행한 wattmeter/data 토픽을 구독하고 데이터를 화면에 표시합니다.
