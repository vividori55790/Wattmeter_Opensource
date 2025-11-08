/*
 * ==============================================================================
 * ESP-01 (ESP8266) MQTT 게이트웨이 코드
 * ==============================================================================
 * [역할]
 * 1. WiFi 및 MQTT 브로커에 접속합니다.
 * 2. Arduino Nano R4로부터 시리얼(UART)로 JSON 데이터를 수신합니다.
 * 3. 수신한 JSON 데이터를 'wattmeter/data' 토픽으로 MQTT 브로커에 발행(Publish)합니다.
 *
 * [필요 라이브러리 (Arduino IDE 라이브러리 매니저에서 설치)]
 * 1. PubSubClient (by Nick O'Leary)
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// *** 1. WiFi 설정 ***
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// --- 2. MQTT 브로커 설정 ---
// (테스트용 공개 브로커 사용. 실제 환경에서는 로컬 Mosquitto나 유료 브로커 권장)
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_data_topic = "wattmeter/data"; // 데이터 발행 토픽

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println("\n[MQTT Gateway] Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  // MQTT 연결이 끊어졌으면 5초마다 재시도
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // 고유한 Client ID 생성
    String clientId = "ESP8266-Wattmeter-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); // Nano R4와 동일한 속도
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  Serial.println("MQTT Gateway setup complete.");
}

void loop() {
  // 1. MQTT 연결 유지
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop(); // MQTT 클라이언트 상태 체크

  // 2. Nano R4 (Serial)로부터 JSON 데이터가 수신되었는지 확인
  if (Serial.available() > 0) {
    // 줄바꿈 문자('\n')를 만날 때까지 데이터를 읽음
    String jsonData = Serial.readStringUntil('\n');
    
    // 유효한 JSON 데이터인지 간단히 확인 (시작과 끝이 {} 인지)
    if (jsonData.startsWith("{") && jsonData.endsWith("}")) {
      // 수신한 데이터를 MQTT 토픽으로 발행(Publish)
      client.publish(mqtt_data_topic, jsonData.c_str());
      
      // (디버깅용) 발행한 데이터 출력
      // Serial.print("Published to MQTT: ");
      // Serial.println(jsonData);
    }
  }
}