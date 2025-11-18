/*
 * --- CONTROLLER 코드 (Nano R4 - 통신 테스트) ---
 * Serial1 (D0, D1)을 사용합니다.
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

// ... (핀 정의는 동일) ...
#define TFT_CS    10
#define TFT_DC    9
#define TFT_RST   8
#define TOUCH_CS  7

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(XPT2046_Touchscreen(TOUCH_CS));

void setup() {
  // Serial.begin(9600); // (X) 이게 아님
  Serial1.begin(9600); // D0, D1 하드웨어 시리얼
  
  tft.begin();
  ts.begin();
  tft.setRotation(1);
  ts.setRotation(1);
  drawReadyScreen();
}

void drawReadyScreen() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(20, 100);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.println("Touch to Ping (R4)");
}

void loop() {
  if (ts.touched()) {
    tft.fillScreen(ILI9341_YELLOW);
    tft.setCursor(20, 100);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(3);
    tft.println("Pinging Processor...");

    // 'P' 신호 전송
    Serial1.write('P');

    // 응답 기다리기 (최대 2초)
    unsigned long startTime = millis();
    bool pongReceived = false;

    while (millis() - startTime < 2000) {
      if (Serial1.available()) { // Serial1로 변경
        char response = Serial1.read(); // Serial1로 변경
        if (response == 'K') {
          pongReceived = true;
          break;
        }
      }
    }

    // 결과 표시
    if (pongReceived) {
      tft.fillScreen(ILI9341_GREEN);
      tft.setCursor(20, 100);
      tft.setTextColor(ILI9341_BLACK);
      tft.setTextSize(3);
      tft.println("SUCCESS! (Got 'K')");
    } else {
      tft.fillScreen(ILI9341_RED);
      tft.setCursor(20, 100);
      tft.setTextColor(ILI9341_WHITE);
      tft.setTextSize(3);
      tft.println("FAIL! (Timeout)");
    }

    delay(1000);
    while (ts.touched());
    drawReadyScreen();
  }
}