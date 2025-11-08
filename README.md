### **Arduino Nano R4 핀 결선 상세 (v2 기준)**

* **MCU:** Arduino Nano R4
* **전원 공급:**
    * `GMS1005` (AC-DC 5V) `OUT+` 핀 → Nano R4 `5V` 핀
    * `GMS1005` (AC-DC 5V) `OUT-` 핀 → Nano R4 `GND` 핀
    * *(이 5V 전원이 Nano R4, 두 센서 모듈, 두 릴레이 모듈의 VCC로 사용됩니다.)*

* **아날로그 센서 입력 (ADC):**
    * `ZMPT101B` (전압 센서) `OUT` 핀 → Nano R4 `A3`
    * `ACS712` (전류 센서) `OUT` 핀 → Nano R4 `A4`
    * *(두 센서 모듈의 VCC/GND는 모두 Nano R4의 5V/GND에 연결합니다.)*

* **디지털 출력 (릴레이):**
    * **메인 릴레이 (과전류 보호):** `IN` 핀 → Nano R4 `D7` **(D13에서 변경됨)**
    * **PFC 릴레이 (역률 개선):** `IN` 핀 → Nano R4 `D12`

* **TFT LCD (ILI9341) & 로직 레벨 컨버터 (필수):**
    * 5V (Nano)와 3.3V (LCD) 간의 신호 레벨 변환을 위해 **2개의 4채널 로직 레벨 컨버터**[cite: 1494-1496]를 사용합니다.
    * **컨버터 전원:**
        * 모든 컨버터의 `HV` (High Voltage) 측 `VCC`/`GND` → Nano R4 `5V`/`GND`
        * 모든 컨버터의 `LV` (Low Voltage) 측 `VCC`/`GND` → Nano R4 `3V3` 핀/`GND`
        * LCD 모듈의 `VCC`/`GND` 핀 → Nano R4 `3V3` 핀/`GND`
    * **컨버터 신호 연결:**
        * `Nano D13 (SCK)` → 컨버터1 `HV1` | 컨버터1 `LV1` → LCD `SCK`
        * `Nano D11 (MOSI)` → 컨버터1 `HV2` | 컨버터1 `LV2` → LCD `SDI` (MOSI)
        * `Nano D10 (TFT_CS)` → 컨버터1 `HV3` | 컨버터1 `LV3` → LCD `CS`
        * `Nano D9 (TFT_DC)` → 컨버터1 `HV4` | 컨버터1 `LV4` → LCD `DC`
        * `Nano D8 (TFT_RST)` → 컨버터2 `HV1` | 컨버터2 `LV1` → LCD `RST`



### **요약 및 주의사항**

1.  **핵심 변경:** **메인 릴레이가 D13에서 D7로 이동**했습니다. 이로써 D13은 LCD의 SPI 클럭(SCK) 신호 전용으로 안전하게 사용할 수 있습니다.
2.  **AC 전원부:** 220V AC를 다루는 `GMS1005`, `ZMPT101B`, `ACS712`[cite: 1530-1532, 1545-1547] 주변 회로 연결은 매우 위험합니다. 반드시 절연 상태를 확인하고, 누전 차단기를 통해 전원을 인가해야 합니다.
3.  **교정 (Calibration):** 이 핀 결선이 완료된 후, 코드 상단의 4가지 상수(`V_OFFSET_ADC`, `I_OFFSET_ADC`, `K_V_REAL`, `K_I_REAL`)를 실제 환경에 맞게 보정해야만 정확한 측정이 가능합니다.
