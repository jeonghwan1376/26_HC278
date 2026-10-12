#include <Dynamixel2Arduino.h>

#define DXL_DIR_PIN 1
#define DXL_ID 3

Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

float target_deg = 0.0;
String rx_data = "";

// 다이나믹셀 내부 0 위치를 사용자 기준 0도로 사용
int32_t home_pulse = 0;

// 1회전 = 4096 pulse = 360도
float pulse_per_deg = 4096.0 / 360.0;

void setup() {
  Serial.begin(1000000);   // PC 디버깅용 시리얼

  dxl.begin(1000000);
  dxl.setProtocol(DXL_ID, 2.0);

  if (dxl.ping(DXL_ID)) {
    Serial.println("Connected");

    dxl.ledOn(DXL_ID);

    dxl.torqueOff(DXL_ID);

    // 음수 방향 위치 제어를 위해 Extended Position Mode 사용
    dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);

    dxl.torqueOn(DXL_ID);

    // 전원 켜면 항상 다이나믹셀 내부 0 위치로 이동
    dxl.setGoalPosition(DXL_ID, home_pulse);

  } else {
    Serial.println("Fail");
  }

  Serial.println("Send angle like: 30 or -45");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (rx_data.length() > 0) {
        target_deg = rx_data.toFloat();

        // 필요시 각도 제한
        if (target_deg > 180.0) {
          target_deg = 180.0;
        }

        if (target_deg < -180.0) {
          target_deg = -180.0;
        }

        // 입력 각도를 pulse 값으로 변환
        // 0도   -> 0 pulse
        // 30도  -> 약 341 pulse
        // -30도 -> 약 -341 pulse
        int32_t target_pulse = home_pulse + (int32_t)(target_deg * pulse_per_deg);

        // 목표 위치 전송
        dxl.setGoalPosition(DXL_ID, target_pulse);

        Serial.print("Received angle: ");
        Serial.print(target_deg);
        Serial.print(" deg, Goal pulse: ");
        Serial.println(target_pulse);

        rx_data = "";
      }
    } else {
      rx_data += c;
    }
  }
}