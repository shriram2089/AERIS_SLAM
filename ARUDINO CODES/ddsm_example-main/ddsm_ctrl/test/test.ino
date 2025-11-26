#include <Arduino.h>
#include <ddsm_ctrl.h>

#define DDSM_RX 18
#define DDSM_TX 19

DDSM_CTRL dc;

int motor_id = 2;
int cmd_rpm = 200; // target rpm (positive = one direction, negative = reverse)
unsigned long lastPrint = 0;

// --- Stop all motors ---
void stopAllMotors() {
  for (int id = 1; id <= 4; id++) {
    dc.ddsm_ctrl(id, 0, 2); // send 0 RPM command + feedback
    delay(5);
  }
  Serial.println("All motors stopped.");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);

  dc.pSerial = &Serial1;
  dc.set_ddsm_type(210);
  dc.clear_ddsm_buffer();

  // Emergency stop at startup
  stopAllMotors();

  Serial.println("=== DDSM Single Motor Test ===");
  Serial.printf("Motor ID: %d | Target RPM: %d\n", motor_id, cmd_rpm);
  delay(1000);
}

void loop() {
  // Send command + read feedback in one shot (act=2)
  dc.ddsm_ctrl(motor_id, -cmd_rpm, 2);

  // Optional small delay for UART stability
  delay(2);

  // Print debug info every 200 ms
  if (millis() - lastPrint > 200) {
    Serial.printf("Motor %d | Target RPM: %d | Feedback RPM: %d | Current: %d | Temp: %d | Fault: %d\n",
                  motor_id, cmd_rpm, dc.speed_data, dc.current, dc.temperature, dc.fault_code);
    lastPrint = millis();
  }
}
