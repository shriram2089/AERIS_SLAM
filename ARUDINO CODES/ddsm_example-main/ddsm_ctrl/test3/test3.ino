#include <WiFi.h>
#include <WebServer.h>
#include <ddsm_ctrl.h>
#include <math.h>

#define DDSM_RX 18
#define DDSM_TX 19
#define WHEEL_RADIUS 0.0355
#define WHEEL_BASE 0.25

DDSM_CTRL dc;

// Robot state
float x = 0.0, y = 0.0, theta = 0.0;
unsigned long lastTime = 0, lastPrint = 0;

bool firstCommand = true;

// Motor speeds
int fr = 200, br = 200, fl = 200, bl = 200;

// Mode flag
enum Mode { MANUAL, AUTO };
Mode mode = AUTO;  // default AUTO for odom test

int test_pwm = 0; // default PWM for AUTO test

void setup() {
  Serial.begin(115200);
  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;
  dc.set_ddsm_type(210);
  dc.clear_ddsm_buffer();

  Serial.println("AUTO motor + odom with serial control starting...");
  lastTime = millis();
}

void processSerialCommand(String line) {
  line.trim();

  if (firstCommand) {
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    fr = br = fl = bl = 0;
    mode = AUTO;
    firstCommand = false;
    Serial.println("Starting fresh: pose reset");
  }

  if (line == "MODE:AUTO") {
    mode = AUTO;
    Serial.println("Switched to AUTO mode");
    return;
  }

  if (line == "MODE:MANUAL") {
    mode = MANUAL;
    Serial.println("Switched to MANUAL mode");
    return;
  }

  if (line == "RESET") {
    x = y = theta = 0.0;
    fr = br = fl = bl = 0;
    mode = AUTO;
    Serial.println("Pose and motors reset");
    return;
  }

  if (mode == AUTO && line.startsWith("V:")) {
    float v = 0.0, w = 0.0;
    int vIdx = line.indexOf("V:");
    int wIdx = line.indexOf("W:");

    if (vIdx != -1 && wIdx != -1) {
      v = line.substring(vIdx + 2, wIdx).toFloat();
      w = line.substring(wIdx + 2).toFloat();

      // Convert to wheel linear velocities
      float v_r = v + (w * WHEEL_BASE / 2.0);
      float v_l = v - (w * WHEEL_BASE / 2.0);

      // Convert linear velocity to motor command (fixed scaling)
      fr = br = -constrain(int(v_r * 60.0 / (2 * PI * WHEEL_RADIUS)), -1000, 1000); // Right inverted
      fl = bl =  constrain(int(v_l * 60.0 / (2 * PI * WHEEL_RADIUS)), -1000, 1000);  // Left normal

      dc.ddsm_ctrl(1, fr, 2);  // Front Right
      dc.ddsm_ctrl(2, br, 2);  // Back Right
      dc.ddsm_ctrl(3, fl, 2);  // Front Left
      dc.ddsm_ctrl(4, bl, 2);  // Back Left
    }
  }
}

void loop() {
  // Handle serial commands
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processSerialCommand(cmd);
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Read RPMs BEFORE sending new motor commands
  float rpm_fr = dc.speed_data;
  float rpm_br = dc.speed_data;
  float rpm_fl = dc.speed_data;
  float rpm_bl = dc.speed_data;

  // Send motor commands
  if (mode == AUTO) {
    // Default test PWM if no serial command overrides
    // dc.ddsm_ctrl(1, -test_pwm, 2); fr = -test_pwm; // FR inverted
    // dc.ddsm_ctrl(2, -test_pwm, 2); br = -test_pwm; // BR inverted
    // dc.ddsm_ctrl(3,  test_pwm, 2); fl =  test_pwm; // FL normal
    // dc.ddsm_ctrl(4,  test_pwm, 2); bl =  test_pwm; // BL normal
  } else {
    // Use fr/br/fl/bl set by serial commands
    dc.ddsm_ctrl(1, fr, 2);
    dc.ddsm_ctrl(2, br, 2);
    dc.ddsm_ctrl(3, fl, 2);
    dc.ddsm_ctrl(4, bl, 2);
  }

  delay(10); // let motors respond

  // Convert RPM to linear velocity
  float v_fr = 2 * PI * WHEEL_RADIUS * rpm_fr / 60.0;
  float v_br = 2 * PI * WHEEL_RADIUS * rpm_br / 60.0;
  float v_fl = 2 * PI * WHEEL_RADIUS * rpm_fl / 60.0;
  float v_bl = 2 * PI * WHEEL_RADIUS * rpm_bl / 60.0;

  float v_right = (v_fr + v_br) / 2.0;
  float v_left  = (v_fl + v_bl) / 2.0;

  // Robot velocities
  float vx = (v_left + v_right) / 2.0;
  float omega = (v_right - v_left) / WHEEL_BASE;

  // Update pose
  x += vx * cos(theta) * dt;
  y += vx * sin(theta) * dt;

  if (fabs(omega) < 0.01) omega = 0.0;

  y = 0.0; // keep as original
  theta += omega * dt;

  // Print every 50 ms
  if (now - lastPrint > 50) {
    Serial.printf("x: %.2f m, y: %.2f m, θ: %.2f rad\n", x, y, theta);
    lastPrint = now;
  }

  delay(50); // prevent overload
}
