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

// Motor speeds
int fr = 0, br = 0, fl = 0, bl = 0;

// Mode flag
enum Mode { MANUAL, AUTO };
Mode mode = MANUAL;

// Web server
WebServer server(80);
int speed_val = 500;

const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>DDSM Car</title></head>
<body style="text-align:center;">
<h1>SLAM MOBILE ROS</h1>
<button onclick="send('F')">Forward</button><br><br>
<button onclick="send('L')">Left</button>
<button onclick="send('S')">Stop</button>
<button onclick="send('R')">Right</button><br><br>
<button onclick="send('B')">Backward</button>
<script>
function send(dir) {
  fetch("/cmd?dir=" + dir);
}
</script>
</body>
</html>
)rawliteral";

void moveMotorsManual(String dir) {
  if (mode != MANUAL) return;  // ignore in AUTO

  if (dir == "F") { fr = -speed_val; br = -speed_val; fl = speed_val;  bl = speed_val; }
  else if (dir == "B") { fr = speed_val;  br = speed_val;  fl = -speed_val; bl = -speed_val; }
  else if (dir == "R") { fr = speed_val;  br = speed_val;  fl = speed_val;  bl = speed_val; }
  else if (dir == "L") { fr = -speed_val; br = -speed_val; fl = -speed_val; bl = -speed_val; }
  else if (dir == "S") { fr = br = fl = bl = 0; }

  dc.ddsm_ctrl(1, fr, 2); dc.ddsm_ctrl(2, br, 2);
  dc.ddsm_ctrl(3, fl, 2); dc.ddsm_ctrl(4, bl, 2);
}

void handleCommand() {
  if (server.hasArg("dir")) {
    moveMotorsManual(server.arg("dir"));
  }
  server.send(200, "text/plain", "OK");
}

void handleRoot() {
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;
  dc.set_ddsm_type(210);
  dc.clear_ddsm_buffer();

  WiFi.softAP("SLAM_MOBILE_ros", "12345678");
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();

  Serial.println("ESP32 ready. Default mode: MANUAL.");
  lastTime = millis();
}

void processSerialCommand(String line) {
  line.trim();

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

  if (line == "RESET:ODOM") {
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  Serial.println("Odometry reset");
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

      // Convert linear velocity to RPM
      float rpm_r = (v_r * 60.0) / (2 * PI * WHEEL_RADIUS);
      float rpm_l = (v_l * 60.0) / (2 * PI * WHEEL_RADIUS);

      // Convert RPM to motor command (scale factor 10 matches DDSM)
      int pwm_r = constrain(int(rpm_r * 10), -1000, 1000);
      int pwm_l = constrain(int(rpm_l * 10), -1000, 1000);

      // Apply to motors (flip right side if needed)
      fr = br = -pwm_r;  // Right side (inverted)
      fl = bl = pwm_l;   // Left side

      dc.ddsm_ctrl(1, -fr, 2);  // Front Right
      dc.ddsm_ctrl(2, -br, 2);  // Back Right
      dc.ddsm_ctrl(3, -fl, 2);  // Front Left
      dc.ddsm_ctrl(4, -bl, 2);  // Back Left

      //Serial.printf("v: %.2f, w: %.2f -> pwm_l: %d, pwm_r: %d\n", v, w, pwm_l, pwm_r);
    } 
  }
}


void loop() {
  // Handle Wi-Fi client
  if (mode == MANUAL) server.handleClient();

  // Handle serial commands
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processSerialCommand(cmd);
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;



  dc.ddsm_ctrl(1, -fr, 2); delay(1); float rpm_fr = dc.speed_data;
  dc.ddsm_ctrl(2, -br, 2); delay(1); float rpm_br = dc.speed_data;
  dc.ddsm_ctrl(3, -fl, 2); delay(1); float rpm_fl = dc.speed_data;
  dc.ddsm_ctrl(4, -bl, 2); delay(1); float rpm_bl = dc.speed_data;

  float v_fr = 2 * PI * WHEEL_RADIUS * rpm_fr / 600.0;
  float v_br = 2 * PI * WHEEL_RADIUS * rpm_br / 600.0;
  float v_fl = 2 * PI * WHEEL_RADIUS * rpm_fl / 600.0;
  float v_bl = 2 * PI * WHEEL_RADIUS * rpm_bl / 600.0;

  float v_right = (v_fr + v_br) / 2.0;
  float v_left  = (v_fl + v_bl) / 2.0;

//  float vx = - (v_left - v_right) / 2.0;
//  float omega = (v_right + v_left) / WHEEL_BASE;
//
//  x += vx * cos(theta) * dt;
//  y += vx * sin(theta) * dt;
//  theta += omega * dt;

//  if (now - lastPrint > 50) {
//    Serial.printf("x: %.2f m, y: %.2f m, θ: %.2f rad\n", x, y, theta);
//    lastPrint = now;
//  }

     float ds = - (v_left - v_right) / 2.0;
  float dtheta = - (v_right + v_left) / WHEEL_BASE;

  // Send every 50ms
  if (now - lastPrint > 50) {
    Serial.printf("ds: %.5f m, dtheta: %.5f rad\n", ds, dtheta);
    lastPrint = now;
  }
}
