#include <Servo.h>

Servo servo1;      // Base pitch
Servo servo2;      // Gripper
Servo lidarServo;  // LiDAR tilt

// --- Calibrated zero positions ---
int angle1 = 20;    
int angle2 = 130;   
int lidarAngle = 25;

// --- Servo step size ---
const int stepSize = 3;
const int minAngle = 0;
const int maxAngle = 180;

// --- LiDAR scanning limits ---
const int lidarMin = 25;
const int lidarMax = 25;

int lidarDir = 1;                   // 1=increasing, -1=decreasing
unsigned long lastLidarMove = 0;
unsigned long lidarInterval = 80;   // default ms per step

void setup() {
  Serial.begin(115200);
  servo1.attach(10);
  servo2.attach(11);
  lidarServo.attach(9);

  // Move servos to calibrated zero positions
  servo1.write(angle1);
  servo2.write(angle2);
  lidarServo.write(lidarAngle);

  Serial.println("Servos ready. Commands: o/c/u/d, lidar speed: sNN");
}

void loop() {
  // --- Non-blocking serial read ---
  if (Serial.available() > 0) {
    char c = Serial.read();

    // --- LiDAR speed command ---
    if (c == 's') {
      // Read numeric part (non-blocking)
      String num = "";
      unsigned long tStart = millis();
      while (millis() - tStart < 30) { // wait a few ms for digits
        if (Serial.available()) {
          char d = Serial.read();
          if (isDigit(d)) num += d;
          else break;
        }
      }
      if (num.length() > 0) {
        int newSpeed = num.toInt();
        if (newSpeed >= 10 && newSpeed <= 500) {
          lidarInterval = newSpeed;
          Serial.print("LiDAR interval set to: ");
          Serial.println(lidarInterval);
        }
      }
    }

    // --- Gripper & pitch commands ---
    else {
      switch (c) {
        case 'o': angle2 = min(angle2 + stepSize, maxAngle); servo2.write(angle2); break;
        case 'c': angle2 = max(angle2 - stepSize, minAngle); servo2.write(angle2); break;
        case 'd': angle1 = min(angle1 + stepSize, maxAngle); servo1.write(angle1); break;
        case 'u': angle1 = max(angle1 - stepSize, minAngle); servo1.write(angle1); break;
        case 'l': lidarAngle = max(lidarAngle - stepSize, lidarMin); lidarServo.write(lidarAngle); break;
        case 'r': lidarAngle = min(lidarAngle + stepSize, lidarMax); lidarServo.write(lidarAngle); break;
        default: break;
      }
    }
  }

  // --- Continuous LiDAR scanning ---
  unsigned long now = millis();
  if (now - lastLidarMove >= lidarInterval) {
    lastLidarMove = now;

    lidarAngle += lidarDir;
    if (lidarAngle >= lidarMax) { lidarAngle = lidarMax; lidarDir = -1; }
    else if (lidarAngle <= lidarMin) { lidarAngle = lidarMin; lidarDir = 1; }

    lidarServo.write(lidarAngle);

    // --- Report current LiDAR angle ---
    Serial.print("Lidar:");
    Serial.println(lidarAngle);
  }
}
