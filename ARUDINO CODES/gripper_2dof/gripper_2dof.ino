#include <Servo.h>

Servo servo1;  // Base pitch
Servo servo2;  // Gripper

// --- Calibrated zero positions ---
int angle1 = 120;
int angle2 = 130;

// --- Servo step size ---
const int stepSize = 3;
const int minAngle = 0;
const int maxAngle = 300;

void setup() {
  Serial.begin(115200);
  servo1.attach(10);
  servo2.attach(11);

  // Move servos to calibrated zero positions
  servo1.write(angle1);
  servo2.write(angle2);

  Serial.println("Servos ready. Commands: o (open), c (close), u (up), d (down)");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    switch (c) {
      case 'd':  // Open gripper
        angle2 = min(angle2 + stepSize, maxAngle);
        servo2.write(angle2);
        break;

      case 'u':  // Close gripper
        angle2 = max(angle2 - stepSize, minAngle);
        servo2.write(angle2);
        break;

      case 'o':  // Move arm down
        angle1 = min(angle1 + stepSize, maxAngle);
        servo1.write(angle1);
        break;

      case 'c':  // Move arm up
        angle1 = max(angle1 - stepSize, minAngle);
        servo1.write(angle1);
        break;

      default:
        break;
    }
  }
}
