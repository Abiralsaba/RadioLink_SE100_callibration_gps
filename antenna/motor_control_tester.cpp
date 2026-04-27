#include <Arduino.h>

#define MOTOR1_PIN 18

void sendPulse(int microseconds) {
  microseconds = constrain(microseconds, 1000, 2000);
  digitalWrite(MOTOR1_PIN, HIGH);
  delayMicroseconds(microseconds);
  digitalWrite(MOTOR1_PIN, LOW);
}

void stopMotor() { sendPulse(1460); }

void moveRight(int speed = 10) {
  speed = constrain(speed, 0, 100);
  int pulse = map(speed, 0, 100, 1460, 2000);
  sendPulse(pulse);
}

void moveLeft(int speed = 10) {
  speed = constrain(speed, 0, 100);
  int pulse = map(speed, 0, 100, 1460, 1000);
  sendPulse(pulse);
}

// void setup() {
//     Serial.begin(115200);
//     pinMode(MOTOR1_PIN, OUTPUT);
//     digitalWrite(MOTOR1_PIN, LOW);
//
//     for (int i = 0; i < 50; i++) {
//         stopMotor();
//         delay(20);
//     }
//
//     Serial.println("Ready. Commands: f <0-100> | b <0-100> | s");
// }
//
// void loop() {
//     static char currentCmd = 's';
//     static int currentSpeed = 0;
//
//     if (Serial.available()) {
//         String input = Serial.readStringUntil('\n');
//         input.trim();
//
//         currentCmd = input.charAt(0);
//         currentSpeed = input.substring(2).toInt();
//
//         if (currentCmd == 'r') {
//             Serial.print("Forward: ");
//             Serial.println(currentSpeed);
//         }
//         else if (currentCmd == 'l') {
//             Serial.print("Back: ");
//             Serial.println(currentSpeed);
//         }
//         else if (currentCmd == 's') {
//             currentSpeed = 0;
//             Serial.println("Stop");
//         }
//         else {
//             Serial.println("Unknown. Use: f <0-100> | b <0-100> | s");
//         }
//     }
//
//     // Always call function every loop
//     if (currentCmd == 'r')      moveRight(currentSpeed);
//     else if (currentCmd == 'l') moveLeft(currentSpeed);
//     else                        stopMotor();
//
//     delay(20);
// }
