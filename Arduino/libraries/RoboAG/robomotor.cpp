#include "roboag.h"

//nano


uint8_t speed = 100;

void setupMotors() {
  pinMode(motorLeft1Pin, OUTPUT);
  pinMode(motorLeft2Pin, OUTPUT);
  pinMode(motorLeftSpeedPin, OUTPUT);

  pinMode(motorRight1Pin, OUTPUT);
  pinMode(motorRight2Pin, OUTPUT);
  pinMode(motorRightSpeedPin, OUTPUT);
}

void setSpeed(uint8_t newSpeed) {
  speed = constrain(newSpeed, 0, 100);
}
int adaptSpeed(uint8_t percentSpeed) {
  return map(percentSpeed, 0, 100, 0, 255);
}

void driveSingleMotor(uint8_t pin1, uint8_t pin2, uint8_t pinSpeed, uint8_t percentSpeed, bool forward) {
  digitalWrite(pin1, forward ? HIGH : LOW);
  digitalWrite(pin2, forward ? LOW : HIGH);
  analogWrite(pinSpeed, adaptSpeed(percentSpeed));
}


void driveForward() {
  driveSingleMotor(motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin, speed, true);
  driveSingleMotor(motorRight1Pin, motorRight2Pin, motorRightSpeedPin, speed, true);
}
void driveBackward() {
  driveSingleMotor(motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin, speed, false);
  driveSingleMotor(motorRight1Pin, motorRight2Pin, motorRightSpeedPin, speed, false);
}
void driveLeft() {
  driveSingleMotor(motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin, speed, false);
  driveSingleMotor(motorRight1Pin, motorRight2Pin, motorRightSpeedPin, speed, true);
}
void driveRight() {
  driveSingleMotor(motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin, speed, true);
  driveSingleMotor(motorRight1Pin, motorRight2Pin, motorRightSpeedPin, speed, false);
}
void stop() {
  analogWrite(motorLeftSpeedPin, LOW);
  analogWrite(motorRightSpeedPin, LOW);
}
void drive(int8_t  left, int8_t right) {
  driveSingleMotor(motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin, constrain(abs(left), 0, 100), left > 0);
  driveSingleMotor(motorRight1Pin, motorRight2Pin, motorRightSpeedPin, constrain(abs(right), 0, 100), right > 0);
}
void testMotors() {
  int delayTime = 2000;
  Log.noticeln(F("Vorwärts fahren"));
  driveForward();
  delay(delayTime);
  Log.noticeln(F("Links fahren"));
  driveLeft();
  delay(delayTime);
  Log.noticeln(F("Rechts fahren"));
  driveRight();
  delay(delayTime);
  Log.noticeln(F("Leicht Links fahren"));
  drive(50, 100);
  delay(delayTime);
  Log.noticeln(F("Leicht Rechts fahren"));
  drive(100, 50);
  delay(delayTime);
  Log.noticeln(F("Rückwärts fahren"));
  driveBackward();
  delay(delayTime*3);
  Log.noticeln(F("Stop"));
  stop();
  delay(delayTime);
}