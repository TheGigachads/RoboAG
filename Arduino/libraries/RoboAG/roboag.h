#ifndef ROBOAG_H
#define ROBOAG_H
#include <Arduino.h>
#include <FastLED.h>
#include <ArduinoLog.h>
#include <HCSR04.h>

const uint8_t motorLeft1Pin = 2;
const uint8_t motorLeft2Pin = 12;
const uint8_t motorLeftSpeedPin = 3;

const uint8_t motorRight1Pin = 4;
const uint8_t motorRight2Pin = 8;
const uint8_t motorRightSpeedPin = 11;

// drive functions
void setupMotors();
void setSpeed(uint8_t speed);
void driveForward();
void driveBackward();
void driveLeft();
void driveRight();
void stop();
void drive(int8_t left, int8_t right);
void testMotors();

// sensor functions
void setupSensors();
void setupSensors(uint8_t outerLeftSensor, uint8_t innerLeftSensor, uint8_t innerRightSensor, uint8_t outerRightSensor);

void enableIRSensor();

bool isOuterLeftBlack();
bool isOuterRightBlack();
bool isInnerLeftBlack();
bool isInnerRightBlack();
void setOuterLeftThreshold(uint16_t threshold);
void setInnerLeftThreshold(uint16_t threshold);
void setInnerRightThreshold(uint16_t threshold);
void setOuterRightThreshold(uint16_t threshold);
bool isSilver();
int getDistanceInMM();

// led functions

void ledSetupDone();
void ledObstacleFound();
void ledObstacleDone();
void ledIntersectionFound();
void ledIntersectionDone();

// utility functions
void disableLogging();
void printPinout();
void printSensorValues();
void printCurrentDistance();

#endif // ROBOAG_H