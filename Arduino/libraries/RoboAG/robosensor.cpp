#include "roboag.h"
// uint8_t (Wertebereich: 0–255)
// int8_t (8 Bit, Wertebereich -128 bis 127)
// uint16_t (Wertebereich: 0 bis 65.535)

const uint8_t ledDataPin = 6;
const uint8_t numberOfLeds = 5;

uint8_t brightness = 50;

uint8_t outerLeftSensorPin = A0;
uint8_t innerLeftSensorPin = A1;
uint8_t innerRightSensorPin = A2;
uint8_t outerRightSensorPin = A3;

uint16_t outerLeftThreshold = 100;
uint16_t innerLeftThreshold = 100;
uint16_t innerRightThreshold = 100;
uint16_t outerRightThreshold = 100;

uint8_t outerLeftLED = 0;
uint8_t innerLeftLED = 1;
uint8_t miscLED = 2;
uint8_t innerRightLED = 3;
uint8_t outerRightLED = 4;
bool isIRSensor = false;

UltraSonicDistanceSensor distanceSensor(7, 9);  // trigger/echo
unsigned int lastDistance = 400;
unsigned long lastDistanceCheck = 0;
const int MIN_DISTANCE_CHECK_MS = 30;

CRGB leds[numberOfLeds];

void setupSensors() {
  // https://github.com/thijse/Arduino-Log/
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  pinMode(innerLeftSensorPin, INPUT);
  pinMode(innerRightSensorPin, INPUT);
  pinMode(outerLeftSensorPin, INPUT);
  pinMode(outerRightSensorPin, INPUT);
  FastLED.addLeds<WS2812B, ledDataPin, GRB>(leds, numberOfLeds);
  FastLED.setBrightness(brightness);
}

void setupSensors(uint8_t outerLeftSensor, uint8_t innerLeftSensor, uint8_t innerRightSensor, uint8_t outerRightSensor) {
  outerLeftSensorPin = outerLeftSensor;
  innerLeftSensorPin = innerLeftSensor;
  innerRightSensorPin = innerRightSensor;
  outerRightSensorPin = outerRightSensor;
  setupSensors();
}

void enableIRSensor() {
  isIRSensor = true;
  outerLeftThreshold = 600;
  innerLeftThreshold = 600;
  innerRightThreshold = 600;
  outerRightThreshold = 600;
}

void disableLogging() {
  Log.begin(LOG_LEVEL_FATAL, &Serial);
}

void printSensorValues() {
  Log.noticeln(F("links außen: %d, links innen: %d, rechts innen: %d, rechts außen: %d"),
               analogRead(outerLeftSensorPin),
               analogRead(innerLeftSensorPin),
               analogRead(innerRightSensorPin),
               analogRead(outerRightSensorPin));
}

void printPinout() {
  Log.noticeln(F("Pinout:"));
  Log.noticeln(F("Sensoren: links außen: %d, links innen: %d, rechts innen: %d, rechts außen: %d"),
               outerLeftSensorPin,
               innerLeftSensorPin,
               innerRightSensorPin,
               outerRightSensorPin);
  Log.noticeln(F("Anmerkung: A0 -> 14, A1 -> 15, A2 -> 16. A3 -> 17, A4 -> 18, A5 -> 19, A6 -> 20, A7 -> 21"));
  Log.noticeln(F("Default Trigger Pin: %d, Echo Pin: %d"), 7, 9);
  Log.noticeln(F("Motor links: %d, %d, %d"), motorLeft1Pin, motorLeft2Pin, motorLeftSpeedPin);
  Log.noticeln(F("Motor rechts: %d, %d, %d"), motorRight1Pin, motorRight2Pin, motorRightSpeedPin);
}

bool changeLed(uint8_t led, CRGB color) {
  if (leds[led] != color) {
    leds[led] = color;
    return true;
  }
  return false;
}

bool isBlack(uint8_t pin, uint16_t threshold, uint8_t led) {
  bool result = analogRead(pin) < threshold;
  result = isIRSensor ? !result : result;
  if (changeLed(led, result ? CRGB::White : CRGB::Black)) {
    FastLED.show();
  }
  return result;
}

void setOuterLeftThreshold(uint16_t threshold) {
  outerLeftThreshold = threshold;
}

void setOuterRightThreshold(uint16_t threshold) {
  outerRightThreshold = threshold;
}

void setInnerLeftThreshold(uint16_t threshold) {
  innerLeftThreshold = threshold;
}

void setInnerRightThreshold(uint16_t threshold) {
  innerRightThreshold = threshold;
}

bool isOuterLeftBlack() {
  return isBlack(outerLeftSensorPin, outerLeftThreshold, outerLeftLED);
}

bool isOuterRightBlack() {
  return isBlack(outerRightSensorPin, outerRightThreshold, outerRightLED);
}

bool isInnerLeftBlack() {
  return isBlack(innerLeftSensorPin, innerLeftThreshold, innerLeftLED);
}

bool isInnerRightBlack() {
  return isBlack(innerRightSensorPin, innerRightThreshold, innerRightLED);
}

bool isSilver(uint8_t pin, uint8_t led) {
  int value = analogRead(pin);  // Wertebereich 0 - 1023
  bool result = isIRSensor ? value < 20 : value > 1010;
  if (result && changeLed(led, CRGB::Blue)) {
    FastLED.show();
  }
  return result;
}

bool isSilver() {
  return isSilver(innerRightSensorPin, innerRightLED) || isSilver(innerLeftSensorPin, innerLeftLED) || isSilver(outerRightSensorPin, outerRightLED) || isSilver(outerLeftSensorPin, outerLeftLED);
}

int getDistanceInMM() {
  if (millis() - lastDistanceCheck > MIN_DISTANCE_CHECK_MS) {
    lastDistanceCheck = millis();
    float currentDistance = distanceSensor.measureDistanceCm();
    if (currentDistance >= 0) {
      lastDistance = currentDistance*10;
    }
  }
  return lastDistance;
}
void printCurrentDistance() {
  Log.noticeln(F("Distanz: %d mm"), getDistanceInMM());
}

void ledAllOff() {
  bool changed = false;
  for (int i = 0; i < numberOfLeds; i++) {
    changed = changeLed(i, CRGB::Black) || changed;
  }
  if (changed) {
    FastLED.show();
  }
}
void ledSetupDone() {
  uint8_t delayTime = 60;
  for (int i = 0; i < numberOfLeds; i++) {
    leds[i] = CRGB::Green;
    FastLED.show();
    delay(delayTime);
    if (i > 0) {
      leds[i - 1] = CRGB::Black;
      FastLED.show();
    }
  }
  delay(delayTime * 3);
  for (int i = numberOfLeds; i >= 0; i--) {
    leds[i] = CRGB::Yellow;
    FastLED.show();
    delay(delayTime);
  }
  delay(100);
  ledAllOff();
}
void ledObstacleFound() {
  if (changeLed(miscLED, CRGB::Red)) {
    FastLED.show();
  }
}
void ledObstacleDone() {
  if (changeLed(miscLED, CRGB::Black)) {
    FastLED.show();
  }
}

void ledIntersectionFound() {
  if (changeLed(miscLED, CRGB::Green)) {
    FastLED.show();
  }
}

void ledIntersectionDone() {
  if (changeLed(miscLED, CRGB::Black)) {
    FastLED.show();
  }
}
