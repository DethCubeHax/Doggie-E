#include <Arduino.h>
#include <ESP32Servo.h>
#include "GlobalVars.h"
#include "InverseKinematics.h"
#include "Leg.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define SERVOMIN 500 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 2500 // this is the 'maximum' pulse length count (out of 4096)

double upperLegLength = 124;
double lowerLegLength = 138;
double bodyWidth = 10;
double bodyLength = 10;

Coordinates dogCoords = {0,0,0};
Legs legs;
InverseKinematics inverseKinematics(upperLegLength, lowerLegLength, bodyWidth, bodyLength, SERVOMIN, SERVOMAX);
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
Leg leg(SERVOMIN, SERVOMAX, 60);

double minLength = sqrt(pow(upperLegLength, 2) + pow(lowerLegLength, 2));

void setup() {
  Serial.begin(9600);
  leg.initLeg();
  delay(10);
  // put your setup code here, to run once:
}

void loop() {
  for (double i = upperLegLength+lowerLegLength; i>=sqrt(pow(upperLegLength, 2) + pow(lowerLegLength,2)); i-=0.3)
  {
    inverseKinematics.getLocalXYZ(0, 0, i, 0, 0, 0, &legs);
    leg.legControl(legs);
    delay(100);
  }
  for (double i = sqrt(pow(upperLegLength, 2) + pow(lowerLegLength,2)); i<=upperLegLength+lowerLegLength; i+=0.3)
  {
    inverseKinematics.getLocalXYZ(0, 0, i, 0, 0, 0, &legs);

    leg.legControl(legs);
    delay(100);
  }

}