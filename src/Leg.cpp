#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "GlobalVars.h"
#include "InverseKinematics.h"
#include "Leg.h"

extern Adafruit_PWMServoDriver servos;
extern InverseKinematics inverseKinematics;

Leg::Leg(int min_servo_microseconds, int max_servo_microseconds, int pwm_freq)
{
    this->minServoMicroseconds = min_servo_microseconds;
    this->maxServoMicroseconds = max_servo_microseconds;
    this->pwmFreq = pwm_freq;
}

void Leg::initLeg()
{
    Serial.println("Initializing Legs");
    servos.begin();
    servos.setPWMFreq(this->pwmFreq);
    
}

void Leg::legControl(Legs legData)
{
    JointAngles jointAngles = inverseKinematics.getJointAngles(legData.rearRightCoords.x, legData.rearRightCoords.y, legData.rearRightCoords.z);
    double shoulderAngle = jointAngles.shoulderAngle;
    double hipAngle = jointAngles.hipAngle + PI/4;
    double kneeAngle = jointAngles.kneeAngle;
    writeServo(legData.rearRightPins[0], shoulderAngle);
    writeServo(legData.rearRightPins[1], hipAngle);
    writeServo(legData.rearRightPins[2], kneeAngle);


    jointAngles = inverseKinematics.getJointAngles(legData.frontRightCoords.x, legData.frontRightCoords.y, legData.frontRightCoords.z);
     shoulderAngle = jointAngles.shoulderAngle;
     hipAngle = jointAngles.hipAngle + PI/4;
     kneeAngle = jointAngles.kneeAngle;
    writeServo(legData.frontRightPins[0], shoulderAngle);
    writeServo(legData.frontRightPins[1], hipAngle);
    writeServo(legData.frontRightPins[2], kneeAngle);


    jointAngles = inverseKinematics.getJointAngles(legData.rearLeftCoords.x, legData.rearLeftCoords.y, legData.rearLeftCoords.z);
    shoulderAngle = jointAngles.shoulderAngle;
    hipAngle = PI - (jointAngles.hipAngle + PI/4);
    kneeAngle = PI - jointAngles.kneeAngle;
    writeServo(legData.rearLeftPins[0], shoulderAngle);
    writeServo(legData.rearLeftPins[1], hipAngle);
    writeServo(legData.rearLeftPins[2], kneeAngle);

    jointAngles = inverseKinematics.getJointAngles(legData.frontLeftCoords.x, legData.frontLeftCoords.y, legData.frontLeftCoords.z);
    shoulderAngle = jointAngles.shoulderAngle;
    hipAngle = PI - (jointAngles.hipAngle + PI/4);
    kneeAngle = PI - jointAngles.kneeAngle;
    writeServo(legData.frontLeftPins[0], shoulderAngle);
    writeServo(legData.frontLeftPins[1], hipAngle);
    writeServo(legData.frontLeftPins[2], kneeAngle);
}

void Leg::localLegControl(Coordinates *coords, double (&angleOffsets)[3], int (&pins)[3])
{
    JointAngles jointAngles = inverseKinematics.getJointAngles(coords->x, coords->y, coords->z);
    double shoulderAngle = jointAngles.shoulderAngle + angleOffsets[0];
    double hipAngle = jointAngles.hipAngle + angleOffsets[1];
    double kneeAngle = jointAngles.kneeAngle + angleOffsets[2];
    writeServo(pins[0], shoulderAngle);
    writeServo(pins[1], hipAngle);
    writeServo(pins[2], kneeAngle);
}

void Leg::writeServo(int servoNum, double servoAngle)
{
    servos.writeMicroseconds(servoNum, inverseKinematics.getMicroseconds(servoAngle));    
}