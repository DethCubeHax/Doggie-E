#ifndef LEG_H
#define LEG_H

#include "GlobalVars.h"
#include "InverseKinematics.h"
#include <ESP32Servo.h>

extern InverseKinematics inverseKinematics;
extern Servo servo[12];

class Leg
{
    public:
        Leg(int min_servo_microseconds, int max_servo_microseconds, int pwm_freq);
        void initLeg();
        void legControl(Legs legData);
    private:
        int minServoMicroseconds;
        int maxServoMicroseconds;
        int pwmFreq;
        void writeServo(int servoNum, double servoAngle);
        void localLegControl(Coordinates *coords, double (&angleOffsets)[3], int (&pins)[3]);
};


#endif