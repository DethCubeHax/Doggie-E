#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "GlobalVars.h"

class InverseKinematics
{
    public:
        InverseKinematics(double upper_leg_length, double lower_leg_length, double body_width, double body_length, int servo_min, int servo_max);
        void getLocalXYZ(double x, double y, double z, double rotX, double rotY, double rotZ, Legs *legs);
        long getMicroseconds(double angle);
        JointAngles getJointAngles(double x, double y, double z);
    private:
        double upperLegLength;
        double lowerLegLength;
        double bodyWidth;
        double bodyLength;
        int servoMin;
        int servoMax;
        void rotXOffset(Legs legs, double rotX);
        void rotYOffset(Legs legs, double rotY);
        void rotZOffset(Legs legs, double rotZ);        // Still under testing
        double shoulderJointAngle(double x, double z);
        double getLegLength(double x, double y, double z);
        double hipJointAngle(double length);
        double kneeJointAngle(double length);

};

#endif