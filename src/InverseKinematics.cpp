#include "InverseKinematics.h"
#include "GlobalVars.h"
#include <math.h>

InverseKinematics::InverseKinematics(double upper_leg_length, double lower_leg_length, double body_width, double body_length, int servo_min, int servo_max)
{
    this->upperLegLength = upper_leg_length;
    this->lowerLegLength = lower_leg_length;
    this->bodyLength = body_length;
    this->bodyWidth = body_width;
    this->servoMin = servo_min;
    this->servoMax = servo_max;
}

void InverseKinematics::getLocalXYZ(double x, double y, double z, double rotX, double rotY, double rotZ, Legs *legs)
{
    legs->rearRightCoords.x = x;
    legs->rearRightCoords.y = y;
    legs->rearRightCoords.z = z;

    legs->frontRightCoords.x = x;
    legs->frontRightCoords.y = y;
    legs->frontRightCoords.z = z;

    legs->rearLeftCoords.x = x;
    legs->rearLeftCoords.y = y;
    legs->rearLeftCoords.z = z;

    legs->frontLeftCoords.x = x;
    legs->frontLeftCoords.y = y;
    legs->frontLeftCoords.z = z;
}

JointAngles InverseKinematics::getJointAngles(double x, double y, double z)
{
    JointAngles jointAngles;
    double length = getLegLength(x, y, z);
    jointAngles.shoulderAngle = shoulderJointAngle(x, z);
    jointAngles.hipAngle = hipJointAngle(length);
    jointAngles.kneeAngle = kneeJointAngle(length);
    return jointAngles;
}


// Rotation offset is still being tested. Very likely broken.
void InverseKinematics::rotXOffset(Legs legs, double rotX)
{
    double zOffset = bodyWidth/2 * sin(rotX);
    double yOffset = bodyWidth/2 - bodyWidth/2 * cos(rotX);

    legs.frontLeftCoords.z += zOffset;
    legs.frontLeftCoords.y -= yOffset;

    legs.rearLeftCoords.z += zOffset;
    legs.rearLeftCoords.y -= yOffset;

    legs.frontRightCoords.z -= zOffset;
    legs.frontRightCoords.y += yOffset;

    legs.rearLeftCoords.z -= zOffset;
    legs.frontLeftCoords.y += yOffset;
}

void InverseKinematics::rotYOffset(Legs legs, double rotY)
{
    double zOffset = bodyWidth/2 * sin(rotY);
    double xOffset = bodyWidth/2 - bodyWidth/2 * cos(rotY);

    legs.frontLeftCoords.z += zOffset;
    legs.frontLeftCoords.x -= xOffset;

    legs.rearLeftCoords.z += zOffset;
    legs.rearLeftCoords.x -= xOffset;

    legs.frontRightCoords.z -= zOffset;
    legs.frontRightCoords.x += xOffset;

    legs.rearLeftCoords.z -= zOffset;
    legs.frontLeftCoords.x += xOffset;
}

void InverseKinematics::rotZOffset(Legs legs, double rotZ)
{
    double xOffset = bodyWidth/2 * sin(rotZ);
    double yOffset = bodyWidth/2 - bodyWidth/2 * cos(rotZ);

    legs.frontLeftCoords.x += xOffset;
    legs.frontLeftCoords.y -= yOffset;

    legs.rearLeftCoords.x -= xOffset;
    legs.rearLeftCoords.y -= yOffset;

    legs.frontRightCoords.x += xOffset;
    legs.frontRightCoords.y += yOffset;

    legs.rearLeftCoords.x -= xOffset;
    legs.frontLeftCoords.y += yOffset;
}

double InverseKinematics::shoulderJointAngle(double x, double z)
{
    double angle = atan(z/x);
    return angle;
}

double InverseKinematics::getLegLength(double x, double y, double z)
{
    double length = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    return length;
}

double InverseKinematics::hipJointAngle(double length)
{
    double angle = acos((pow(length, 2) + pow(upperLegLength, 2) - pow(lowerLegLength, 2)) / (2 * upperLegLength * length));
    return angle;
}

double InverseKinematics::kneeJointAngle(double length)
{
    double angle = acos((pow(upperLegLength, 2) + pow(lowerLegLength, 2) - pow(length, 2)) / (2 * upperLegLength * lowerLegLength));
    return angle;
}

long InverseKinematics::getMicroseconds(double Angle)
{
    double microseconds = ((this->servoMax - this->servoMin) * (Angle) / (PI)) + this->servoMin;
    return microseconds;
}