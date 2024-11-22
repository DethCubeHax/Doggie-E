#ifndef GLOBALVARS_H
#define GLOBALVARS_H

#include <Arduino.h>

struct JointAngles
{
    double shoulderAngle;
    double hipAngle;
    double kneeAngle;
};

struct Coordinates
{
    double x;
    double y;
    double z;
};

struct Legs
{   
    int rearRightPins[3] = {0,1,2};
    Coordinates rearRightCoords = {0,0,0};
    double rearRightAngleOffsets[3] = {0, 0, 0};

    int frontRightPins[3] = {4,5,6};
    Coordinates frontRightCoords = {0,0,0};
    double frontRightAngleOffsets[3] = {0 , PI/2, 0};

    int frontLeftPins[3] = {8,9,10};
    Coordinates frontLeftCoords = {0,0,0};
    double frontLeftAngleOffsets[3] = {0 , PI/2, 0};

    int rearLeftPins[3] = {12,13,14};
    Coordinates rearLeftCoords = {0,0,0};
    double rearLeftAngleOffsets[3] = {0 , PI/2, 0};
};


#endif