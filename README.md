# BREED HKU Doggi-E: Quadruped Robot Control System

## Overview
This system implements inverse kinematics control for a quadruped robot with 12 servos (3 per leg). The code controls leg movements and positioning using inverse kinematics calculations to translate desired foot positions into servo angles.

## Key Components

### 1. Main Controller (`main.cpp`)
- Initializes the robot systems
- Controls the main movement loop
- Hardware configuration:
  - Upper leg length: 124mm
  - Lower leg length: 138mm
  - Body width: 10mm
  - Body length: 10mm
- Uses PWM servo control with range 500-2500 microseconds

### 2. Leg Controller (`Leg.cpp`)
Handles the actuation of all four legs with functions:
- `initLeg()`: Initializes servo controllers
- `legControl()`: Controls all legs simultaneously
- `localLegControl()`: Controls individual leg positions
- `writeServo()`: Converts angles to PWM signals

### 3. Inverse Kinematics (`InverseKinematics.cpp`)
Calculates required joint angles for desired foot positions:

#### Key Methods
- `getLocalXYZ()`: Converts global coordinates to local leg coordinates
- `getJointAngles()`: Calculates required joint angles for a given position
- `shoulderJointAngle()`: Calculates shoulder servo angle
- `hipJointAngle()`: Calculates hip servo angle
- `kneeJointAngle()`: Calculates knee servo angle

#### Rotation Methods
- `rotXOffset()`: Calculates leg position offsets for X-axis rotation
- `rotYOffset()`: Calculates leg position offsets for Y-axis rotation
- `rotZOffset()`: Calculates leg position offsets for Z-axis rotation

## Hardware Configuration

### Servo Layout
Each leg has 3 servos:
1. Shoulder servo: Controls leg rotation
2. Hip servo: Controls upper leg position
3. Knee servo: Controls lower leg position

### Leg Configuration
```
Front Left (FL) -- -- Front Right (FR)
      |                    |
      |                    |
Rear Left (RL)  -- -- Rear Right (RR)
```

## Movement Control
The system uses inverse kinematics to:
1. Calculate required joint angles for desired foot positions
2. Convert angles to PWM signals
3. Apply servo commands across all legs

## Operating Parameters
- Servo PWM Frequency: 60Hz
- Minimum pulse length: 500 microseconds
- Maximum pulse length: 2500 microseconds
- Minimum leg length: √(upperLeg² + lowerLeg²)
- Maximum leg length: upperLeg + lowerLeg

Would you like me to explain any specific part of the documentation in more detail?
