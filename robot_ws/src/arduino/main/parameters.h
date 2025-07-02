// Created on 22/06/2025

// Parameters.h

// Copyright RoboVac All Rights Reserved

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <Arduino.h> 

// ---------------- Motor Parameters ----------------

// DC motors
// Motor rpm at 12V is 550 rpm
// Wheel radius is 0.035 m
// Speed in m/s = (rpm * 2 * pi * radius) / 60
// Speed in m/s = (550 * 2 * 3.14159 * 0.035) / 60
// Speed in m/s = 2.015 m/s
const float MOTOR_MAX_RPM = 550.0;         // at 12V
const float MOTOR_GEAR_RATIO = 19.0;
const float WHEEL_RADIUS = 0.0325;          // meters
const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const float MAX_SPEED = (MOTOR_MAX_RPM * WHEEL_CIRCUMFERENCE) / 60.0;  // ≈ 1.87 m/s


// ---------------- Servo Parameters ----------------
const int STEERING_ANGLE_MIN = -30;
const int STEERING_ANGLE_MAX = 30;
const int STEERING_ANGLE_CENTER = 0; 

// ---------------- Robot Parameters ----------------
const float WHEELBASE = 0.1651;    
const float TRACK_WIDTH = 0.191;              // TBD: measure it!
const int TURN_ANGLE = 15;                 
const float TURN_ANGLE_RAD = TURN_ANGLE * DEG_TO_RAD;

#endif // PARAMETERS_H