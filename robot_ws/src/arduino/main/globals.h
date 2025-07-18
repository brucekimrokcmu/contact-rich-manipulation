// Created on 22/06/2025

// Globals.h

// Copyright RoboVac All Rights Reserved

#ifndef GLOBALS_H
#define GLOBALS_H

#include "parameters.h"

// // in m/s
// // min speed is 0 m/s
// // max speed is 2.015 m/s
// float speed = 0.0;

// // in degrees
// // allowed range is -30 to 30 deg
// int steering_angle = 0;

// ---------------- FSM Timings ----------------
const int FORWARD_TIME = 5000;
const int REVERSE_TIME = 1500;
const unsigned long SPIRAL_DURATION = 30000; // 30 seconds
const float MIN_TURN_ANGLE = TURN_ANGLE;
const float MAX_TURN_ANGLE = 0.0; // straight line

// Typical operating speeds
const float scale = 1.5;
const float FORWARD_SPEED = 0.4 * scale; // m/s
const float TURN_SPEED = 0.4 * scale;    // m/s
const float REVERSE_SPEED = -0.05;       // m/s

#endif // GLOBALS_H