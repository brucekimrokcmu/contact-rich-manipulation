// Created on 22/06/2025

// Parameters.h

// Copyright RoboVac All Rights Reserved

#ifndef PARAMETERS_H
#define PARAMETERS_H

// DC motors
// Motor rpm at 12V is 550 rpm
// Wheel radius is 0.035 m
// Speed in m/s = (rpm * 2 * pi * radius) / 60
// Speed in m/s = (550 * 2 * 3.14159 * 0.035) / 60
// Speed in m/s = 2.015 m/s
const float MAX_SPEED = 2.015; // m/s

// Servo motors
const int STEERING_ANGLE_MIN = -30;
const int STEERING_ANGLE_MAX = 30;

#endif // PINS_H