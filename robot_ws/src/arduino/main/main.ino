// Created on 22/06/2025

// Main.ino

// Copyright RoboVac All Rights Reserved

#include <Servo.h>

#include "globals.h"
#include "pins.h"
#include "parameters.h"

Servo myservo; // Create servo object to control the servo

void setup()
{
    // put your setup code here, to run once:
    pinMode(MOTOR1PIN1, OUTPUT);
    pinMode(MOTOR1PIN2, OUTPUT);
    pinMode(MOTOR1EN, OUTPUT);
    pinMode(MOTOR2PIN1, OUTPUT);
    pinMode(MOTOR2PIN2, OUTPUT);
    pinMode(MOTOR2EN, OUTPUT);

    myservo.attach(SERVOPIN);
}

// 0 degree -> 90
// 30 degree -> 120
// -30 degree -> 60
// Anything outside this range will be clamped to 60 or 120
void set_steering_angle(int angle)
{
    // Constrain the angle to be within the range of -30 to 30 degrees
    angle = constrain(angle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    // Map the angle to the servo's range (0 to 180 degrees)
    angle += 90; // Shift the range from [-30, 30] to [60, 120]
    // Print the angle for debugging
    // Serial.print("Setting steering angle to: ");
    // Serial.println(angle);
    // Write the angle to the servo
    myservo.write(angle);
}

// Motor speed is controlled by PWM, where 0 = off and 255 = max speed
// Convert speed from m/s to PWM value
// Assuming a linear mapping for simplicity, you can adjust this based on your motor's characteristics
// Motor rpm at 12V is 550 rpm
// Wheel radius is 0.035 m
// Speed in m/s = (rpm * 2 * pi * radius) / 60
// Speed in m/s = (550 * 2 * 3.14159 * 0.035) / 60
// Speed in m/s = 2.015 m/s
void set_speed(int speed)
{
    // Constrain the speed to be within -2.015 to 2.015 m/s
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    // Map the speed to PWM value (0 to 255)
    // Assuming a linear mapping for simplicity, you can adjust this based on your motor's characteristics
    int pwmValue = map(speed, 0, MAX_SPEED, 0, 255);

    // Set direction based on speed sign
    if (speed >= 0)
    {
        // Forward direction
        digitalWrite(MOTOR1PIN1, LOW);
        digitalWrite(MOTOR1PIN2, HIGH);
        digitalWrite(MOTOR2PIN1, LOW);
        digitalWrite(MOTOR2PIN2, HIGH);
    }
    else
    {
        // Reverse direction
        digitalWrite(MOTOR1PIN1, HIGH);
        digitalWrite(MOTOR1PIN2, LOW);
        digitalWrite(MOTOR2PIN1, HIGH);
        digitalWrite(MOTOR2PIN2, LOW);
    }

    // Set the speed of the motors
    analogWrite(MOTOR1EN, speed);
    analogWrite(MOTOR2EN, speed);
}

void loop()
{
    // put your main code here, to run repeatedly:

    set_steering_angle(steering_angle);
    set_speed(speed);

    delay(200);
}
