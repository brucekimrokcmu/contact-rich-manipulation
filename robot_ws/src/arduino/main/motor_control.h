#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Servo.h>
extern Servo steeringServo;
extern int currentSteeringAngle;

void intializeMotors();
void setSteeringAngle(int angle);
void setSpeed(float speed);

unsigned long computeTurnDuration(float wheelbase, 
                                float steeringAngleDeg, 
                                float turnArcAngleRad, 
                                float speed
                            )

#endif // MOTOR_CONTROL_H