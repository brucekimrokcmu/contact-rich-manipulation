#include "motor_control.h"
#include "globals.h"
#include "pins.h"
#include "parameters.h"

Servo steeringServo;
int currentSteeringAngle = STEERING_ANGLE_CENTER;

void initializeMotors()
{
    pinMode(MOTOR1PIN1, OUTPUT);
    pinMode(MOTOR1PIN2, OUTPUT);
    pinMode(MOTOR1EN, OUTPUT);
    pinMode(MOTOR2PIN1, OUTPUT);
    pinMode(MOTOR2PIN2, OUTPUT);
    pinMode(MOTOR2EN, OUTPUT);

    steeringServo.attach(SERVOPIN);
    setSteeringAngle(STEERING_ANGLE_CENTER); // Center the steering servo
    setSpeed(0.0);
}

void setSteeringAngle(int angle)
{
    angle = constrain(angle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    angle += 90; // Shift the range from [-30, 30] to [60, 120]
    steeringServo.write(angle);
    currentSteeringAngle = angle - 90; // Store the original angle
}

void setSpeed(float speed)
{
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    int pwmValue = map(abs(speed) * 1000, 0, MAX_SPEED * 1000, 0, 255);
    
    if (speed >= 0)
    {
        digitalWrite(MOTOR1PIN1, LOW);
        digitalWrite(MOTOR1PIN2, HIGH);
        digitalWrite(MOTOR2PIN1, LOW);
        digitalWrite(MOTOR2PIN2, HIGH);
    }
    else
    {
        digitalWrite(MOTOR1PIN1, HIGH);
        digitalWrite(MOTOR1PIN2, LOW);
        digitalWrite(MOTOR2PIN1, HIGH);
        digitalWrite(MOTOR2PIN2, LOW);
    }
    
    analogWrite(MOTOR1EN, pwmValue);
    analogWrite(MOTOR2EN, pwmValue);
}

unsigned long computeTurnDuration(float wheelbase,
                                  float steeringAngleDeg,
                                  float turnArcAngleRad,
                                  float speed)
{
    float steeringAngleRad = steeringAngleDeg * DEG_TO_RAD;
    if (abs(tan(steeringAngleRad)) < 1e-3 || speed <= 0.0) {
        return 0;
    }
    float turnRadius = wheelbase / tan(steeringAngleRad);
    float arcLength = turnRadius * turnArcAngleRad;      
    float timeSeconds = arcLength / speed;
    
    return (unsigned long)(timeSeconds * 1000.0);
}