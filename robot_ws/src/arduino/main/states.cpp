#include "robot_states.h"
#include "motor_control.h"
#include "globals.h"
#include "parameters.h"

RobotState currentState = IDLE;
CleaningPattern cleaningMode = SYSTEMATIC_ROWS;
unsigned long stateStartTime = 0;
unsigned long totalRunTime = 0;
unsigned long lastDirectionChange = 0;
unsigned long idleStartTime = 0;
int turnDirection = 1;          // 1 for CW, -1 for CCW
bool bumpLeftDetected = false;
bool bumpRightDetected = false;
bool bumpAnyDetected = false; 
bool robotStarted = false;
unsigned long ARC_TURN_TIME = 0;

void executeIdle()
{
    setSteeringAngle(STEERING_ANGLE_CENTER);
    setSpeed(0.0);
    
    if (millis() - stateStartTime > 5000)
    {
        Serial.println("Auto start vacuum after 5 seconds");
        robotStarted = true;
        changeState(FORWARD);
        return;
    }
}

void executeForward()
{
    setSteeringAngle(STEERING_ANGLE_CENTER);
    setSpeed(FORWARD_SPEED);
    
    if (bumpAnyDetected)
    {
        changeState(BUMP_DETECTED);
        return;
    }

    if (cleaningMode == SYSTEMATIC_ROWS)
    {
        /**
            FORWARD (10s)
            ↓
            REVERSE (1.5s)
            ↓
            TURN_STEER_OUT (e.g., steer left, forward)
            ↓
            TURN_STEER_IN (e.g., steer right, forward)
            ↓
            FORWARD

          repeat
         */

        if (millis() - lastDirectionChange > FORWARD_TIME) // 10 seconds
        {
            Serial.println("Forward time exceeded, reversing");
            changeState(REVERSE);
            return; 
        }
    }
    else if (cleaningMode == SPIRAL_PATTERN)
    {
        // TODO: Implement spiral pattern logic
        if (millis() - lastDirectionChange > FORWARD_TIME)
        {
            Serial.println("Spiral pattern forward time exceeded, switching to spiral out");
            changeState(SPIRAL_OUT);
            return;
        }
    }
    else if (cleaningMode == WALL_FOLLOWING)
    {
        // TODO: Implement wall following logic
    }
    else if (cleaningMode == RANDOM_BOUNCE)
    {
        // TODO: Implement random bounce logic
    }
}

void executeBumpDetected()
{
    setSpeed(0.0);
    delay(500); 
    
    if (bumpLeftDetected && bumpRightDetected)
    {
        Serial.println("Both bump sensors triggered, reversing");
        changeState(REVERSE);
    }
    else if (bumpLeftDetected)
    {
        Serial.println("Left bump sensor triggered, turning right");
        turnDirection = 1;
        changeState(REVERSE);
    }
    else if (bumpRightDetected)
    {
        Serial.println("Right bump sensor triggered, turning left");
        turnDirection = -1;
        changeState(REVERSE);
    }
}

void executeReverse()
{
    setSteeringAngle(STEERING_ANGLE_CENTER);
    setSpeed(REVERSE_SPEED);
    
    if (cleaningMode == SYSTEMATIC_ROWS)
    {
        if (millis() - stateStartTime > REVERSE_TIME)
        {
            Serial.println("Finished reversing. Turning ...");
            changeState(TURN_STEER_OUT);
            return;
        }
    }
    else if (cleaningMode == SPIRAL_PATTERN)
    {
        // TODO: Implement spiral pattern logic
    }
    else if (cleaningMode == WALL_FOLLOWING)
    {
        // TODO: Implement wall following logic
    }
    else if (cleaningMode == RANDOM_BOUNCE)
    {
        // TODO: Implement random bounce logic
    }
}

void executeTurnCW()
{
    // TODO: Implement clockwise turn logic
}

void executeTurnCCW()
{
    // TODO: Implement counter-clockwise turn logic
}

void executeTurnSteerOut()
{
    currentSteeringAngle = STEERING_ANGLE_CENTER + TURN_ANGLE * turnDirection;
    currentSteeringAngle = constrain(currentSteeringAngle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    setSteeringAngle(currentSteeringAngle);
    setSpeed(TURN_SPEED);

    if (millis() - stateStartTime > ARC_TURN_TIME)
    {
        Serial.println("Finished steering out. Steering in ...");
        changeState(TURN_STEER_IN);
        return;
    }
}

void executeTurnSteerIn()
{
    currentSteeringAngle = STEERING_ANGLE_CENTER - TURN_ANGLE * turnDirection;
    currentSteeringAngle = constrain(currentSteeringAngle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    setSteeringAngle(currentSteeringAngle);
    setSpeed(TURN_SPEED);

    if (millis() - stateStartTime > ARC_TURN_TIME)
    {
        Serial.println("Finished steering in. Forward ...");
        turnDirection = -turnDirection; 
        lastDirectionChange = millis();
        changeState(FORWARD);
        return;
    }
}

void executeSpiralOut()
{
    // TODO: Implement spiral out logic
    if (spiralStartTime == 0)
    {
        spiralStartTime = millis();
    }
    
    unsigned long elapsedTime = millis() - spiralStartTime;
    float t = constrain(elapsedTime / 1000.0, 0.0, SPIRAL_DURATION / 1000.0);

    float currentAngle = STEERING_ANGLE_CENTER + (MIN_TURN_ANGLE + (MAX_TURN_ANGLE - MIN_TURN_ANGLE) * t) * turnDirection;
    setSteeringAngle(currentAngle);
    setSpeed(TURN_SPEED);
}

void executeWallFollow()
{
    // TODO: Implement wall follow logic
}

void executeEdgeTrace()
{
    // TODO: Implement edge trace logic
}

void executeRandomWalk()
{
    // TODO: Implement random walk logic
}

void switchCleaningPattern()
{
    // TODO: Implement pattern switching logic
}

String getStateName(RobotState state)
{
    switch (state)
    {
    case IDLE: 
        return "IDLE";
    case FORWARD: 
        return "FORWARD";
    case BUMP_DETECTED: 
        return "BUMP_DETECTED";
    case REVERSE:
        return "REVERSE";
    case TURNCW:
        return "TURN_CW";
    case TURNCCW:
        return "TURN_CCW";
    case TURN_STEER_OUT:
        return "TURN_STEER_OUT";
    case TURN_STEER_IN:
        return "TURN_STEER_IN";
    case SPIRAL_OUT:
        return "SPIRAL_OUT";
    case WALL_FOLLOW:
        return "WALL_FOLLOW";
    case EDGE_TRACE:
        return "EDGE_TRACE";    
    case RANDOM_WALK:
        return "RANDOM_WALK";
    default:
        return "UNKNOWN_STATE";
    }
}

void changeState(RobotState newState)
{
    if (newState != currentState)
    {
        Serial.print("State change: ");
        Serial.print(getStateName(currentState));
        Serial.print(" -> ");
        Serial.println(getStateName(newState));
        currentState = newState;
        stateStartTime = millis();
    }
}