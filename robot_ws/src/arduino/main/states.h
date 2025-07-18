#ifndef ROBOT_STATES_H
#define ROBOT_STATES_H

#include <Arduino.h>
#include "motor_control.h"
#include "globals.h"
#include "parameters.h"

enum RobotState
{
    IDLE,
    BUMP_DETECTED,
    FORWARD,
    REVERSE,
    TURNCW,
    TURNCCW,
    TURN_STEER_IN, 
    TURN_STEER_OUT,
    SPIRAL_OUT, 
    WALL_FOLLOW, 
    EDGE_TRACE, 
    RANDOM_WALK
};

enum CleaningPattern
{
    SYSTEMATIC_ROWS,
    SPIRAL_PATTERN,
    WALL_FOLLOWING,
    RANDOM_BOUNCE
};

extern RobotState currentState;
extern CleaningPattern cleaningMode;
extern unsigned long stateStartTime;
extern unsigned long totalRunTime;
extern unsigned long lastDirectionChange;
extern unsigned long idleStartTime;
extern unsigned long sprialStartTime;
extern int turnDirection;
extern bool bumpLeftDetected;
extern bool bumpRightDetected;
extern bool bumpAnyDetected;
extern bool robotStarted;
extern unsigned long ARC_TURN_TIME;

void changeState(RobotState newState);
String getStateName(RobotState state);
void switchCleaningPattern();

void executeIdle();
void executeForward();
void executeBumpDetected();
void executeReverse();
void executeTurnCW();
void executeTurnCCW();
void executeTurnSteerOut();
void executeTurnSteerIn();
void executeSpiralOut();
void executeWallFollow();
void executeEdgeTrace();
void executeRandomWalk();

#endif