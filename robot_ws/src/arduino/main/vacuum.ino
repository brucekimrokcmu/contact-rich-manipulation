#include <Servo.h>
#include "globals.h"
#include "pins.h"
#include "parameters.h"

enum RobotState
{
    IDLE,
    BUMP_DETECETED,
    FORWARD,
    REVERSE,
    TURNCW,
    TURNCCW
}; // TODO: SPIRAL_OUT, WALL_FOLLOW, EDGE_TRACE, RANDOM_WALK

enum CleaningPattern
{
    SYSTEMATIC_ROWS,
    SPIRAL_PATTERN,
    WALL_FOLLOWING,
    RANDOM_BOUNCE
};

Servo steeringServo;
RobotState currentState = FORWARD;
CleaningPattern cleaningMode = SYSTEMATIC_ROWS;


void setup()
{

}

void loop()
{
    // read sensors
    // bumpDetected = !digitalRead(BUMP_SENSOR_PIN)
    // ...

    switch(currentState) 
    {
        case FORWARD;
            executeForward();
            break;



        default:
            break;
    }
    
}

void executeForward()
{

}

void switchCleaningPattern()
{
}

void changeState(RobotState newState)
{

}

void setSteering(int angle)
{
}

void setSpeed(int left, int right)
{
}