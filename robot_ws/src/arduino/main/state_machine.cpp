#include "state_machine.h"
#include "states.h"
// #include "sensor_control.h"
#include "motor_control.h"
#include "parameters.h"

void initializeStateMachine()
{
    ARC_TURN_TIME = computeTurnDuration(
        WHEELBASE,
        TURN_ANGLE,         // in degrees
        PI,                 // radians, e.g., 180° turn
        TURN_SPEED          // m/s
    );
    
    Serial.print("Computed ARC_TURN_TIME (ms): ");
    Serial.println(ARC_TURN_TIME);

    currentState = IDLE;
    cleaningMode = SYSTEMATIC_ROWS;

    stateStartTime = millis();
    idleStartTime = millis();
    totalRunTime = millis();

    Serial.println("State: IDLE. Wait 5 seconds for vacuum start");
    Serial.println("Cleaning Mode: Systematic Rows");
}

void updateStateMachine()
{
    // readSensors();

    switch(currentState) 
    {
        case IDLE:
            executeIdle();
            break;
        case FORWARD:
            executeForward();
            break;
        case BUMP_DETECTED:
            executeBumpDetected();
            break;
        case REVERSE:
            executeReverse();
            break;
        case TURNCW:
            executeTurnCW();            
            break;
        case TURNCCW:     
            executeTurnCCW();
            break;
        case TURN_STEER_OUT:
            executeTurnSteerOut();
            break;
        case TURN_STEER_IN:
            executeTurnSteerIn();
            break;
        case SPIRAL_OUT:
            executeSpiralOut();
            break;
        case WALL_FOLLOW:
            executeWallFollow();        
            break;
        case EDGE_TRACE:
            executeEdgeTrace();
            break;
        case RANDOM_WALK:
            executeRandomWalk();
            break; 
        default:
            Serial.println("Unknown state detected!");
            changeState(IDLE);
            break;
    }
}

void checkPatternSwitching()
{
    // Switch cleaning pattern after 1 minute of active operation
    if (currentState != IDLE && millis() - totalRunTime > 60000) // 1 minute
    {
        switchCleaningPattern();
        totalRunTime = millis();
    }
}