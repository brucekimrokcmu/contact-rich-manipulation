
#include "globals.h"
#include "pins.h"
#include "parameters.h"
#include "motor_control.h"
// #include "sensor_control.h"
#include "state_machine.h"
#include "vacuum.h"

void setup()
{
    initializeMotors();
    initializeVacuum();
    // initializeSensors();
    initializeStateMachine();
}

void loop()
{
    updateStateMachine();
    // checkPatternSwitching();
    delay(50);
}