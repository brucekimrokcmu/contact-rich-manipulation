
#include "globals.h"
#include "pins.h"
#include "parameters.h"
#include "motor_control.h"
#include "sensor_control.h"
#include "state_machine.h"

void setup()
{
    Serial.begin(9600);
    Serial.println("Robot Vacuum Starting...");
    
    initializeMotors();
    initializeSensors();
    initializeStateMachine();
    
    Serial.println("Initialization complete!");
}

void loop()
{
    updateStateMachine();
    checkPatternSwitching();
    delay(50);
}