#include "vacuum.h"
#include "pins.h"
#include <Arduino.h>

void initializeVacuum()
{
    pinMode(RELAYPIN, OUTPUT);
    stopVacuum(); // Ensure vacuum is off initially
}

void startVacuum()
{
    digitalWrite(RELAYPIN, HIGH); // Turn on the vacuum
}

void stopVacuum()
{
    digitalWrite(RELAYPIN, LOW); // Turn off the vacuum
}