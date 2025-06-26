#include <Servo.h>
#include "globals.h"
#include "pins.h"
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

Servo steeringServo;
RobotState currentState = IDLE;
CleaningPattern cleaningMode = SYSTEMATIC_ROWS;

unsigned long stateStartTime = 0.0;
unsigned long totalRunTime = 0.0;
unsigned long lastDirectionChange = 0;
unsigned long idleStartTime = 0.0;

int currentSteeringAngle = STEERING_ANGLE_CENTER;
int turnDirection = 1;          // 1 for CW, -1 for CCW
bool bumpLeftDetected = false;
bool bumpRightDetected = false;
bool bumpAnyDetected = false; 
bool robotStarted = false;
unsigned long ARC_TURN_TIME = 0;

/*
 * @brief Compute time in milliseconds required to complete a turn arc.
 * 
 * @param wheelbase          Distance between front and rear axles (meters)
 * @param steeringAngleDeg   Front wheel steering angle (degrees)
 * @param turnArcAngleRad    Desired arc to sweep in radians (e.g. PI for 180°)
 * @param speed              Vehicle speed during turn (m/s)
 * @return unsigned long     Time in milliseconds needed to complete the turn
 */
unsigned long computeTurnDuration(float wheelbase,
                                  float steeringAngleDeg,
                                  float turnArcAngleRad,
                                  float speed)
{
    // Convert steering angle to radians
    float steeringAngleRad = steeringAngleDeg * DEG_TO_RAD;

    // Prevent division by zero or invalid input
    if (abs(tan(steeringAngleRad)) < 1e-3 || speed <= 0.0) {
        return 0;
    }

    // Calculate turning radius from Ackermann geometry
    float turnRadius = wheelbase / tan(steeringAngleRad); // meters

    // Calculate arc length the robot must travel
    float arcLength = turnRadius * turnArcAngleRad;       // meters

    // Compute time required to traverse that arc at given speed
    float timeSeconds = arcLength / speed;

    // Return as milliseconds
    return (unsigned long)(timeSeconds * 1000.0);
}


void setSteeringAngle(int angle)
{
    angle = constrain(angle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    angle += 90; // Shift the range from [-30, 30] to [60, 120]
    steeringServo.write(angle);
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

void setup()
{

    Serial.begin(9600);
    
    pinMode(MOTOR1PIN1, OUTPUT);
    pinMode(MOTOR1PIN2, OUTPUT);
    pinMode(MOTOR1EN, OUTPUT);
    pinMode(MOTOR2PIN1, OUTPUT);
    pinMode(MOTOR2PIN2, OUTPUT);
    pinMode(MOTOR2EN, OUTPUT);

    steeringServo.attach(SERVOPIN);
    setSteeringAngle(STEERING_ANGLE_CENTER); // Center the steering servo
    setSpeed(0.0);

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

    Serial.println("State: IDLE. wait 5 seconds for vacuum start");
    Serial.println("Cleaning Mode: Systematic Rows");
}

void loop()
{
    // TODO: read bump sensors bumpDetected = !digitalRead(BUMP_SENSOR_PIN)
    bumpLeftDetected = !digitalRead(BUMPLEFTPIN);
    bumpRightDetected = !digitalRead(BUMPRIGHTPIN);
    bumpAnyDetected = bumpLeftDetected || bumpRightDetected;

    switch(currentState) 
    {
        case IDLE:
            executeIdle();
            break;
        case FORWARD;
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
            break;
    }

    if (currentState != IDLE && millis() - totalRunTime > 60000) // 1 minute
    {
        switchCleaningPattern();
        totalRunTime = millis();
    }

    delay(50);
}

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
    setSteergingAngle(STEERING_ANGLE_CENTER);
    setSpeed(FORWARD_SPEED);
    if (bumpDetected)
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
     
    }
    else if (cleaningMode == WALL_FOLLOWING)
    {
     
    }
    else if (cleaningMode == RANDOM_BOUNCE)
    {
     
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

void exectuteReverse()
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
     
    }
    else if (cleaningMode == WALL_FOLLOWING)
    {
     
    }
    else if (cleaningMode == RANDOM_BOUNCE)
    {
     
    }
}

void executeTurnSteerOut()
{

    currentSteeringAngle = STEERING_ANGLE_CENTER + TURN_ANGLE * turnDirection;
    currentSteeringAngle = constrain(currentSteeringAngle, STEERING_ANGLE_MIN, STEERING_ANGLE_MAX);
    setSteeringAngle(currentSteeringAngle);
    setSpeed(TURN_SPEED);

    if (millis() - stateStartTime > ARC_TURN_TIME) // 1 second
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

    if (millis() - stateStartTime > ARC_TURN_TIME) // 1 second
    {
        Serial.println("Finished steering in. Forward ...");
        turnDirection = -turnDirection; 
        lastDirectionChange = millis();
        changeState(FORWARD);
        return;
    }
}

void switchCleaningPattern()
{
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
