// DC motor 1
const int MOTOR1PIN1 = 4;
const int MOTOR1PIN2 = 5;
const int MOTOR1EN = 6;

// DC motor 2
const int MOTOR2PIN1 = 7;
const int MOTOR2PIN2 = 8;
const int MOTOR2EN = 9;

// DC motors
// Motor rpm at 12V is 550 rpm
// Wheel radius is 0.035 m
// Speed in m/s = (rpm * 2 * pi * radius) / 60
// Speed in m/s = (550 * 2 * 3.14159 * 0.035) / 60
// Speed in m/s = 2.015 m/s
const float MOTOR_MAX_RPM = 550.0; // at 12V
const float MOTOR_GEAR_RATIO = 19.0;
const float WHEEL_RADIUS = 0.0325; // meters
const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const float MAX_SPEED = (MOTOR_MAX_RPM * WHEEL_CIRCUMFERENCE) / 60.0; // ≈ 1.87 m/s

void setup()
{
    // put your setup code here, to run once:
    pinMode(MOTOR1PIN1, OUTPUT);
    pinMode(MOTOR1PIN2, OUTPUT);
    pinMode(MOTOR1EN, OUTPUT);
    pinMode(MOTOR2PIN1, OUTPUT);
    pinMode(MOTOR2PIN2, OUTPUT);
    pinMode(MOTOR2EN, OUTPUT);
}

// Motor speed is controlled by PWM, where 0 = off and 255 = max speed
// Convert speed from m/s to PWM value
void set_speed(float speed)
{
    // Constrain the speed to be within -2.015 to 2.015 m/s
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    // Map the speed to PWM value (0 to 255)
    // Assuming a linear mapping for simplicity, you can adjust this based on your motor's characteristics
    int pwmValue = map(abs(speed) * 1000, 0, MAX_SPEED * 1000, 0, 255);

    // Set direction based on speed sign
    if (speed >= 0)
    {
        // Forward direction
        digitalWrite(MOTOR1PIN1, LOW);
        digitalWrite(MOTOR1PIN2, HIGH);
        digitalWrite(MOTOR2PIN1, LOW);
        digitalWrite(MOTOR2PIN2, HIGH);
    }
    else
    {
        // Reverse direction
        digitalWrite(MOTOR1PIN1, HIGH);
        digitalWrite(MOTOR1PIN2, LOW);
        digitalWrite(MOTOR2PIN1, HIGH);
        digitalWrite(MOTOR2PIN2, LOW);
    }

    // Set the speed of the motors
    analogWrite(MOTOR1EN, 85);
    analogWrite(MOTOR2EN, 70);
}

void loop()
{
    // put your main code here, to run repeatedly:

    set_speed(0.4);

    delay(200);
}
