#include <Servo.h>

Servo myservo; // Create servo object to control the servo

int motor1pin1 = 4;
int motor1pin2 = 5;
int motor1en = 6;

int motor2pin1 = 7;
int motor2pin2 = 8;
int motor2en = 9;

const int RELAY_PIN = 13;

void speedControl()
{
    // Turn on motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);

    digitalWrite(RELAY_PIN, HIGH);

    // Accelerate from zero to maximum speed
    for (int i = 0; i < 100; i++)
    {
        analogWrite(motor1en, i);
        analogWrite(motor2en, i);
        delay(20);
    }

    // Decelerate from maximum speed to zero
    for (int i = 100; i >= 0; --i)
    {
        analogWrite(motor1en, i);
        analogWrite(motor2en, i);
        delay(20);
    }

    // Now turn off motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin1, LOW);

    digitalWrite(RELAY_PIN, LOW);
}

void setup()
{
    myservo.attach(3);
    // put your setup code here, to run once:
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor1en, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);
    pinMode(motor2en, OUTPUT);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
}
void set_home_pos()
{
    const int HOMEPOS = 110; // 60 ~ 120
}

void loop()
{
    // put your main code here, to run repeatedly:

    //   delay(3000);
    myservo.write(90);
    speedControl();
    // set_home_pos();
    delay(1000);
}
