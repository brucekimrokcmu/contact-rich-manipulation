#include <Servo.h>

Servo myservo; // Create servo object to control the servo

int motor1pin1 = 4;
int motor1pin2 = 5;
int motor1en = 6;

int motor2pin1 = 7;
int motor2pin2 = 8;
int motor2en = 9;

void speedControl()
{
    // Turn on motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);

    // Accelerate from zero to maximum speed
    for (int i = 0; i < 256; i++)
    {
        analogWrite(motor1en, i);
        analogWrite(motor2en, i);
        myservo.write(constrain(i, 60, 120));
        delay(20);
    }

    // Decelerate from maximum speed to zero
    for (int i = 255; i >= 0; --i)
    {
        analogWrite(motor1en, i);
        analogWrite(motor2en, i);
        myservo.write(constrain(i, 60, 120));
        delay(20);
    }

    // Now turn off motors
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin1, LOW);
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
}
void set_home_pos()
{
    const int HOMEPOS = 110; // 60 ~ 120
}

void loop()
{
    // put your main code here, to run repeatedly:

    //   delay(3000);
    speedControl();
    // set_home_pos();
    delay(1000);
}
