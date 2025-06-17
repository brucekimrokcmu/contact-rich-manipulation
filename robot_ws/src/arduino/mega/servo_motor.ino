#include <Servo.h>

Servo myservo; 

void setup()
{
    Serial.begin(9600); // Start serial communication for debug
    myservo.attach(5);  // Attach the servo to digital pin 5
    Serial.println("Servo test started.");
}

void servo_contro()
{
    for (int pos = 0; pos < 270; pos++)
    {
        int angle = constrain(int(pos / 1.5), 0, 180); 
        myservo.write(angle);
        delay(20);
    }

    for (int pos = 270; pos > 0; pos--)
    {
        int angle = constrain(int(pos / 1.5), 0, 180);
        myservo.write(angle);
        delay(20);
    }
}

void loop()
{
    delay(500);   
    servo_contro(); 
    delay(1000);   
}
