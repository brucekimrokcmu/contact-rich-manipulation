const int RELAY_PIN = 13;
const unsigned long DELAY_TIME_MS = 5000;

void setup()
{
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
}

void loop()
{
    digitalWrite(RELAY_PIN, HIGH);
    delay(DELAY_TIME_MS);
    digitalWrite(RELAY_PIN, LOW);
    delay(DELAY_TIME_MS);
}