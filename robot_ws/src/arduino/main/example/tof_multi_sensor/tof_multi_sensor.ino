// Example sketch for using multiple VL53L0X time-of-flight distance sensors

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

#define SENSOR1_ADDRESS 0x31
#define SENSOR2_ADDRESS 0x32

// set the pins to shutdown
#define SHT_SENSOR1 7
#define SHT_SENSOR2 6

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

// #define HIGH_SPEED
#define HIGH_ACCURACY

void setID()
{

  // Turn on sensor 1
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, LOW);

  // Init sensor 1
  if (!sensor1.init())
  {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1)
    {
    }
  }
  sensor1.setAddress(SENSOR1_ADDRESS);

  // Turn off sensor 1 and turn on sensor 2
  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, HIGH);

  // Init sensor 2
  if (!sensor2.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1)
    {
    }
  }
  sensor2.setAddress(SENSOR2_ADDRESS);

  // Turn on both sensors
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, HIGH);
}

void setup()
{

  pinMode(SHT_SENSOR1, OUTPUT);
  pinMode(SHT_SENSOR2, OUTPUT);

  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, LOW);

  Serial.begin(9600);
  Wire.begin();

  setID();

  sensor1.setTimeout(2000);
  sensor2.setTimeout(2000);

  // increase timing budget to 200 ms
  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
}

void loop()
{
  Serial.print1(sensor1.getAddress());
  Serial.print(" | ");
  Serial.print(sensor2.getAddress());
  Serial.print(" | ");
  Serial.print(sensor1.readRangeSingleMillimeters());
  Serial.print(" | ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor1.timeoutOccurred())
  {
    Serial.print(" TIMEOUT 1");
  }
  if (sensor2.timeoutOccurred())
  {
    Serial.print(" TIMEOUT 2");
  }

  Serial.println();
  delay(500);
}
