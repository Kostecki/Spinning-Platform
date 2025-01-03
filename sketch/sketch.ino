#include <Stepper.h>

#define DEBUG 0 // Set to 0 to disable debug prints

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define SERIAL_BEGIN() Serial.begin(9600)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define SERIAL_BEGIN()
#endif

#define STEPS 2048

Stepper stepper(STEPS, D10, D8, D9, D7);

const int minSpeed = 2;
const int maxSpeed = 15;
const int stepsPerUpdate = STEPS / 100;

const int lowThreshold = 20;
const int highThreshold = 1023;

const int numReadings = 10;

void setup()
{
  SERIAL_BEGIN();
  DEBUG_PRINTLN("Stepper Motor Control Started");

  stepper.setSpeed(minSpeed);
  stepper.step(stepsPerUpdate);
}

void loop()
{
  static unsigned long lastReadTime = 0;
  const unsigned long readInterval = 50; // milliseconds

  if ((millis() - lastReadTime) > readInterval)
  {
    lastReadTime = millis();

    long sum = 0;

    for (int i = 0; i < numReadings; i++)
    {
      sum += analogRead(A0);
    }

    int sensorValue = sum / numReadings;

    sensorValue = constrain(sensorValue, 0, 1023);

    int motorSpeed = map(sensorValue, lowThreshold, highThreshold, minSpeed, maxSpeed);
    motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);

    stepper.setSpeed(motorSpeed);

    DEBUG_PRINT("Potentiometer Value: ");
    DEBUG_PRINT(sensorValue);
    DEBUG_PRINT(" | Motor Speed: ");
    DEBUG_PRINTLN(motorSpeed);

    stepper.step(stepsPerUpdate);
  }
}
