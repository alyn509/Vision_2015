#include "VisionSensor.h"

void VisionSensor::initPin(int sensorPin)
{
  this->sensorPin = sensorPin;
  enabled = true;
  pullup = false;
  pinMode(sensorPin, INPUT);
}

void VisionSensor::setAsPullup()
{
  pullup = true;
  pinMode(sensorPin, INPUT_PULLUP);
}

boolean VisionSensor::detect()
{
  return enabled && (digitalRead(sensorPin) ^ pullup);
}

void VisionSensor::enable()
{
  enabled = true;
}

void VisionSensor::disable()
{
  enabled = false;
}

boolean VisionSensor::isEnabled()
{
  return enabled;
}
