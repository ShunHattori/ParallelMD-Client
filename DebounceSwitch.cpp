#include "DebounceSwitch.h"

DebounceSwitch::DebounceSwitch(uint8_t inputPin, Pinmode pinmode) : sampledTime(millis()),
                                                                    _switchPin(inputPin),
                                                                    isPinAnalog(0),
                                                                    detectFlagNumber(3),
                                                                    analogDetectValue(900), //10 bit
                                                                    samplingPeriod(5),
                                                                    mode(pinmode),
                                                                    ButtonStats(0),
                                                                    isEnableUpdating(1)
{
  if (A0 <= inputPin && inputPin <= A7) //analogPin
    isPinAnalog = 1;
  pinMode(inputPin, INPUT);
  return;

  if (mode == PULLUP) //digitalPin
  {
    pinMode(inputPin, INPUT_PULLUP);
  }
  else
  {
    pinMode(inputPin, INPUT);
  }
}

void DebounceSwitch::update()
{
  if (!isEnableUpdating)
    return;

  if (isPinAnalog)
    analogProcess();
  else
    digitalProcess();

  setStateFlag();
  return;
}

void DebounceSwitch::analogProcess()
{
  if (samplingPeriod < (millis() - sampledTime))
  {
    if (uint16_t(analogRead(_switchPin)) > analogDetectValue)
    {
      buttonPressCount++;
    }
    else
      buttonPressCount = 0;
    sampledTime = millis();
  }
}

void DebounceSwitch::digitalProcess()
{
  if (samplingPeriod < (millis() - sampledTime))
  {
    switch (mode)
    {
    case PULLUP:
      if (!digitalRead(_switchPin))
      {
        buttonPressCount++;
      }
      else
        buttonPressCount = 0;
      break;

    default:
      if (digitalRead(_switchPin))
      {
        buttonPressCount++;
      }
      else
        buttonPressCount = 0;
      break;
    }
    sampledTime = millis();
  }
}

void DebounceSwitch::setStateFlag()
{
  if (detectFlagNumber < buttonPressCount)
  {
    ButtonStats = 1;
  }
  else
  {
    ButtonStats = 0;
  }
}

void DebounceSwitch::setDetectCount(int count)
{
  detectFlagNumber = count;
}
void DebounceSwitch::setButtonState(bool state)
{
  ButtonStats = state;
}
void DebounceSwitch::disableStateUpdate()
{
  isEnableUpdating = 0;
}
void DebounceSwitch::enableStateUpdate()
{
  isEnableUpdating = 1;
}
bool DebounceSwitch::stats()
{
  return ButtonStats;
}
