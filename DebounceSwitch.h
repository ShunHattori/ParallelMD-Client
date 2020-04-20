#pragma once
#include "Arduino.h"

/*
 *  Anti-chatter read for atmega328 controller
 */

class DebounceSwitch
{
public:
  typedef enum Pinmode
  {
    PULLUP,
    NONE
  } Pinmode;

  DebounceSwitch(uint8_t, Pinmode);
  void update();
  void setDetectCount(int);
  void setButtonState(bool);
  void disableStateUpdate();
  void enableStateUpdate();
  bool stats();

private:
  uint64_t sampledTime;
  int _switchPin, isPinAnalog;
  uint16_t detectFlagNumber, analogDetectValue, samplingPeriod, mode, buttonPressCount;
  bool ButtonStats, isEnableUpdating;
  void analogProcess();
  void digitalProcess();
  void setStateFlag();
};