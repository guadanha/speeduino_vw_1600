#pragma once

#include <stdint.h>

class DecodersBase {
 public:

  virtual void triggerSetup() = 0;

  virtual void triggerHandler() = 0;

  virtual void triggerSecondaryHandler() = 0;

  virtual void triggerTertiaryHandler() = 0;

  virtual uint16_t getRPM() = 0;

  virtual int getCrankAngle() = 0;

  virtual void triggerSetEndTeeth() = 0;

  uint16_t triggerToothAngle_;
};