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
  uint16_t toothCurrentCount_;  // The current number of teeth (Once sync has
                                // been achieved, this can never actually be 0
  unsigned long triggerFilterTime_ =
      0;  // The shortest time (in uS) that pulses will be accepted (Used for
          // debounce filtering)

  // Trigger filter time is the shortest possible time (in uS) that
  // there can be between crank teeth (ie at max RPM). Any pulses that
  // occur faster than this time will be discarded as noise. This is
  // simply a default value, the actual values are set in the setup()
  // functions of each decoder

  unsigned long triggerSecFilterTime_;  // The shortest time (in uS) that pulses
  // will be accepted (Used for debounce
  // filtering) for the secondary input
  uint8_t decoderState_;
};