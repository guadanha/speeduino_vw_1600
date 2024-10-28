#pragma once

#include "DecodersBase.h"

class DualWell : public DecodersBase {
public:

  auto triggerSetup() -> void override;

  auto triggerHandler() -> void override;

  auto triggerSecondaryHandler() -> void override;

  auto triggerTertiaryHandler() -> void override;

  auto getRPM() -> uint16_t override;

  auto getCrankAngle()  -> int override;

  auto triggerSetEndTeeth() -> void override;

};