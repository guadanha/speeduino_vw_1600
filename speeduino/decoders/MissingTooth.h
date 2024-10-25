#pragma once

#include "DecodersBase.h"

class MissingTooth : public DecodersBase {
public:

  auto triggerHandler() -> void override;


  auto triggerSecondaryHandler() -> void override;

  auto triggerTertiaryHandler() -> void override;

  auto getRPM() -> uint16_t override;

  auto getCrankAngle()  -> int override;

  auto triggerSetEndTeeth() -> void override;

};