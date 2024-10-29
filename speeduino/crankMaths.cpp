#include "globals.h"
#include "crankMaths.h"
#include "decoders.h"

uint32_t angleToTimeMicroSecPerDegree(uint16_t angle) {
  UQ24X8_t micros = (uint32_t)angle * (uint32_t)microsPerDegree;
  return RSHIFT_ROUND(micros, microsPerDegree_Shift);
}

uint16_t timeToAngleDegPerMicroSec(uint32_t time) {
    uint32_t degFixed = time * (uint32_t)degreesPerMicro;
    return RSHIFT_ROUND(degFixed, degreesPerMicro_Shift);
}