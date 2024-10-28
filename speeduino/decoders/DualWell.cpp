#include "DualWell.h"

constexpr auto kCamSpeed = 1;

/** Dual wheels - 2 wheels located either both on the crank or with the primary
on the crank and the secondary on the cam. Note: There can be no missing teeth
on the primary wheel.
* @defgroup dec_dual Dual wheels
* @{
*/
/** Dual Wheel Setup.
 *
 * */
auto DualWell::triggerSetup() -> void {
  triggerToothAngle_ = 360 / configPage4.triggerTeeth;  // The number of
  // degrees that passes from tooth to tooth
  if (configPage4.TrigSpeed == kCamSpeed) {
    triggerToothAngle_ = 720 / configPage4.triggerTeeth;
  }  // Account for cam speed
  toothCurrentCount = 255;  // Default value
  triggerFilterTime =
      (MICROS_PER_SEC /
       (MAX_RPM / 60U *
        configPage4.triggerTeeth));  // Trigger filter time is the shortest
  // possible time(in uS) that there can be between crank teeth(ie at max RPM)
  //.Any pulses that occur faster than this time will be discarded
  //   as noise
  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  // Same as above, but fixed at 2 teeth on the secondary input and divided
  // by 2 (for cam speed)
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);  // This is always
  // true for this pattern
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / 50U) * triggerToothAngle);
  // Minimum 50rpm. (3333uS is the time per degree at 50rpm)
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
};

auto DualWell::triggerHandler() -> void {};

auto DualWell::triggerSecondaryHandler() -> void {};

auto DualWell::triggerTertiaryHandler() -> void {};

auto DualWell::getRPM() -> uint16_t { return 0; };

auto DualWell::getCrankAngle() -> int { return 0; };

auto DualWell::triggerSetEndTeeth() -> void {};