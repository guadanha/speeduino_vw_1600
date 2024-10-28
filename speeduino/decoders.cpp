#define MJR 1

/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 * 
 * Crank and Cam decoders
 * 
 * This file contains the various crank and cam wheel decoder functions.
 * Each decoder must have the following 4 functions (Where xxxx is the decoder name):
 * 
 * - **triggerSetup_xxxx** - Called once from within setup() and configures any required variables
 * - **triggerPri_xxxx** - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **triggerSec_xxxx** - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **getRPM_xxxx** - Returns the current RPM, as calculated by the decoder
 * - **getCrankAngle_xxxx** - Returns the current crank angle, as calculated by the decoder
 * - **getCamAngle_xxxx** - Returns the current CAM angle, as calculated by the decoder
 *
 * Each decoder must utilise at least the following variables:
 * 
 * - toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
 */

/* Notes on Doxygen Groups/Modules documentation style:
 * - Installing doxygen (e.g. Ubuntu) via pkg mgr: sudo apt-get install doxygen graphviz
 * - @defgroup tag name/description becomes the short name on (Doxygen) "Modules" page
 * - Relying on JAVADOC_AUTOBRIEF (in Doxyfile, essentially automatic @brief), the first sentence (ending with period) becomes
 *   the longer description (second column following name) on (Doxygen) "Modules" page (old Desc: ... could be this sentence)
 * - All the content after first sentence (like old Note:...) is visible on the page linked from the name (1st col) on "Modules" page
 * - To group all decoders together add 1) @defgroup dec Decoders (on top) and 2) "@ingroup dec" to each decoder (under @defgroup)
 * - To compare Speeduino Doxyfile to default config, do: `doxygen -g Doxyfile.default ; diff Doxyfile.default Doxyfile`
 */
#include <limits.h>
#include "globals.h"
#include "decoders.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "crankMaths.h"
#include "timers.h"
#include "schedule_calcs.h"

void nullTriggerHandler (void){return;} //initialisation function for triggerhandlers, does exactly nothing
uint16_t nullGetRPM(void){return 0;} //initialisation function for getRpm, returns safe value of 0
int nullGetCrankAngle(void){return 0;} //initialisation function for getCrankAngle, returns safe value of 0

void (*triggerHandler)(void) = nullTriggerHandler; ///Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondaryHandler)(void) = nullTriggerHandler; ///Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
void (*triggerTertiaryHandler)(void) = nullTriggerHandler; ///Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(void) = nullGetRPM; ///Pointer to the getRPM function (Gets pointed to the relevant decoder)
int (*getCrankAngle)(void) = nullGetCrankAngle; ///Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(void) = 0; ///Pointer to the triggerSetEndTeeth function of each decoder

static inline void triggerRecordVVT1Angle (void);

volatile unsigned long curTime;
volatile unsigned long curGap;
volatile unsigned long curTime2;
volatile unsigned long curGap2;
volatile unsigned long curTime3;
volatile unsigned long curGap3;
volatile unsigned long lastGap;
volatile unsigned long targetGap;

unsigned long MAX_STALL_TIME = MICROS_PER_SEC/2U; //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.
volatile uint16_t toothCurrentCount = 0; //The current number of teeth (Once sync has been achieved, this can never actually be 0
volatile byte toothSystemCount = 0; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
volatile unsigned long toothSystemLastToothTime = 0; //As below, but used for decoders where not every tooth count is used for calculation
volatile unsigned long toothLastToothTime = 0; //The time (micros()) that the last tooth was registered
volatile unsigned long toothLastSecToothTime = 0; //The time (micros()) that the last tooth was registered on the secondary input
volatile unsigned long toothLastThirdToothTime = 0; //The time (micros()) that the last tooth was registered on the second cam input
volatile unsigned long toothLastMinusOneToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered
volatile unsigned long toothLastMinusOneSecToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile unsigned long toothLastToothRisingTime = 0; //The time (micros()) that the last tooth rose (used by special decoders to determine missing teeth polarity)
volatile unsigned long toothLastSecToothRisingTime = 0; //The time (micros()) that the last tooth rose on the secondary input (used by special decoders to determine missing teeth polarity)
volatile unsigned long targetGap2;
volatile unsigned long targetGap3;
volatile unsigned long toothOneTime = 0; //The time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneMinusOneTime = 0; //The 2nd to last time (micros()) that tooth 1 last triggered
volatile bool revolutionOne = 0; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)
volatile bool revolutionLastOne = 0; // used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.

volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
volatile unsigned int secondaryLastToothCount = 0; // used to identify in the rover pattern which has a non unique primary trigger something unique - has the secondary tooth changed.
volatile unsigned long secondaryLastToothTime = 0; //The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long secondaryLastToothTime1 = 0; //The time (micros()) that the last tooth was registered (Cam input)

volatile unsigned int thirdToothCount; //Used for identifying the current third (Usually exhaust cam - used for VVT2) tooth for patterns with multiple secondary teeth
volatile unsigned long thirdLastToothTime = 0; //The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long thirdLastToothTime1 = 0; //The time (micros()) that the last tooth was registered (Cam input)

uint16_t triggerActualTeeth;
volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile unsigned long triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
volatile unsigned long triggerThirdFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the Third input

volatile uint8_t decoderState = 0;

UQ24X8_t microsPerDegree;
UQ1X15_t degreesPerMicro;

unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
unsigned long elapsedTime;
unsigned long lastCrankAngleCalc;
unsigned long lastVVTtime; //The time between the vvt reference pulse and the last crank pulse

uint16_t ignition1EndTooth = 0;
uint16_t ignition2EndTooth = 0;
uint16_t ignition3EndTooth = 0;
uint16_t ignition4EndTooth = 0;
uint16_t ignition5EndTooth = 0;
uint16_t ignition6EndTooth = 0;
uint16_t ignition7EndTooth = 0;
uint16_t ignition8EndTooth = 0;

int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
static libdivide::libdivide_s16_t divTriggerToothAngle;
#endif

/** Universal (shared between decoders) decoder routines.
*
* @defgroup dec_uni Universal Decoder Routines
* 
* @{
*/
// whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)

/** Add tooth log entry to toothHistory (array).
 * Enabled by (either) currentStatus.toothLogEnabled and currentStatus.compositeTriggerUsed.
 * @param toothTime - Tooth Time
 * @param whichTooth - 0 for Primary (Crank), 2 for Secondary (Cam) 3 for Tertiary (Cam)
 */
static inline void addToothLogEntry(unsigned long toothTime, byte whichTooth)
{
  if(BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) { return; }
  //High speed tooth logging history
  if( (currentStatus.toothLogEnabled == true) || (currentStatus.compositeTriggerUsed > 0) ) 
  {
    bool valueLogged = false;
    if(currentStatus.toothLogEnabled == true)
    {
      //Tooth log only works on the Crank tooth
      if(whichTooth == TOOTH_CRANK)
      { 
        toothHistory[toothHistoryIndex] = toothTime; //Set the value in the log. 
        valueLogged = true;
      } 
    }
    else if(currentStatus.compositeTriggerUsed > 0)
    {
      compositeLogHistory[toothHistoryIndex] = 0;
      if(currentStatus.compositeTriggerUsed == 4)
      {
        // we want to display both cams so swap the values round to display primary as cam1 and secondary as cam2, include the crank in the data as the third output
        if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
        if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); }
        if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); }
        if(whichTooth > TOOTH_CAM_SECONDARY) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      }
      else
      {
        // we want to display crank and one of the cams
        if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
        if(currentStatus.compositeTriggerUsed == 3)
        { 
          // display cam2 and also log data for cam 1
          if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); } // only the COMPOSITE_LOG_SEC value is visualised hence the swapping of the data
          if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); } 
        } 
        else
        { 
          // display cam1 and also log data for cam 2 - this is the historic composite view
          if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); } 
          if(READ_THIRD_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD); }
        }
        if(whichTooth > TOOTH_CRANK) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      }  
      if(currentStatus.hasSync == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SYNC); }

      if(revolutionOne == 1)
      { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);}
      else
      { BIT_CLEAR(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);}

      toothHistory[toothHistoryIndex] = micros();
      valueLogged = true;
    }

    //If there has been a value logged above, update the indexes
    if(valueLogged == true)
    {
     if(toothHistoryIndex < (TOOTH_LOG_SIZE-1)) { toothHistoryIndex++; BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
     else { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
    }


  } //Tooth/Composite log enabled
}

/** Interrupt handler for primary trigger.
* This function is called on both the rising and falling edges of the primary trigger, when either the 
* composite or tooth loggers are turned on. 
*/
void loggerPrimaryISR(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  bool validEdge = false; //This is set true below if the edge 
  /* 
  Need to still call the standard decoder trigger. 
  Two checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  If either of these are true, the primary decoder function is called
  */
  if( ( (primaryTriggerEdge == RISING) && (READ_PRI_TRIGGER() == HIGH) ) || ( (primaryTriggerEdge == FALLING) && (READ_PRI_TRIGGER() == LOW) ) || (primaryTriggerEdge == CHANGE) )
  {
    triggerHandler();
    validEdge = true;
  }
  if( (currentStatus.toothLogEnabled == true) && (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Tooth logger only logs when the edge was correct
    if(validEdge == true) 
    { 
      addToothLogEntry(curGap, TOOTH_CRANK);
    }
  }
  else if( (currentStatus.compositeTriggerUsed > 0) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap, TOOTH_CRANK);
  }
}

/** Interrupt handler for secondary trigger.
* As loggerPrimaryISR, but for the secondary trigger.
*/
void loggerSecondaryISR(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  if( ( (secondaryTriggerEdge == RISING) && (READ_SEC_TRIGGER() == HIGH) ) || ( (secondaryTriggerEdge == FALLING) && (READ_SEC_TRIGGER() == LOW) ) || (secondaryTriggerEdge == CHANGE) )
  {
    triggerSecondaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeTriggerUsed > 0) && (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap2, TOOTH_CAM_SECONDARY);
  }
}

/** Interrupt handler for third trigger.
* As loggerPrimaryISR, but for the third trigger.
*/
void loggerTertiaryISR(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  
  
  if( ( (tertiaryTriggerEdge == RISING) && ( READ_THIRD_TRIGGER() == HIGH) ) || ( (tertiaryTriggerEdge == FALLING) && (READ_THIRD_TRIGGER() == LOW) ) || (tertiaryTriggerEdge == CHANGE) )
  {
    triggerTertiaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeTriggerUsed > 0) && (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap3, TOOTH_CAM_TERTIARY);
  }  
}

static inline bool IsCranking(const statuses &status) {
  return (status.RPM < status.crankRPM) && (status.startRevolutions == 0U);
}

#if defined(UNIT_TEST)
bool SetRevolutionTime(uint32_t revTime)
#else
static __attribute__((noinline)) bool SetRevolutionTime(uint32_t revTime)
#endif
{
  if (revTime!=revolutionTime) {
    revolutionTime = revTime;
    microsPerDegree = div360(revolutionTime << microsPerDegree_Shift);
    degreesPerMicro = (uint16_t)UDIV_ROUND_CLOSEST((UINT32_C(360) << degreesPerMicro_Shift), revolutionTime, uint32_t);
    return true;
  } 
  return false;
}

static bool UpdateRevolutionTimeFromTeeth(bool isCamTeeth) {
  noInterrupts();
  bool updatedRevTime = HasAnySync(currentStatus) 
    && !IsCranking(currentStatus)
    && (toothOneMinusOneTime!=UINT32_C(0))
    && (toothOneTime>toothOneMinusOneTime) 
    //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
    && SetRevolutionTime((toothOneTime - toothOneMinusOneTime) >> (isCamTeeth ? 1U : 0U)); 

  interrupts();
 return updatedRevTime;  
}

static inline uint16_t clampRpm(uint16_t rpm) {
    return rpm>=MAX_RPM ? currentStatus.RPM : rpm;
}

static inline uint16_t RpmFromRevolutionTimeUs(uint32_t revTime) {
  if (revTime<UINT16_MAX) {
    return clampRpm(udiv_32_16_closest(MICROS_PER_MIN, revTime));
  } else {
    return clampRpm((uint16_t)UDIV_ROUND_CLOSEST(MICROS_PER_MIN, revTime, uint32_t)); //Calc RPM based on last full revolution time (Faster as /)
  }
}

/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM (The time the last full revolution took) A common function is simpler.
* @param degreesOver - the number of crank degrees between tooth #1s. Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
static __attribute__((noinline)) uint16_t stdGetRPM(bool isCamTeeth)
{
  if (UpdateRevolutionTimeFromTeeth(isCamTeeth)) {
    return RpmFromRevolutionTimeUs(revolutionTime);
  }

  return currentStatus.RPM;
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static inline void setFilter(unsigned long curGap)
{
  if(configPage4.triggerFilter == 0) { triggerFilterTime = 0; } //trigger filter is turned off.
  else if(configPage4.triggerFilter == 1) { triggerFilterTime = curGap >> 2; } //Lite filter level is 25% of previous gap
  else if(configPage4.triggerFilter == 2) { triggerFilterTime = curGap >> 1; } //Medium filter level is 50% of previous gap
  else if (configPage4.triggerFilter == 3) { triggerFilterTime = (curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
  else { triggerFilterTime = 0; } //trigger filter is turned off.
}

/**
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution.
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evenly spaced.
It takes an argument of the full (COMPLETE) number of teeth per revolution.
For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static __attribute__((noinline)) int crankingGetRPM(byte totalTeeth, bool isCamTeeth)
{
  if( (currentStatus.startRevolutions >= configPage4.StgCycles) && ((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) )
  {
    if((toothLastMinusOneToothTime > 0) && (toothLastToothTime > toothLastMinusOneToothTime) )
    {
      noInterrupts();
      bool newRevtime = SetRevolutionTime(((toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth) >> (isCamTeeth ? 1U : 0U));
      interrupts();
      if (newRevtime) {
        return RpmFromRevolutionTimeUs(revolutionTime);
      }
    }
  }

  return currentStatus.RPM;
}

/**
On decoders that are enabled for per tooth based timing adjustments, this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate and an end compare value (This situation occurs when both the start and end of the ignition pulse happen after the end tooth, but before the next tooth)
*/
static inline void checkPerToothTiming(int16_t crankAngle, uint16_t currentTooth)
{
  if ( (fixedCrankingOverride == 0) && (currentStatus.RPM > 0) )
  {
    if ( (currentTooth == ignition1EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule1, ignition1EndAngle, crankAngle);
    }
    else if ( (currentTooth == ignition2EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule2, ignition2EndAngle, crankAngle);
    }
    else if ( (currentTooth == ignition3EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule3, ignition3EndAngle, crankAngle);
    }
    else if ( (currentTooth == ignition4EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule4, ignition4EndAngle, crankAngle);
    }
#if IGN_CHANNELS >= 5
    else if ( (currentTooth == ignition5EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule5, ignition5EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 6
    else if ( (currentTooth == ignition6EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule6, ignition6EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 7
    else if ( (currentTooth == ignition7EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule7, ignition7EndAngle, crankAngle);
    }
#endif
#if IGN_CHANNELS >= 8
    else if ( (currentTooth == ignition8EndTooth) )
    {
      adjustCrankAngle(ignitionSchedule8, ignition8EndAngle, crankAngle);
    }
#endif
  }
}
/** @} */
  
/** A (single) multi-tooth wheel with one of more 'missing' teeth.
* The first tooth after the missing one is considered number 1 and is the basis for the trigger angle.
* Note: This decoder does not currently support dual wheel (ie missing tooth + single tooth on cam).
* @defgroup dec_miss Missing tooth wheel
* @{
*/
void triggerSetup_missingTooth(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == CAM_SPEED) 
  { 
    //Account for cam speed missing tooth
    triggerToothAngle = 720 / configPage4.triggerTeeth; 
    BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  } 
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  if (configPage4.trigPatternSec == SEC_TRIGGER_4_1)
  {
    triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 4U / 2U;
  }
  else 
  {
    triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U));
  }
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0; 
  thirdToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM/50U) * triggerToothAngle * (configPage4.triggerMissingTeeth + 1U)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  if( (configPage4.TrigSpeed == CRANK_SPEED) && ( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage6.vvtEnabled > 0)) ) { BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY); }
  else { BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY); }
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif  
}

void triggerSec_missingTooth(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if( (toothLastSecToothTime == 0) )
  { 
    curGap2 = 0; 
    toothLastSecToothTime = curTime2;
  }

  if ( curGap2 >= triggerSecFilterTime )
  {
    switch (configPage4.trigPatternSec)
    {
      case SEC_TRIGGER_4_1:
        targetGap2 = (3 * (toothLastSecToothTime - toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
        toothLastMinusOneSecToothTime = toothLastSecToothTime;
        if ( (curGap2 >= targetGap2) || (secondaryToothCount > 3) )
        {
          secondaryToothCount = 1;
          revolutionOne = 1; //Sequential revolution reset
          triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
          triggerRecordVVT1Angle();
        }
        else
        {
          triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalc'd for the regular teeth, not the missing one.
          secondaryToothCount++;
        }
        break;

      case SEC_TRIGGER_POLL:
        //Poll is effectively the same as SEC_TRIGGER_SINGLE, however we do not reset revolutionOne
        //We do still need to record the angle for VVT though
        triggerSecFilterTime = curGap2 >> 1; //Next secondary filter is half the current gap
        triggerRecordVVT1Angle();
        break;

      case SEC_TRIGGER_SINGLE:
        //Standard single tooth cam trigger
        revolutionOne = 1; //Sequential revolution reset
        triggerSecFilterTime = curGap2 >> 1; //Next secondary filter is half the current gap
        secondaryToothCount++;
        triggerRecordVVT1Angle();
        break;

      case SEC_TRIGGER_TOYOTA_3:
        // designed for Toyota VVTI (2JZ) engine - 3 triggers on the cam. 
        // the 2 teeth for this are within 1 rotation (1 tooth first 360, 2 teeth second 360)
        secondaryToothCount++;
        if(secondaryToothCount == 2)
        { 
          revolutionOne = 1; // sequential revolution reset
          triggerRecordVVT1Angle();         
        }        
        //Next secondary filter is 25% the current gap, done here so we don't get a great big gap for the 1st tooth
        triggerSecFilterTime = curGap2 >> 2; 
        break;
    }
    toothLastSecToothTime = curTime2;
  } //Trigger filter
}

static inline void triggerRecordVVT1Angle (void)
{
  //Record the VVT Angle
  if( (configPage6.vvtEnabled > 0) && (revolutionOne == 1) )
  {
    int16_t curAngle;
    curAngle = getCrankAngle();
    while(curAngle > 360) { curAngle -= 360; }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage10.vvtCL0DutyAng; }

    currentStatus.vvt1Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
  }
}


void triggerThird_missingTooth(void)
{
//Record the VVT2 Angle (the only purpose of the third trigger)
//NB no filtering of this signal with current implementation unlike Cam (VVT1)

  int16_t curAngle;
  curTime3 = micros();
  curGap3 = curTime3 - toothLastThirdToothTime;

  //Safety check for initial startup
  if( (toothLastThirdToothTime == 0) )
  { 
    curGap3 = 0; 
    toothLastThirdToothTime = curTime3;
  }

  if ( curGap3 >= triggerThirdFilterTime )
  {
    thirdToothCount++;
    triggerThirdFilterTime = curGap3 >> 2; //Next third filter is 25% the current gap
    
    curAngle = getCrankAngle();
    while(curAngle > 360) { curAngle -= 360; }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage4.vvt2CL0DutyAng; }
    //currentStatus.vvt2Angle = int8_t (curAngle); //vvt1Angle is only int8, but +/-127 degrees is enough for VVT control
    currentStatus.vvt2Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt2Angle);    

    toothLastThirdToothTime = curTime3;
  } //Trigger filter
}

uint16_t getRPM_missingTooth(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed==CAM_SPEED); //Account for cam speed
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(configPage4.TrigSpeed==CAM_SPEED); //Account for cam speed
  }
  return tempRPM;
}

int getCrankAngle_missingTooth(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempRevolutionOne = revolutionOne;
    tempToothLastToothTime = toothLastToothTime;
    interrupts();

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    lastCrankAngleCalc = micros();
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(elapsedTime);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

static inline uint16_t clampToToothCount(int16_t toothNum, uint8_t toothAdder) {
  int16_t toothRange = (int16_t)configPage4.triggerTeeth + (int16_t)toothAdder;
  return (uint16_t)nudge(1, toothRange, toothNum, toothRange);
}

static inline uint16_t clampToActualTeeth(uint16_t toothNum, uint8_t toothAdder) {
  if(toothNum > triggerActualTeeth && toothNum <= configPage4.triggerTeeth) { toothNum = triggerActualTeeth; }
  return min(toothNum, (uint16_t)(triggerActualTeeth + toothAdder));
}

static uint16_t __attribute__((noinline)) calcEndTeeth_missingTooth(int endAngle, uint8_t toothAdder) {
  //Temp variable used here to avoid potential issues if a trigger interrupt occurs part way through this function
  int16_t tempEndTooth;
#ifdef USE_LIBDIVIDE  
  tempEndTooth = libdivide::libdivide_s16_do(endAngle - configPage4.triggerAngle, &divTriggerToothAngle);
#else
  tempEndTooth = (endAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerToothAngle;
#endif
  //For higher tooth count triggers, add a 1 tooth margin to allow for calculation time. 
  if(configPage4.triggerTeeth > 12U) { tempEndTooth = tempEndTooth - 1; }
  
  // Clamp to tooth count
  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

void triggerSetEndTeeth_missingTooth(void)
{
  uint8_t toothAdder = 0;
  if( ((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage4.sparkMode == IGN_MODE_SINGLE)) && (configPage4.TrigSpeed == CRANK_SPEED) && (configPage2.strokes == FOUR_STROKE) ) { toothAdder = configPage4.triggerTeeth; }

  ignition1EndTooth = calcEndTeeth_missingTooth(ignition1EndAngle, toothAdder);
  ignition2EndTooth = calcEndTeeth_missingTooth(ignition2EndAngle, toothAdder);
  ignition3EndTooth = calcEndTeeth_missingTooth(ignition3EndAngle, toothAdder);
  ignition4EndTooth = calcEndTeeth_missingTooth(ignition4EndAngle, toothAdder);
#if IGN_CHANNELS >= 5
  ignition5EndTooth = calcEndTeeth_missingTooth(ignition5EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  ignition6EndTooth = calcEndTeeth_missingTooth(ignition6EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  ignition7EndTooth = calcEndTeeth_missingTooth(ignition7EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  ignition8EndTooth = calcEndTeeth_missingTooth(ignition8EndAngle, toothAdder);
#endif
}
/** @} */


/** Dual Wheel Primary.
 * 
 * */
void triggerPri_DualWheel(void)
{
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    if ( curGap >= triggerFilterTime )
    {
      toothCurrentCount++; //Increment the tooth counter
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;

      if ( currentStatus.hasSync == true )
      {
        if ( (toothCurrentCount == 1) || (toothCurrentCount > configPage4.triggerTeeth) )
        {
          toothCurrentCount = 1;
          revolutionOne = !revolutionOne; //Flip sequential revolution tracker
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.startRevolutions++; //Counter
          if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
        }

        setFilter(curGap); //Recalc the new filter value
      }

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + configPage4.triggerAngle;
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += 360;
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
        }
        else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   } //Trigger filter
}
/** Dual Wheel Secondary.
 * 
 * */
void triggerSec_DualWheel(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if ( curGap2 >= triggerSecFilterTime )
  {
    toothLastSecToothTime = curTime2;
    triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed

    if( (currentStatus.hasSync == false) || (currentStatus.startRevolutions <= configPage4.StgCycles) )
    {
      toothLastToothTime = micros();
      toothLastMinusOneToothTime = micros() - ((MICROS_PER_MIN/10U) / configPage4.triggerTeeth); //Fixes RPM at 10rpm until a full revolution has taken place
      toothCurrentCount = configPage4.triggerTeeth;
      triggerFilterTime = 0; //Need to turn the filter off here otherwise the first primary tooth after achieving sync is ignored

      currentStatus.hasSync = true;
    }
    else 
    {
      if ( (toothCurrentCount != configPage4.triggerTeeth) && (currentStatus.startRevolutions > 2)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
      if (configPage4.useResync == 1) { toothCurrentCount = configPage4.triggerTeeth; }
    }

    revolutionOne = 1; //Sequential revolution reset
  }
  else 
  {
    triggerSecFilterTime = revolutionTime >> 1; //Set filter at 25% of the current cam speed. This needs to be performed here to prevent a situation where the RPM and triggerSecFilterTime get out of alignment and curGap2 never exceeds the filter value
  } //Trigger filter
}
/** Dual Wheel - Get RPM.
 * 
 * */
uint16_t getRPM_DualWheel(void)
{
  if( currentStatus.hasSync == true )
  {
    //Account for cam speed
    if( currentStatus.RPM < currentStatus.crankRPM )
    {
      return crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed==CAM_SPEED);
    }
    else
    {
      return stdGetRPM(configPage4.TrigSpeed==CAM_SPEED);
    }
  }
  return 0U;
}

/** Dual Wheel - Get Crank angle.
 * 
 * */
int getCrankAngle_DualWheel(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    tempRevolutionOne = revolutionOne;
    lastCrankAngleCalc = micros();
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = configPage4.triggerTeeth; }

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngleDegPerMicroSec(elapsedTime);

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

static uint16_t __attribute__((noinline)) calcEndTeeth_DualWheel(int ignitionAngle, uint8_t toothAdder) {
  int16_t tempEndTooth =
#ifdef USE_LIBDIVIDE
      libdivide::libdivide_s16_do(ignitionAngle - configPage4.triggerAngle, &divTriggerToothAngle);
#else
      (ignitionAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerToothAngle;
#endif
  return clampToToothCount(tempEndTooth, toothAdder);
}

/** Dual Wheel - Set End Teeth.
 * 
 * */
void triggerSetEndTeeth_DualWheel(void)
{
  //The toothAdder variable is used for when a setup is running sequentially, but the primary wheel is running at crank speed. This way the count of teeth will go up to 2* the number of primary teeth to allow for a sequential count. 
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  ignition1EndTooth = calcEndTeeth_DualWheel(ignition1EndAngle, toothAdder);
  ignition2EndTooth = calcEndTeeth_DualWheel(ignition2EndAngle, toothAdder);
  ignition3EndTooth = calcEndTeeth_DualWheel(ignition3EndAngle, toothAdder);
  ignition4EndTooth = calcEndTeeth_DualWheel(ignition4EndAngle, toothAdder);
#if IGN_CHANNELS >= 5
  ignition5EndTooth = calcEndTeeth_DualWheel(ignition5EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  ignition6EndTooth = calcEndTeeth_DualWheel(ignition6EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  ignition7EndTooth = calcEndTeeth_DualWheel(ignition7EndAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  ignition8EndTooth = calcEndTeeth_DualWheel(ignition8EndAngle, toothAdder);
#endif
}
/** @} */
