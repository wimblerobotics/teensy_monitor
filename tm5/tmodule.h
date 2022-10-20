#pragma once

#include <stdint.h>

#include "Arduino.h"

/**
 * Base class for a kind of module which:
 *  - Needs to be setup once via a call to setup().
 *  - Needs to be invoked once per "processing cycle" via a call to loop().
 *  - Needs performance statistics gathered about performance during loop
 * execution.
 *  - Needs periodic, summary performance statistics reported.
 *
 * Usage:
 * Each module exists as a singleton. Somewhere, modules are
 * brought into existence by calling, typically, their
 * singleton() method. As each module comes into existence,
 * This TModule class keeps a list of them, in creation order.
 *
 * After all modules are brought into existence, you call the
 * doSetup() method to invoke the setup() method for all registered
 * modules. Then in the main processing loop, you invoke the
 * doLoop() method to invoked the loop() method for all registered
 * modules.
 */

class TModule {
 public:
  // A list of all possible modules.
  // Used only to definitely define NUMBER_MODULES.
  typedef enum MODULE {
    kALARM,
    kALERT,
    kMICRO_ROS,
    kMOTOR_CURRENT,
    kPANEL_SELECTOR,
    kRELAY,
    kROBOCLAW,
    kSONAR,
    kTEMPERATURE,
    kTIME_OF_FLIGHT,
    kNumberModules  // The number of all possible modules.
  } MODULE;

  // Call loop() for all registered modules.
  static void DoLoop();

  // Call setup() for all registered modules.
  static void DoSetup();

  // Perform regular, cyclic work for the module.
  virtual void loop() = 0;

  // Return module name.
  virtual const char* name() = 0;

  // Perform one-time setup for the module.
  virtual void setup() = 0;

  static void getStatistics(char* outString, size_t outStringSize);

 protected:
  TModule(MODULE moduleKind);

 private:
  TModule();

  // Define slots for gathering statistics for the module.
  typedef enum SLOT {
    kMin,
    kMax,
    kSum,
    kNumberSlots,            // Number of slots to reserve for statistics.
    kNumberReadings = 1'000  // Number of statistical readings to gather before
                             // generating a summary report.
  } SLOT;

  // Number of times DoLoop() was called.
  static uint32_t total_do_loop_count_;

  // A list of all registered modules.
  static TModule* all_modules_[];

  // Index to the next statistic reading for the module.
  int loop_calls_between_get_statistics_calls;

  // Statistics gathered for all registered modules.
  float duration_stats_[kNumberSlots];

  // // Singleton instance.
  // static TModule* singleton_;
};
