#pragma once

#include <SD.h>

#include "tmodule.h"

class TSd : TModule {
 public:
  // Write message to log file.
  void log(char* message);

  // From TModule.
  void loop();

  // From TModule.
  virtual const char* name() { return "TSd"; }

  void setup();

  // Singleton constructor.
  static TSd& singleton();

 private:
  // Private constructor.
  TSd();

  // Has SD device been properly initialized?
  static bool g_initialized;

  static File g_logFile;

  // Singleton instance.
  static TSd* g_singleton;
};