#pragma once

#include <Regexp.h>
#include <SD.h>

#include "tmodule.h"

// A class used to write logging information to an SD memory card.
//
// On setup, the SD card is examined for any existing files with names like
// "LOG12345.TXT", where "12345" is a 5-digit serial number. A new log file
// will be created on the card with a name having a serial number of one higher
// than any existing log file name,or LOG00001.TXT if no existing log files
// were found.
//
// Every time TSd::singleton().log() is called, the message is added to a
// string buffer. When the buffer becomes full enough, it is written as one
// large chunck of text to the log file. This is done so that lots of little
// writes are done, which would slow down the outer loop of the Arduino device.
//
// The downside is that if the card is pulled from the Arduino device, the last
// chunk of text won't have been written to the device. Currently, there is a fair 
// number of things written to the log file so if you just wait a few seconds before
// pulling the card, the interesting thing you were looking for in the log file may
// have been actually successfully written to the file before you pulled the card.
//
// This is intended to be used the the TModule software module as part of the
// TeensyMonitor stack. So all you need to do to instantiate and get this module
// going is to call TSd::singleton() in your ".ino" file before calling TModule::setup().
//
// If a write to the card fails, perhaps because the card is full or the card has been
// pulled from the Arduino device, further writes are not attempted until the system
// is restarted.

class TSd : TModule {
 public:
  // Write message to log file.
  void log(const char* message);

  // Singleton constructor.
  static TSd& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule.
  virtual const char* name() { return "TSd"; }

  void setup();

 private:
  // Private constructor.
  TSd();

  static void regexpMatchCallback(const char* match, const unsigned int length,
                                  const MatchState& matchState);

  // Used to hold a big chunk of data so writes are fewer.
  String data_buffer_;

  // The SD card device.
  static SDClass g_sd_;

  // Used to find to highest log file number already on the SD card.
  static int g_highestExistingLogFileNumber_;

  // Has SD device been properly initialized?
  static bool g_initialized_;

  // The file handle for the log file on the SD card device.
  static File g_logFile_;

  // Singleton instance.
  static TSd* g_singleton_;
};