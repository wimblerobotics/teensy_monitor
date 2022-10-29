#include "tmodule.h"

#include <Arduino.h>
#include <stdint.h>

#include "tmicro_ros.h"

#define DO_TIMING true

TModule::TModule(TModule::MODULE moduleKind) {
  all_modules_[moduleKind] = this;
  loop_calls_between_get_statistics_calls = 0;
  for (size_t i = 0; i < kNumberSlots; i++) {
    duration_stats_[i] = 0.0;
  }
}

void TModule::GetStatistics(char* outString, size_t outStringSize) {
  static uint32_t statTimingStart = micros();
  char statList[2048];

  statList[0] = '\0';

  for (size_t i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      TModule* module = all_modules_[i];
      static size_t MAXLEN = 512;
      char temp[MAXLEN];
      temp[0] = '\0';
      snprintf(temp, MAXLEN,
               "{\"n\":\"%-s\",\"MnMxAv\":[%-2.1f,%-2.1f,%-2.1f]},",
               all_modules_[i]->name(), module->duration_stats_[kMin],
               module->duration_stats_[kMax],
               module->duration_stats_[kSum] / total_do_loop_count_);
      strcat(statList, temp);
      module->loop_calls_between_get_statistics_calls = 0;
      module->duration_stats_[kMin] = 10'000'000.0;
      module->duration_stats_[kMax] = -10'000'000.0;
      module->duration_stats_[kSum] = 0.0;
    }
  }

  // Remove trailing comma from previous list.
  if (strlen(statList) > 0) {
    statList[strlen(statList) - 1] = '\0';
  }

  snprintf(outString, outStringSize,
           "{\"loops\":%-ld,\"Ms\":%-2.1f,\"mdls\":[%-s]}",
           total_do_loop_count_, ((micros() * 1.0) - statTimingStart) / 1000.0,
           statList);
  statTimingStart = micros();
  total_do_loop_count_ = 0;
}

void TModule::DoLoop() {
  for (size_t i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      TModule* module = all_modules_[i];
      uint32_t start = micros();
      // char diagnostic[128];
      // snprintf(diagnostic, sizeof(diagnostic), "info TModule::DoLoop name: %s",
      //          all_modules_[i]->name());
      // TMicroRos::singleton().publishDiagnostic(diagnostic);


      all_modules_[i]->loop();

      float duration = ((micros() * 1.0) - start) / 1000.0;
      module->duration_stats_[kSum] += duration;
      if (duration < module->duration_stats_[kMin]) {
        module->duration_stats_[kMin] = duration;
      }

      if (duration > module->duration_stats_[kMax]) {
        module->duration_stats_[kMax] = duration;
      }

      module->loop_calls_between_get_statistics_calls++;
    }
  }

  total_do_loop_count_++;
}

void TModule::DoSetup() {
  for (int i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      all_modules_[i]->setup();
    }
  }
}

TModule* TModule::all_modules_[TModule::kNumberModules + 1] = {
    nullptr, nullptr, nullptr, nullptr, nullptr,
    nullptr, nullptr, nullptr, nullptr, nullptr};

uint32_t TModule::total_do_loop_count_ = 0;