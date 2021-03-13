#include <Arduino.h>
#include <stdint.h>

#include "talarm.h"

TAlarm::TAlarmKind TAlarm::highestPriorityAlarm() {
  TAlarmKind result = NUMBER_ALARMS;

  if (g_alarms[MOTOR_ALARM]) {
    result = MOTOR_ALARM;
  }

  return result;
}

void TAlarm::loop() {
  TAlarmKind newHighestPriorityAlarm = highestPriorityAlarm();
  // Serial.print("newHighestPriorityAlarm: ");Serial.println(newHighestPriorityAlarm);
  if (newHighestPriorityAlarm == NUMBER_ALARMS) {
    g_lastStageTransitionMs = 0;
    noTone(TONE_PIN);
    return;
  }

  if (newHighestPriorityAlarm != g_highestPriorityAlarm) {
    // Start a new cycle.
    Serial.print("[TAlarm::loop] newHighestPriorityAlarm: ");Serial.println(newHighestPriorityAlarm);
    g_highestPriorityAlarm = newHighestPriorityAlarm;

    Serial.print("Start new alarm: ");Serial.println(newHighestPriorityAlarm);
    g_toneStage = 0;
    g_lastStageTransitionMs = millis();
    switch (g_highestPriorityAlarm) {
      case MOTOR_ALARM:
        g_currentAlarmMelody = &MOTOR_ALARM_TUNE[0];
        tone(TONE_PIN, g_currentAlarmMelody[0], DURATION_STAGE_TRANSITIONS_MS);
        return;

      default:
        return;
    }
  }


  uint32_t now = millis();
  if ((g_lastStageTransitionMs != 0) && (now > (g_lastStageTransitionMs + DURATION_STAGE_TRANSITIONS_MS))) {
    g_toneStage++;
    TPitch nextNote = g_currentAlarmMelody[g_toneStage];
    if (nextNote == END) {
      g_toneStage = 0;
      nextNote = g_currentAlarmMelody[0];
    }

    tone(TONE_PIN, nextNote, DURATION_STAGE_TRANSITIONS_MS);
    g_lastStageTransitionMs = now;
  } else {
    // Stay on same note.
  }
}


void TAlarm::set(TAlarmKind alarmKind) {
  g_alarms[alarmKind] = true;
  Serial.print("[TAlarm::set] alarmKind: ");
  Serial.println(alarmKind);
}


void TAlarm::setup() {}


void TAlarm::unset(TAlarmKind alarmKind) {
  g_alarms[alarmKind] = false;
  g_highestPriorityAlarm = NUMBER_ALARMS;
  Serial.print("[TAlarm::unset] alarmKind: ");
  Serial.println(alarmKind);
}

TAlarm::TAlarm() {}

TAlarm& TAlarm::singleton() {
  if (!g_singleton) {
    g_singleton = new TAlarm();
  }

  return *g_singleton;
}

bool TAlarm::g_alarms[TAlarm::NUMBER_ALARMS];

const TAlarm::TPitch* TAlarm::g_currentAlarmMelody = nullptr;

TAlarm::TAlarmKind TAlarm::g_highestPriorityAlarm = TAlarm::NUMBER_ALARMS;

uint32_t TAlarm::g_lastStageTransitionMs = 0;

TAlarm* TAlarm::g_singleton = nullptr;

uint8_t TAlarm::g_toneStage = 0;

const TAlarm::TPitch TAlarm::MOTOR_ALARM_TUNE[] = {
    TAlarm::NOTE_C4, TAlarm::NOTE_E4, TAlarm::NOTE_G4, TAlarm::END};