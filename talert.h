#pragma once

class TAlert {
public:

  typedef enum TAlertSource {
    CURRENT_LEFT_MOTOR,
    CURRENT_RIGHT_MOTOR,
    SONAR_BACK,
    SONAR_FRONT,
    SONAR_LEFT,
    SONAR_RIGHT,
    TEMP_LEFT_MOTOR,
    TEMP_RIGHT_MOTOR,
    TOF_LOWER_LEFT_BACKWARD,
    TOF_LOWER_LEFT_SIDEWAY,
    TOF_LOWER_RIGHT_BACKWARD,
    TOF_LOWER_RIGHT_SIDEWAY,
    TOF_UPPER_LEFT_FORWARD,
    TOF_UPPER_LEFT_SIDEWAY,
    TOF_UPPER_RIGHT_FORWARD,
    TOF_UPPER_RIGHT_SIDEWAY,
    NUMER_ALERT_SOURCES
  } TAlertSource;

  void loop();

  void set(TAlertSource alert);

  void setup();

  static TAlert& singleton();

  void unset(TAlertSource alert);

private:

static bool g_alertsTriggered[NUMER_ALERT_SOURCES];

static TAlert* g_singleton;

};