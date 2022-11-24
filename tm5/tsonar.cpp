#include "tsonar.h"

#include <stdint.h>

#include "Arduino.h"
#include "TimerOne.h"
#include "Wire.h"
#include "tconfiguration.h"
#include "tmicro_ros.h"

void TSonar::CommonInterruptHandler(uint8_t pin, long& end_time,
                                    long& start_time, uint8_t& average_index,
                                    size_t sonar_index) {
  switch (digitalRead(pin)) {
    case HIGH:
      // When the SONAR begins transmitting, the echo pin will
      // go HIGH. Capture the start time.
      end_time = 0;
      start_time = micros();
      break;

    case LOW:
      // When an echo is received, the echo pin will go LOW.
      // Compute the distance to the object and send a ROS message.
      end_time = micros();
      g_values_mm_[sonar_index] =
          (end_time - start_time) * g_time_to_mm_scaler_;
      g_values_mm_history_[sonar_index][average_index++] =
          g_values_mm_[sonar_index];
      if (average_index >= kNumberReadingsToAverage) {
        average_index = 0;
      }

      // Compute an average distance over the last few readings
      // to help reduce the sensor noise.
      int average_sum_mm = 0;
      for (size_t i = 0; i < kNumberReadingsToAverage; i++) {
        average_sum_mm += g_values_mm_history_[sonar_index][i];
      }

      g_average_value_m_[sonar_index] =
          (average_sum_mm * 0.001) / kNumberReadingsToAverage;

      // Publish the average distance for the sensor.
      TMicroRos::PublishSonar(sonar_index, g_average_value_m_[sonar_index]);
      break;
  }
}

float TSonar::GetAverageValueM(Sonar device) {
  return g_average_value_m_[device];
}

void TSonar::Echo0InterruptHandler() {
  // Each sensor has it's own distance timer and it's own
  // set of values used to compute an averate distance.
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho0, end_time, start_time, average_index, 0);
}

void TSonar::Echo1InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho1, end_time, start_time, average_index, 1);
}

void TSonar::Echo2InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho2, end_time, start_time, average_index, 2);
}

void TSonar::Echo3InterruptHandler() {
  static long end_time = 0;
  static long start_time = 0;
  static uint8_t average_index = 0;
  CommonInterruptHandler(kPinEcho3, end_time, start_time, average_index, 3);
}

int TSonar::GetValueMm(Sonar device) {
  if (static_cast<int>(device) >= kNumberSonars) {
    return -1;
  } else {
    return g_values_mm_[static_cast<int>(device)];
  }
}

void TSonar::loop() {
  if (TM5::kDoDetailDebug) {
    TMicroRos::PublishDiagnostic("INFO [TSonar::loop]");
  }
  // Nothing needs to be done in the loop. Everything is drivven
  // by interrupts.
}

void TSonar::setup() {
  if (TM5::kDoDetailDebug) {
    TMicroRos::PublishDiagnostic("INFO [TSonar::setup]");
  }
  // Set up the Teensy pins to talk to the SONAR devices.
  pinMode(kPinEcho0, INPUT);
  pinMode(kPinTrigger0, OUTPUT);
  pinMode(kPinEcho1, INPUT);
  pinMode(kPinTrigger1, OUTPUT);
  pinMode(kPinEcho2, INPUT);
  pinMode(kPinTrigger2, OUTPUT);
  pinMode(kPinEcho3, INPUT);
  pinMode(kPinTrigger3, OUTPUT);

  // Setup a timer to drive the state machine.
  Timer1.initialize(kTimerPeriodUSec);
  Timer1.attachInterrupt(TimerInterruptHandler);

  // Attach an interrupt handler to each SONAR's echo pin.
  attachInterrupt(kPinEcho0, Echo0InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho1, Echo1InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho2, Echo2InterruptHandler, CHANGE);
  attachInterrupt(kPinEcho3, Echo3InterruptHandler, CHANGE);
}

void TSonar::TimerInterruptHandler() {
  // The states of the state machine.
  typedef enum {
    COUNTDOWN,   // Count down timer ticks.
    PULSE_HIGH,  // Begin transmitting the SONAR pulse.
    PULSE_LOW  // Stop transmitting the SONAR pulse and begin listening for the
               // echo.
  } TTimerState;

  // The current state of the state machine.
  static volatile TTimerState state = COUNTDOWN;

  // Used to count timer pulses to drive the timing of each SONAR sensor.
  static volatile long countdown = kTimerCountsPerSamplingPeriod;

  if (--countdown == 0) {
    // Time to start the next SONAR sensor in the list of sensors.
    state = PULSE_HIGH;
    countdown = kTimerCountsPerSamplingPeriod;
  }

  switch (state) {
    case COUNTDOWN:
      // Continue counting so that the sensors are spaced
      // apart in time so they don't interfere with each other.
      break;

    case PULSE_HIGH:
      // Time to send out the ping for the next SONAR in the list.
      if ((g_next_sensor_index_ % 4) == 0) {
        digitalWrite(kPinTrigger0, HIGH);
      } else if ((g_next_sensor_index_ % 4) == 1) {
        digitalWrite(kPinTrigger1, HIGH);
      } else if ((g_next_sensor_index_ % 4) == 2) {
        digitalWrite(kPinTrigger2, HIGH);
      } else {
        digitalWrite(kPinTrigger3, HIGH);
      }

      state = PULSE_LOW;
      break;

    case PULSE_LOW:
      // Time to stop the ping output and begin listening for the
      // echo for the next SONAR in the list.
      if ((g_next_sensor_index_ % 4) == 0) {
        digitalWrite(kPinTrigger0, LOW);
      } else if ((g_next_sensor_index_ % 4) == 1) {
        digitalWrite(kPinTrigger1, LOW);
      } else if ((g_next_sensor_index_ % 4) == 2) {
        digitalWrite(kPinTrigger2, LOW);
      } else {
        digitalWrite(kPinTrigger3, LOW);
      }

      g_next_sensor_index_++;  // Control the next sensor in the list the next
                               // time around.
      state = COUNTDOWN;       // Begin counting down again.
      break;
  }
}

TSonar::TSonar() : TModule(TModule::kSonar) {
  for (size_t i = 0; i < kNumberSonars; i++) {
    // Initialize the array of readings used to computer the average distance
    // per sensor.
    for (size_t j = 0; j < kNumberReadingsToAverage; j++) {
      g_values_mm_history_[i][j] = 0;
    }
  }
}

TSonar& TSonar::singleton() {
  // A singleton pattern is used to support the TModule design pattern.
  if (!g_singleton_) {
    g_singleton_ = new TSonar();
  }

  return *g_singleton_;
}

uint8_t TSonar::g_next_sensor_index_ =
    0;  // Which sensor in the list is being controlled now.

TSonar* TSonar::g_singleton_ =
    nullptr;  // Used to ensure that only one instance of this class is ever
              // constructed.

int TSonar::g_values_mm_[TSonar::kNumberSonars] = {
    -1, -1, -1, -1};  // The last reading per sensor.

int TSonar::g_values_mm_history_
    [TSonar::kNumberSonars]
    [TSonar::kNumberReadingsToAverage];  // List of readings per sensur used to
                                         // computer an average per sensor.

float TSonar::g_average_value_m_[TSonar::kNumberSonars] = {
    0, 0, 0, 0};  // The computed average distance per sensor.

const float TSonar::g_time_to_mm_scaler_ =
    (10.0 / 2.0) / 29.1;  // For converting time to distance.