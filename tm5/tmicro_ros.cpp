#include "tmicro_ros.h"

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/temperature.h>
#include <stdint.h>
#include <stdio.h>

#include "tmodule.h"
#include "trelay.h"
#include "troboclaw.h"
#include "tsd.h"

#define ignore_result(x) \
  if (x) {               \
  }

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }

void TMicroRos::SyncTime() {
  static uint32_t time_at_last_sync = micros();
  TMicroRos::singleton().await_time_sync_ =
      true;  // Disable motor commands except for stop.

  static const int timeout_ms = 1000;
  uint32_t start = micros();
  rmw_ret_t sync_result =
      rmw_uros_sync_session(timeout_ms);  // Atttempt synchronization.
  int32_t now = micros();
  float sync_duration_ms = (now - start) / 1000.0;
  float duration_since_last_sync_ms = (now - time_at_last_sync) / 1000.0;
  time_at_last_sync = now;
  if (sync_result == RMW_RET_OK) {
    ros_sync_time_ =
        rmw_uros_epoch_nanos();  // Capture the current ROS time after
                                 // attempted synchronization.
  }

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "[TMicroRos::SyncTime] rmw_uros_sync_session(), call result: %d, sync_duration_ms: %f, "
           "duration_since_last_sync_ms: %f, new time: "
           "%lld.%lld",
           sync_result, sync_duration_ms, duration_since_last_sync_ms,
           ros_sync_time_ / 1'000'000'000, ros_sync_time_ % 1'000'000'000);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);

  TMicroRos::singleton().await_time_sync_ = false;  // Renable motor commands.
}

int64_t TMicroRos::FixedTime() {
  int64_t ros_time = rmw_uros_epoch_nanos();
  int64_t skew = ros_time - ros_sync_time_;
  if (skew < 0) {
    // Time has gone backwards !
    TMicroRos::singleton().SyncTime();
    ros_time = ros_sync_time_;
  } else if (skew > 30'000'000) {
    // The current time appears to be 30ms or more out of whack from
    // previously synced time
    TMicroRos::singleton().SyncTime();
    ros_time = ros_sync_time_;
  } else if (skew <= 5'000'000) {
    // If the current time is within 5ms of the last synced time,
    // assume that the current time is probably good.
    // The 5 ms is because the loop rate of this monitor is expected
    // to be more than 20 frames per second.
    ros_sync_time_ = ros_time;
  }

  return ros_time;
}

void TMicroRos::loop() {
  switch (state_) {
    case kWaitingAgent: {
      TSd::singleton().log("INFO [TMicroRos::loop] kWaitingAgent");
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 500) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? kAgentAvailable
                                                             : kWaitingAgent;
        last_time = uxr_millis();
      }
    } break;

    case kAgentAvailable: {
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentAvailable");
      if (CreateEntities()) {
        TSd::singleton().log(
            "INFO [TMicroRos::loop] kAgentAvailable successful "
            "CreateEntities");
        state_ = kAgentConnected;
      } else {
        TSd::singleton().log(
            "ERROR [TMicroRos::loop] kAgentAvailable FAILED CreateEntities");
        state_ = kWaitingAgent;
        DestroyEntities();
      }
    } break;

    case kAgentConnected: {
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentConnected");
      static int64_t last_time = uxr_millis();
      if ((uxr_millis() - last_time) > 10) {
        state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                     ? kAgentConnected
                     : kAgentDisconnected;
        last_time = uxr_millis();
        if (TMicroRos::singleton().state_ == kAgentConnected) {
          rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
        }
      }
    } break;

    case kAgentDisconnected: {
      TSd::singleton().log("INFO [TMicroRos::loop] kAgentDisconnected");
      DestroyEntities();
      state_ = kWaitingAgent;
    } break;

    default:
      break;
  }

  {
#define LED_PIN 13
    static int32_t blink_counter = 0;
    if ((blink_counter++ % 100) == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}

void TMicroRos::PublishDiagnostic(const char *msg) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    snprintf(g_singleton_->string_msg_.data.data,
             g_singleton_->string_msg_.data.capacity, "%s", msg);
    g_singleton_->string_msg_.data.size =
        strlen(g_singleton_->string_msg_.data.data);
    ignore_result(rcl_publish(&g_singleton_->diagnostics_publisher_,
                              &g_singleton_->string_msg_, nullptr));
    TSd::singleton().log(msg);
  }
}

void TMicroRos::PublishSonar(uint8_t frame_id, float range) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    int64_t timestamp = TMicroRos::singleton().FixedTime();
    g_singleton_->sonar_range_msg_.header.stamp.nanosec =
        (int32_t)(timestamp % 1'000'000'000);
    g_singleton_->sonar_range_msg_.header.stamp.sec =
        (int32_t)(timestamp / 1'000'000'000);
    snprintf(g_singleton_->sonar_range_msg_.header.frame_id.data,
             g_singleton_->sonar_range_msg_.header.frame_id.capacity,
             "sonar_%1d", frame_id);
    g_singleton_->sonar_range_msg_.header.frame_id.size =
        strlen(g_singleton_->sonar_range_msg_.header.frame_id.data);
    g_singleton_->sonar_range_msg_.radiation_type = 0;
    g_singleton_->sonar_range_msg_.field_of_view = 0.523599;  // 30 degrees.
    g_singleton_->sonar_range_msg_.max_range = 2.0;
    g_singleton_->sonar_range_msg_.min_range = 0.0254;
    g_singleton_->sonar_range_msg_.range = range;
    ignore_result(rcl_publish(&g_singleton_->sonar_publisher_[frame_id],
                              &g_singleton_->sonar_range_msg_, nullptr));
  }
}

void TMicroRos::PublishTemperature(const char *frame_id, float temperature) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    int64_t timestamp = TMicroRos::singleton().FixedTime();
    g_singleton_->temperature_msg_.header.stamp.nanosec =
        (int32_t)(timestamp % 1'000'000'000);
    g_singleton_->temperature_msg_.header.stamp.sec =
        (int32_t)(FLEXCAN3_HR_TIME_STAMP0 / 1'000'000'000);

    snprintf(g_singleton_->temperature_msg_.header.frame_id.data,
             g_singleton_->temperature_msg_.header.frame_id.capacity, "%s",
             frame_id);
    g_singleton_->temperature_msg_.temperature = temperature;
    g_singleton_->temperature_msg_.variance = 0;
    ignore_result(rcl_publish(&g_singleton_->temperature_publisher_,
                              &g_singleton_->temperature_msg_, nullptr));
  }
}

void TMicroRos::PublishTof(uint8_t frame_id, float range) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    int64_t timestamp = TMicroRos::singleton().FixedTime();
    g_singleton_->tof_range_msg_.header.stamp.nanosec =
        (int32_t)(timestamp % 1'000'000'000);
    g_singleton_->tof_range_msg_.header.stamp.sec =
        (int32_t)(timestamp/ 1'000'000'000);

    snprintf(g_singleton_->tof_range_msg_.header.frame_id.data,
             g_singleton_->tof_range_msg_.header.frame_id.capacity, "tof_%1d",
             frame_id);
    g_singleton_->tof_range_msg_.header.frame_id.size =
        strlen(g_singleton_->tof_range_msg_.header.frame_id.data);
    g_singleton_->tof_range_msg_.radiation_type = 0;
    g_singleton_->tof_range_msg_.field_of_view = 0.436332;  // 25 degrees
    g_singleton_->tof_range_msg_.max_range = 2.0;
    g_singleton_->tof_range_msg_.min_range = 0.0254;
    g_singleton_->tof_range_msg_.radiation_type =
        sensor_msgs__msg__Range__ULTRASOUND;
    g_singleton_->tof_range_msg_.range = range;
    ignore_result(rcl_publish(&g_singleton_->tof_publisher_[frame_id],
                              &g_singleton_->tof_range_msg_, nullptr));
  }
}

void TMicroRos::setup() {
  static bool is_setup = false;
  if (!is_setup) {
    set_microros_transports();
    state_ = kWaitingAgent;
    while (state_ != kAgentConnected) {
      loop();
    }

    TRoboClaw::singleton().SetM1PID(7.26239, 1.36838, 00, 2437);
    TRoboClaw::singleton().SetM2PID(7.26239, 1.36838, 00, 2437);
    is_setup = true;
  }
}

void TMicroRos::TimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    if (timer != NULL) {
      const size_t MAXSIZE = 512;
      char stats[MAXSIZE];
      TModule::GetStatistics(stats, MAXSIZE);
      snprintf(g_singleton_->string_msg_.data.data,
               g_singleton_->string_msg_.data.capacity, "{\"Stats\": %s}",
               stats);
      g_singleton_->string_msg_.data.size =
          strlen(g_singleton_->string_msg_.data.data);
      ignore_result(rcl_publish(&g_singleton_->teensy_stats_publisher_,
                                &g_singleton_->string_msg_, nullptr));

      uint32_t error = TRoboClaw::singleton().getError();
      snprintf(g_singleton_->string_msg_.data.data,
               g_singleton_->string_msg_.data.capacity,
               "{\"LogicVoltage\":%-2.1f,\"MainVoltage\":%-2.1f,\"Encoder_"
               "Left\":%-ld,\"Encoder_Right\":"
               "%-ld,\"LeftMotorCurrent\":%-2.3f,\"RightMotorCurrent\":%-2.3f,"
               "\"LeftMotorSpeed\":%ld,\"RightMotorSpeed\":%ld,"
               "\"Errror\":%-lX}",
               TRoboClaw::singleton().GetBatteryLogic(),
               TRoboClaw::singleton().GetBatteryMain(),
               TRoboClaw::singleton().GetM1Encoder(),
               TRoboClaw::singleton().GetM2Encoder(),
               TRoboClaw::singleton().GetM1Current(),
               TRoboClaw::singleton().GetM2Current(),
               TRoboClaw::singleton().GetM1Speed(),
               TRoboClaw::singleton().GetM2Speed(), error);
      g_singleton_->string_msg_.data.size =
          strlen(g_singleton_->string_msg_.data.data);
      TSd::singleton().log(g_singleton_->string_msg_.data.data);
      ignore_result(rcl_publish(&g_singleton_->roboclaw_status_publisher_,
                                &g_singleton_->string_msg_, nullptr));
    }
  }
}

void TMicroRos::HeartbeatCallback(const void *heartbeat_msg) {
  // if (TMicroRos::singleton().state_ == kAgentConnected) {
  //   const diagnostic_msgs__msg__DiagnosticStatus *msg =
  //       (const diagnostic_msgs__msg__DiagnosticStatus *)heartbeat_msg;

  //   int32_t nanosec = (int32_t)(TMicroRos::singleton().FixedTime() %
  //   1000000000); int32_t sec = (int32_t)(TMicroRos::singleton().FixedTime() /
  //   1000000000);
  //   // msg.
  // }
}

void TMicroRos::TwistCallback(const void *twist_msg) {
  if (TMicroRos::singleton().state_ == kAgentConnected) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)twist_msg;

    if (TMicroRos::singleton().await_time_sync_ &&
        ((msg->linear.x != 0) || (msg->angular.z != 0))) {
      // A potential sync to the system time is ongoing.
      // Don't handle any motor commands except for stop (x==0, z==0).
      return;
    }

    double x_velocity =
        min(max((float)msg->linear.x, -g_singleton_->max_linear_velocity_),
            g_singleton_->max_linear_velocity_);
    double yaw_velocity =
        min(max((float)msg->angular.z, -g_singleton_->max_angular_velocity_),
            g_singleton_->max_angular_velocity_);
    if ((msg->linear.x == 0) && (msg->angular.z == 0)) {
      TRoboClaw::singleton().DoMixedSpeedDist(0, 0, 0, 0);
    } else if ((fabs(x_velocity) > 0.01) || (fabs(yaw_velocity) > 0.01)) {
      const double m1_desired_velocity =
          x_velocity - (yaw_velocity * g_singleton_->wheel_separation_ / 2.0) /
                           g_singleton_->wheel_radius_;
      const double m2_desired_velocity =
          x_velocity + (yaw_velocity * g_singleton_->wheel_separation_ / 2.0) /
                           g_singleton_->wheel_radius_;

      const int32_t m1_quad_pulses_per_second =
          m1_desired_velocity * g_singleton_->quad_pulses_per_meter_;
      const int32_t m2_quad_pulses_per_second =
          m2_desired_velocity * g_singleton_->quad_pulses_per_meter_;
      const int32_t m1_max_distance =
          fabs(m1_quad_pulses_per_second *
               g_singleton_->max_seconds_uncommanded_travel_);
      const int32_t m2_max_distance =
          fabs(m2_quad_pulses_per_second *
               g_singleton_->max_seconds_uncommanded_travel_);
      char diagnostic_message[256];
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMicroRos::TwistCallback(] accel qpps: %ld, m1 qpps: "
               "%ld, m1 "
               "max d: %ld, m2 qpps: %ld, m2 max d: %ld",
               g_singleton_->accel_quad_pulses_per_second_,
               m1_quad_pulses_per_second, m1_max_distance,
               m2_quad_pulses_per_second, m2_max_distance);
      TSd::singleton().log(diagnostic_message);
      TRoboClaw::singleton().DoMixedSpeedAccelDist(
          g_singleton_->accel_quad_pulses_per_second_,
          m1_quad_pulses_per_second, m1_max_distance, m2_quad_pulses_per_second,
          m2_max_distance);
    }
  }
}

TMicroRos::TMicroRos()
    : TModule(TModule::kMicroRos),
      await_time_sync_(true),
      accel_quad_pulses_per_second_(1000),
      max_angular_velocity_(0.07),
      max_linear_velocity_(0.3),
      max_seconds_uncommanded_travel_(0.25),
      quad_pulses_per_meter_(1566),
      wheel_radius_(0.10169),
      wheel_separation_(0.345) {
  string_msg_.data.capacity = 512;
  string_msg_.data.data =
      (char *)malloc(string_msg_.data.capacity * sizeof(char));
  string_msg_.data.size = 0;

  sonar_range_msg_.header.frame_id.capacity = 32;
  sonar_range_msg_.header.frame_id.data =
      (char *)malloc(sonar_range_msg_.header.frame_id.capacity * sizeof(char));
  sonar_range_msg_.header.frame_id.size = 0;

  temperature_msg_.header.frame_id.capacity = 32;
  temperature_msg_.header.frame_id.data =
      (char *)malloc(sonar_range_msg_.header.frame_id.capacity * sizeof(char));
  temperature_msg_.header.frame_id.size = 0;

  tof_range_msg_.header.frame_id.capacity = 32;
  tof_range_msg_.header.frame_id.data =
      (char *)malloc(sonar_range_msg_.header.frame_id.capacity * sizeof(char));
  tof_range_msg_.header.frame_id.size = 0;
}

bool TMicroRos::CreateEntities() {
  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator_));

  // create node
  RCCHECK(rclc_node_init_default(&node_, "teensy_node", "", &support_));

  // create publishers.
  RCCHECK(rclc_publisher_init_best_effort(
      &roboclaw_status_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "roboclaw_status"));

  RCCHECK(rclc_publisher_init_best_effort(
      &diagnostics_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy_diagnostics"));

  RCCHECK(rclc_publisher_init_best_effort(
      &teensy_stats_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "teensy_stats"));

  for (size_t i = 0; i < 4; i++) {
    char topic_name[16];
    sprintf(topic_name, "sonar%-1dSensor", i);
    RCCHECK(rclc_publisher_init_best_effort(
        &sonar_publisher_[i], &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic_name));
  }

  RCCHECK(rclc_publisher_init_best_effort(
      &temperature_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
      "temperature"));

  for (size_t i = 0; i < 8; i++) {
    char topic_name[16];
    sprintf(topic_name, "tof%-1dSensor", i);
    RCCHECK(rclc_publisher_init_default(
        &tof_publisher_[i], &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic_name));
  }

  // Create subscribers.
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // RCCHECK(rclc_subscription_init_default(
  //     &heatbeat_subscriber_, &node_,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "heartbeat"));

  // Create timer,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  RCCHECK(rclc_timer_init_default(&timer_, &support_, timer_timeout_ns,
                                  TimerCallback));

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 3, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));
  RCCHECK(rclc_executor_add_subscription(&executor_, &cmd_vel_subscriber_,
                                         &twist_msg_, &TwistCallback,
                                         ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_subscription(&executor_, &heatbeat_subscriber_,
  //                                        &twist_msg_, &HeartbeatCallback,
  //                                        ON_NEW_DATA));
  return true;
}

void TMicroRos::DestroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  // ignore_result(rcl_publisher_fini(&roboclaw_status_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&teensy_stats_publisher_, &node_));
  for (size_t i = 0; i < 4; i++) {
    ignore_result(rcl_publisher_fini(&sonar_publisher_[i], &node_));
  }

  ignore_result(rcl_publisher_fini(&temperature_publisher_, &node_));
  for (size_t i = 0; i < 8; i++) {
    ignore_result(rcl_publisher_fini(&tof_publisher_[i], &node_));
  }

  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rclc_executor_fini(&executor_));
  ignore_result(rcl_node_fini(&node_));
  ignore_result(rclc_support_fini(&support_));
}

TMicroRos &TMicroRos::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TMicroRos();
  }

  return *g_singleton_;
}

TMicroRos *TMicroRos::g_singleton_ = nullptr;
volatile int64_t TMicroRos::ros_sync_time_ = 0;