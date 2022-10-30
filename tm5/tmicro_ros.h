#pragma once

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/temperature.h>
#include <stdio.h>

#include "tmodule.h"

class TMicroRos : TModule {
 public:
  static void publishDiagnostic(const char* msg);

  // Called by SONAR sensor handler to publish a reading.
  static void publishSonar(uint8_t frame_id, float range);

  // Called by SONAR sensor handler to publish a reading.
  static void publishTemperature(const char* frame_id, float temperature);

  // Called by Time of Flight sensor handler to publish a reading.
  static void publishTof(uint8_t frame_id, float range);

  // Singleton constructor.
  static TMicroRos& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule
  const char* name() { return "uRos"; }

  // From TModule
  void setup();

 private:
  // Private constructor.
  TMicroRos();

  bool create_entities();

  void destroy_entities();


  // Regular maintenance, publish stats, etc.
  static void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

  // Handler for cmd_vel messages.
  static void twist_callback(const void* msg);

  // Micro-ROS variables
  rcl_allocator_t allocator_;
  rcl_subscription_t cmd_vel_subscriber_;
  rcl_publisher_t diagnostics_publisher_;
  rclc_executor_t executor_;
  bool micro_ros_init_successful_;
  std_msgs__msg__String msg_;
  rcl_node_t node_;
  rcl_publisher_t roboclaw_status_publisher_;
  rclc_support_t support_;
  rcl_publisher_t teensy_stats_publisher_;
  rcl_timer_t timer_;
  rcl_publisher_t sonar_publisher_[4];
  rcl_publisher_t temperature_publisher_;
  rcl_publisher_t tof_publisher_[8];

  // ROS messages, allocated once.
  sensor_msgs__msg__Range sonar_range_msg_;
  sensor_msgs__msg__Temperature temperature_msg_;
  sensor_msgs__msg__Range tof_range_msg_;
  geometry_msgs__msg__Twist twist_msg_;

  // Motor driver configuration values.
  int32_t accel_quad_pulses_per_second_;
  double max_angular_velocity_;
  double max_linear_velocity_;
  double max_seconds_uncommanded_travel_;
  int32_t quad_pulses_per_meter_;
  double wheel_radius_;
  double wheel_separation_;

  // Singleton instance.
  static TMicroRos* g_singleton;
};
