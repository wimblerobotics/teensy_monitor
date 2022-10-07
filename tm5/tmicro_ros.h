#pragma once

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

#include "tmodule.h"

class TMicroRos : TModule {
 public:
  // From TModule.
  void loop();

  // From TModule
  const char* name() { return "TMicroRos"; }

  // From TModule
  void setup();

  // Singleton constructor.
  static TMicroRos& singleton();

 private:
  enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state_;

  // Private constructor.
  TMicroRos();

  bool create_entities();

  void destroy_entities();

  static void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

  rcl_allocator_t allocator_;
  rclc_executor_t executor_;
  bool micro_ros_init_successful_;
  std_msgs__msg__String msg_;
  rcl_node_t node_;
  rcl_publisher_t publisher_;
  uint32_t sequence_number_;
  rclc_support_t support_;
  rcl_timer_t timer_;

  // Singleton instance.
  static TMicroRos* g_singleton;
};
