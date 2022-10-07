#include "tmicro_ros.h"

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>

//#include <micro_ros_arduino.h>
//#include <yaml.h>
//
//#include "tmicro_ros.h"
//
//#include <rcl.h>
//#include <rclc/rclc.h>
//#include <rclc/executor.h>

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)\

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }

void TMicroRos::loop() {
  // Serial.print("State: ");Serial.println(state_);
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_AVAILABLE
                                      : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state_ == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(1, state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_CONNECTED
                                        : AGENT_DISCONNECTED;);
      if (state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void TMicroRos::setup() {
  set_microros_transports();
  state_ = WAITING_AGENT;
  sequence_number_ = 0;
  msg_.data.capacity = 256;
  msg_.data.size = 256;
}

void TMicroRos::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    static const size_t STR_SIZE = 256;
    char message[STR_SIZE];
    g_singleton->msg_.data.size = sprintf(message, "%ld", g_singleton->sequence_number_++);
    g_singleton->msg_.data.capacity = g_singleton->msg_.data.size + 1;
    g_singleton->msg_.data.data = message;
    rcl_ret_t ignore = rcl_publish(&g_singleton->publisher_, &g_singleton->msg_, nullptr);
    // Serial.print("Serial number: ");Serial.println(g_singleton->sequence_number_);
  }
}

TMicroRos::TMicroRos() : TModule() {}

bool TMicroRos::create_entities() {
  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator_));

  // create node
  RCCHECK(
      rclc_node_init_default(&node_, "teensy_publisher", "", &support_));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "teensy_sensors"));

  // create timer,
  const unsigned int timer_timeout = 5;
  RCCHECK(rclc_timer_init_default(&timer_, &support_,
                                  RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));

  return true;
}

void TMicroRos::destroy_entities() {
  rcl_ret_t ignore;
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore = rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  ignore = rcl_publisher_fini(&publisher_, &node_);
  ignore = rcl_timer_fini(&timer_);
  ignore = rclc_executor_fini(&executor_);
  ignore = rcl_node_fini(&node_);
  ignore = rclc_support_fini(&support_);
}

TMicroRos &TMicroRos::singleton() {
  if (!g_singleton) {
    g_singleton = new TMicroRos();
  }

  return *g_singleton;
}

TMicroRos *TMicroRos::g_singleton = nullptr;
