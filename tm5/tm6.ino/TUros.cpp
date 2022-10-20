#include "TURos.h"

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#define ignore_result(x) \
  if (x) {               \
  }

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
  } while (0)

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }

void TUros::loop() {
  rmw_uros_sync_session(1000);
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(10,
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
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }

  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
}

void TUros::setup() {
  set_microros_transports();
  state_ = WAITING_AGENT;
}

void TUros::timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    // uint32_t error = TRoboClaw::singleton().getError();
    const size_t MAXSIZE = 512;
    char stats[MAXSIZE];
    static const sequence_number = 0;
    TModule::getStatistics(stats, MAXSIZE);
    int64_t period;
    ignore_result((timer, &period));
    snprintf(g_singleton->msg_.data.data, g_singleton->msg_.data.capacity,
             "Sequence: %d, period: %lld, last_call_time: %lld",
             sequence_number++, period, last_call_time);
    g_singleton->msg_.data.size = strlen(g_singleton->msg_.data.data);
    ignore_result(
        rcl_publish(&g_singleton->publisher_, &g_singleton->msg_, nullptr));
  }
}

TUros::TUros() : TModule(TModule::kMICRO_ROS) {
  msg_.data.capacity = 512;
  msg_.data.data = (char *)malloc(msg_.data.capacity * sizeof(char));
  msg_.data.size = 0;
}

bool TUros::create_entities() {
  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator_));

  // create node
  RCCHECK(rclc_node_init_default(&node_, "teensy_node", "", &support_));

  // create publishers.
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy_stats"));

  // Create timer,
  const unsigned int timer_timeout_ns = 1'000'000'000;
  RCCHECK(rclc_timer_init_default(&timer_, &support_, timer_timeout_ns,
                                  timer_callback));

  // Create executor
  executor_ = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 2, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));
  return true;
}

void TUros::destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_publisher_fini(&publisher_, &node_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rclc_executor_fini(&executor_));
  ignore_result(rcl_node_fini(&node_));
  ignore_result(rclc_support_fini(&support_));
}

TUros &TUros::singleton() {
  if (!g_singleton) {
    g_singleton = new TUros();
  }

  return *g_singleton;
}

TUros *TUros::g_singleton = nullptr;
