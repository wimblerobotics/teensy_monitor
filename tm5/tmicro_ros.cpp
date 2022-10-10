#include "tmicro_ros.h"

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <stdio.h>

#define ignore_result(x) \
  if (x) {               \
  }

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
  } while (0)

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

void TMicroRos::publishSonar(uint8_t frame_id, float range) {
  if (g_singleton->state_ == AGENT_CONNECTED) {
    snprintf(g_singleton->sonar_range_msg_.header.frame_id.data,
             g_singleton->sonar_range_msg_.header.frame_id.capacity, "sonar_%1d",
             frame_id);
    g_singleton->sonar_range_msg_.header.frame_id.size =
        strlen(g_singleton->sonar_range_msg_.header.frame_id.data);
    g_singleton->sonar_range_msg_.radiation_type = 0;
    g_singleton->sonar_range_msg_.field_of_view = 0.523599;  // 30 degrees.
    g_singleton->sonar_range_msg_.max_range = 2.0;
    g_singleton->sonar_range_msg_.min_range = 0.0254;
    g_singleton->sonar_range_msg_.radiation_type =
        sensor_msgs__msg__Range__ULTRASOUND;
    g_singleton->sonar_range_msg_.range = range;
    ignore_result(
        rcl_publish(&g_singleton->sonar_publisher_, &g_singleton->sonar_range_msg_, nullptr));
  }
}

void TMicroRos::publishTof(uint8_t frame_id, float range) {
  if (g_singleton->state_ == AGENT_CONNECTED) {
    snprintf(g_singleton->tof_range_msg_.header.frame_id.data,
             g_singleton->tof_range_msg_.header.frame_id.capacity, "tof_%1d",
             frame_id);
    g_singleton->tof_range_msg_.header.frame_id.size =
        strlen(g_singleton->tof_range_msg_.header.frame_id.data);
    g_singleton->tof_range_msg_.radiation_type = 0;
    g_singleton->tof_range_msg_.field_of_view = 0.436332;  // 25 degrees
    g_singleton->tof_range_msg_.max_range = 2.0;
    g_singleton->tof_range_msg_.min_range = 0.0254;
    g_singleton->tof_range_msg_.radiation_type =
        sensor_msgs__msg__Range__ULTRASOUND;
    g_singleton->tof_range_msg_.range = range;
    ignore_result(
        rcl_publish(&g_singleton->tof_publisher_, &g_singleton->tof_range_msg_, nullptr));
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
    g_singleton->msg_.data.size =
        sprintf(message, "%ld", g_singleton->sequence_number_++);
    g_singleton->msg_.data.capacity = g_singleton->msg_.data.size + 1;
    g_singleton->msg_.data.data = message;
    ignore_result(
        rcl_publish(&g_singleton->publisher_, &g_singleton->msg_, nullptr));
    // Serial.print("Serial number:
    // ");Serial.println(g_singleton->sequence_number_);
  }
}

TMicroRos::TMicroRos() : TModule() {
  sonar_range_msg_.header.frame_id.capacity = 32;
  sonar_range_msg_.header.frame_id.data =
      (char *)malloc(sonar_range_msg_.header.frame_id.capacity * sizeof(char));
  sonar_range_msg_.header.frame_id.size = 0;
  tof_range_msg_.header.frame_id.capacity = 32;
  tof_range_msg_.header.frame_id.data =
      (char *)malloc(sonar_range_msg_.header.frame_id.capacity * sizeof(char));
  tof_range_msg_.header.frame_id.size = 0;
}

bool TMicroRos::create_entities() {
  allocator_ = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support_, 0, nullptr, &allocator_));

  // create node
  RCCHECK(rclc_node_init_default(&node_, "teensy_publisher", "", &support_));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy_sensors"));

  RCCHECK(rclc_publisher_init_best_effort(
      &sonar_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "sonarSensor"));

  RCCHECK(rclc_publisher_init_best_effort(
      &tof_publisher_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "tofSensor"));

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
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support_.context);
  ignore_result(
      rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

  ignore_result(rcl_publisher_fini(&publisher_, &node_));
  ignore_result(rcl_publisher_fini(&sonar_publisher_, &node_));
  ignore_result(rcl_publisher_fini(&tof_publisher_, &node_));
  ignore_result(rcl_timer_fini(&timer_));
  ignore_result(rclc_executor_fini(&executor_));
  ignore_result(rcl_node_fini(&node_));
  ignore_result(rclc_support_fini(&support_));
}

TMicroRos &TMicroRos::singleton() {
  if (!g_singleton) {
    g_singleton = new TMicroRos();
  }

  return *g_singleton;
}

TMicroRos *TMicroRos::g_singleton = nullptr;
