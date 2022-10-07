// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs:msg/TeensySensors.idl
// generated code does not contain a copyright notice

#ifndef MSGS__MSG__DETAIL__TEENSY_SENSORS__STRUCT_HPP_
#define MSGS__MSG__DETAIL__TEENSY_SENSORS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__msgs__msg__TeensySensors __attribute__((deprecated))
#else
# define DEPRECATED__msgs__msg__TeensySensors __declspec(deprecated)
#endif

namespace msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TeensySensors_
{
  using Type = TeensySensors_<ContainerAllocator>;

  explicit TeensySensors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence_number = 0l;
      this->motor_current_left_ma = 0;
      this->motor_current_right_ma = 0;
      this->sonar_front_mm = 0;
      this->sonar_right_mm = 0;
      this->sonar_back_mm = 0;
      this->sonar_left_mm = 0;
      this->temp_motor_left_tenthsc = 0;
      this->temp_motor_right_tenthsc = 0;
      this->tof_front_left_forwards_mm = 0;
      this->tof_front_right_forwards_mm = 0;
      this->tof_center_left_sideways_mm = 0;
      this->tof_center_right_sideways_mm = 0;
      this->tof_back_left_sideways_mm = 0;
      this->tof_back_right_sideways_mm = 0;
      this->tof_back_left_backwards_mm = 0;
      this->tof_back_right_backwards_mm = 0;
    }
  }

  explicit TeensySensors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sequence_number = 0l;
      this->motor_current_left_ma = 0;
      this->motor_current_right_ma = 0;
      this->sonar_front_mm = 0;
      this->sonar_right_mm = 0;
      this->sonar_back_mm = 0;
      this->sonar_left_mm = 0;
      this->temp_motor_left_tenthsc = 0;
      this->temp_motor_right_tenthsc = 0;
      this->tof_front_left_forwards_mm = 0;
      this->tof_front_right_forwards_mm = 0;
      this->tof_center_left_sideways_mm = 0;
      this->tof_center_right_sideways_mm = 0;
      this->tof_back_left_sideways_mm = 0;
      this->tof_back_right_sideways_mm = 0;
      this->tof_back_left_backwards_mm = 0;
      this->tof_back_right_backwards_mm = 0;
    }
  }

  // field types and members
  using _sequence_number_type =
    int32_t;
  _sequence_number_type sequence_number;
  using _motor_current_left_ma_type =
    int16_t;
  _motor_current_left_ma_type motor_current_left_ma;
  using _motor_current_right_ma_type =
    int16_t;
  _motor_current_right_ma_type motor_current_right_ma;
  using _sonar_front_mm_type =
    int16_t;
  _sonar_front_mm_type sonar_front_mm;
  using _sonar_right_mm_type =
    int16_t;
  _sonar_right_mm_type sonar_right_mm;
  using _sonar_back_mm_type =
    int16_t;
  _sonar_back_mm_type sonar_back_mm;
  using _sonar_left_mm_type =
    int16_t;
  _sonar_left_mm_type sonar_left_mm;
  using _temp_motor_left_tenthsc_type =
    int16_t;
  _temp_motor_left_tenthsc_type temp_motor_left_tenthsc;
  using _temp_motor_right_tenthsc_type =
    int16_t;
  _temp_motor_right_tenthsc_type temp_motor_right_tenthsc;
  using _tof_front_left_forwards_mm_type =
    int16_t;
  _tof_front_left_forwards_mm_type tof_front_left_forwards_mm;
  using _tof_front_right_forwards_mm_type =
    int16_t;
  _tof_front_right_forwards_mm_type tof_front_right_forwards_mm;
  using _tof_center_left_sideways_mm_type =
    int16_t;
  _tof_center_left_sideways_mm_type tof_center_left_sideways_mm;
  using _tof_center_right_sideways_mm_type =
    int16_t;
  _tof_center_right_sideways_mm_type tof_center_right_sideways_mm;
  using _tof_back_left_sideways_mm_type =
    int16_t;
  _tof_back_left_sideways_mm_type tof_back_left_sideways_mm;
  using _tof_back_right_sideways_mm_type =
    int16_t;
  _tof_back_right_sideways_mm_type tof_back_right_sideways_mm;
  using _tof_back_left_backwards_mm_type =
    int16_t;
  _tof_back_left_backwards_mm_type tof_back_left_backwards_mm;
  using _tof_back_right_backwards_mm_type =
    int16_t;
  _tof_back_right_backwards_mm_type tof_back_right_backwards_mm;

  // setters for named parameter idiom
  Type & set__sequence_number(
    const int32_t & _arg)
  {
    this->sequence_number = _arg;
    return *this;
  }
  Type & set__motor_current_left_ma(
    const int16_t & _arg)
  {
    this->motor_current_left_ma = _arg;
    return *this;
  }
  Type & set__motor_current_right_ma(
    const int16_t & _arg)
  {
    this->motor_current_right_ma = _arg;
    return *this;
  }
  Type & set__sonar_front_mm(
    const int16_t & _arg)
  {
    this->sonar_front_mm = _arg;
    return *this;
  }
  Type & set__sonar_right_mm(
    const int16_t & _arg)
  {
    this->sonar_right_mm = _arg;
    return *this;
  }
  Type & set__sonar_back_mm(
    const int16_t & _arg)
  {
    this->sonar_back_mm = _arg;
    return *this;
  }
  Type & set__sonar_left_mm(
    const int16_t & _arg)
  {
    this->sonar_left_mm = _arg;
    return *this;
  }
  Type & set__temp_motor_left_tenthsc(
    const int16_t & _arg)
  {
    this->temp_motor_left_tenthsc = _arg;
    return *this;
  }
  Type & set__temp_motor_right_tenthsc(
    const int16_t & _arg)
  {
    this->temp_motor_right_tenthsc = _arg;
    return *this;
  }
  Type & set__tof_front_left_forwards_mm(
    const int16_t & _arg)
  {
    this->tof_front_left_forwards_mm = _arg;
    return *this;
  }
  Type & set__tof_front_right_forwards_mm(
    const int16_t & _arg)
  {
    this->tof_front_right_forwards_mm = _arg;
    return *this;
  }
  Type & set__tof_center_left_sideways_mm(
    const int16_t & _arg)
  {
    this->tof_center_left_sideways_mm = _arg;
    return *this;
  }
  Type & set__tof_center_right_sideways_mm(
    const int16_t & _arg)
  {
    this->tof_center_right_sideways_mm = _arg;
    return *this;
  }
  Type & set__tof_back_left_sideways_mm(
    const int16_t & _arg)
  {
    this->tof_back_left_sideways_mm = _arg;
    return *this;
  }
  Type & set__tof_back_right_sideways_mm(
    const int16_t & _arg)
  {
    this->tof_back_right_sideways_mm = _arg;
    return *this;
  }
  Type & set__tof_back_left_backwards_mm(
    const int16_t & _arg)
  {
    this->tof_back_left_backwards_mm = _arg;
    return *this;
  }
  Type & set__tof_back_right_backwards_mm(
    const int16_t & _arg)
  {
    this->tof_back_right_backwards_mm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs::msg::TeensySensors_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs::msg::TeensySensors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs::msg::TeensySensors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs::msg::TeensySensors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs::msg::TeensySensors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs::msg::TeensySensors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs::msg::TeensySensors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs::msg::TeensySensors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs::msg::TeensySensors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs::msg::TeensySensors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs__msg__TeensySensors
    std::shared_ptr<msgs::msg::TeensySensors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs__msg__TeensySensors
    std::shared_ptr<msgs::msg::TeensySensors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TeensySensors_ & other) const
  {
    if (this->sequence_number != other.sequence_number) {
      return false;
    }
    if (this->motor_current_left_ma != other.motor_current_left_ma) {
      return false;
    }
    if (this->motor_current_right_ma != other.motor_current_right_ma) {
      return false;
    }
    if (this->sonar_front_mm != other.sonar_front_mm) {
      return false;
    }
    if (this->sonar_right_mm != other.sonar_right_mm) {
      return false;
    }
    if (this->sonar_back_mm != other.sonar_back_mm) {
      return false;
    }
    if (this->sonar_left_mm != other.sonar_left_mm) {
      return false;
    }
    if (this->temp_motor_left_tenthsc != other.temp_motor_left_tenthsc) {
      return false;
    }
    if (this->temp_motor_right_tenthsc != other.temp_motor_right_tenthsc) {
      return false;
    }
    if (this->tof_front_left_forwards_mm != other.tof_front_left_forwards_mm) {
      return false;
    }
    if (this->tof_front_right_forwards_mm != other.tof_front_right_forwards_mm) {
      return false;
    }
    if (this->tof_center_left_sideways_mm != other.tof_center_left_sideways_mm) {
      return false;
    }
    if (this->tof_center_right_sideways_mm != other.tof_center_right_sideways_mm) {
      return false;
    }
    if (this->tof_back_left_sideways_mm != other.tof_back_left_sideways_mm) {
      return false;
    }
    if (this->tof_back_right_sideways_mm != other.tof_back_right_sideways_mm) {
      return false;
    }
    if (this->tof_back_left_backwards_mm != other.tof_back_left_backwards_mm) {
      return false;
    }
    if (this->tof_back_right_backwards_mm != other.tof_back_right_backwards_mm) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeensySensors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TeensySensors_

// alias to use template instance with default allocator
using TeensySensors =
  msgs::msg::TeensySensors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs

#endif  // MSGS__MSG__DETAIL__TEENSY_SENSORS__STRUCT_HPP_
