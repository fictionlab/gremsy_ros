// Copyright 2024 Fictionlab sp. z o.o.
// All rights reserved.

#include <chrono>
#include <memory>
#include <string>

#include "gremsy_sdk/gimbal_interface.hpp"
#include "gremsy_sdk/serial_port.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "gremsy_wrapper_parameters.hpp"

using namespace std::chrono_literals;

namespace gremsy_wrapper
{
// imu_raw_reading * imu_accel_scale = imu_accel in mili-g
// This number is never provided - this is an educated guess
constexpr double GIMBAL_IMU_ACCEL_SCALE = 0.00119634146;
constexpr int TIMEOUT_TRY_NUM = 10;
constexpr int ENCODER_OFFSET_ROLL = 8192;
constexpr int ENCODER_OFFSET_TILT = 21304;
constexpr int ENCODER_RESOLUTION = 32768;
constexpr double PI = 3.141592653589793238463;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double GIMBAL_ATTITUDE_LIMIT = 90.0;  // Degrees

class GremsyWrapper : public rclcpp::Node
{
  Serial_Port * serial_port_;
  Gimbal_Interface * gimbal_interface_;
  std::string com_port_;
  int baud_rate_;

  int gimbal_mode_;

  int tilt_hold_strength_;
  int tilt_stiffness_;
  int roll_hold_strength_;
  int roll_stiffness_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_rotation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // In radians
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_rotation_sub_;
  // In radians/s
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_velocity_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_lock_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;

  geometry_msgs::msg::Vector3Stamped::SharedPtr goal_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  ParamListener param_listener_;
  Params params_;

public:
  explicit GremsyWrapper(const rclcpp::NodeOptions & options)
  : Node ("gremsy_wrapper", options),
    param_listener_(get_node_parameters_interface())
  {
    goal_ = nullptr;

    this->update_parameters();

    // Publishers
    this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu_raw", 1);
    this->gimbal_rotation_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("~/rotation", 1);
    this->joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
        1);

    // Subscriptions
    this->gimbal_rotation_sub_ =
      this->create_subscription<geometry_msgs::msg::Vector3Stamped>("~/gimbal_goal", 1,
        std::bind(&GremsyWrapper::desired_rotation_callback, this, std::placeholders::_1));
    this->gimbal_velocity_sub_ =
      this->create_subscription<geometry_msgs::msg::Vector3Stamped>("~/gimbal_velocity", 1,
        std::bind(&GremsyWrapper::desired_velocity_callback, this, std::placeholders::_1));

    // Services
    this->enable_lock_mode_service_ = this->create_service<std_srvs::srv::SetBool>("~/lock_mode",
        std::bind(&GremsyWrapper::enable_lock_mode_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    this->return_home_service_ = this->create_service<std_srvs::srv::Trigger>("~/return_home",
        std::bind(&GremsyWrapper::return_home_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    this->reboot_service_ = this->create_service<std_srvs::srv::Trigger>("~/reboot",
        std::bind(&GremsyWrapper::reboot_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    serial_port_ = new Serial_Port(params_.com_port.c_str(), params_.baud_rate);
    gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, 191,
        Gimbal_Interface::MAVLINK_GIMBAL_V2, mavlink_channel_t::MAVLINK_COMM_0);

    serial_port_->start();
    gimbal_interface_->start();

    while (!gimbal_interface_->present()) {
      RCLCPP_INFO(this->get_logger(),
          "Gimbal interface not present, waiting for it to be connected");
      rclcpp::sleep_for(500ms);
    }

    this->setup_gimbal();

    gimbal_mode_ = params_.gimbal_mode_on_startup;

    this->set_gimbal_mode(gimbal_mode_);

    poll_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 /
        params_.state_poll_rate), std::bind(&GremsyWrapper::gimbal_state_timer_callback, this));
    goal_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 /
        params_.goal_push_rate), std::bind(&GremsyWrapper::gimbal_goal_timer_callback, this));
    check_timer_ = this->create_wall_timer(1s, std::bind(&GremsyWrapper::check_parameters, this));
  }

  ~GremsyWrapper()
  {
    serial_port_->stop();
    gimbal_interface_->stop();
  }

private:
  void update_parameters()
  {
    param_listener_.refresh_dynamic_parameters();
    params_ = param_listener_.get_params();
  }

  void check_parameters()
  {
    if (param_listener_.is_old(params_)) {
      this->update_parameters();
      this->set_gimbal_stiffness_params();
    }
  }

  void setup_gimbal()
  {
    Gimbal_Interface::fw_version_t fw = gimbal_interface_->get_gimbal_version();

    RCLCPP_INFO(this->get_logger(), "Gimbal Firmware version is is %d.%d.%d.%s", fw.x, fw.y, fw.z,
        fw.type);

    gimbal_interface_->set_msg_encoder_rate(params_.state_poll_rate);
    gimbal_interface_->set_msg_mnt_orient_rate(params_.state_poll_rate);
    gimbal_interface_->set_msg_attitude_status_rate(params_.state_poll_rate);
    gimbal_interface_->set_msg_raw_imu_rate(params_.state_poll_rate);

    gimbal_interface_->set_gimbal_encoder_type_send(true);
    gimbal_interface_->request_gimbal_device_info();

    this->set_gimbal_stiffness_params();
    this->set_gimbal_limits();

    RCLCPP_INFO(this->get_logger(), "Gimbal mode: %d", gimbal_interface_->get_gimbal_mode());
    RCLCPP_INFO(this->get_logger(), "Gimbal state: %d",
        gimbal_interface_->get_gimbal_status().state);

    auto angle_limit = gimbal_interface_->get_limit_angle_pitch();
    RCLCPP_INFO(this->get_logger(), "Pitch max: %d -- Pitch min: %d", angle_limit.angle_max,
        angle_limit.angle_min);
    angle_limit = gimbal_interface_->get_limit_angle_roll();
    RCLCPP_INFO(this->get_logger(), "Roll max: %d -- Roll min: %d", angle_limit.angle_max,
        angle_limit.angle_min);

    Gimbal_Interface::gimbal_motor_control_t tilt, roll, pan;
    uint8_t gyro_filter, output_filter;
    gimbal_interface_->get_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter);

    RCLCPP_INFO(this->get_logger(), "Tilt hold strength: %d -- Tilt stiffness: %d",
        tilt.holdstrength, tilt.stiffness);
    RCLCPP_INFO(this->get_logger(), "Roll hold strength: %d -- Roll stiffness: %d",
        roll.holdstrength, roll.stiffness);
    RCLCPP_INFO(this->get_logger(), "Gyro filter: %d", gyro_filter);
    RCLCPP_INFO(this->get_logger(), "Output filter: %d\n", output_filter);
  }

  void set_gimbal_stiffness_params()
  {
    Gimbal_Interface::gimbal_motor_control_t tilt = {params_.tilt_stiffness,
      params_.tilt_hold_strength};
    Gimbal_Interface::gimbal_motor_control_t roll = {params_.roll_stiffness,
      params_.roll_hold_strength};
    // Pan axis not used in tilt-roll 2-axis gimbal - using default values
    Gimbal_Interface::gimbal_motor_control_t pan = {50, 30};
    const uint8_t gyro_filter = params_.gyro_filter;
    const uint8_t output_filter = params_.output_filter;
    gimbal_interface_->set_gimbal_motor_control(tilt, roll, pan, gyro_filter, output_filter);
  }

  void set_gimbal_limits()
  {
    Gimbal_Interface::limit_angle_t limits;
    limits.angle_min = params_.tilt_min * RAD_TO_DEG;
    limits.angle_max = params_.tilt_max * RAD_TO_DEG;
    gimbal_interface_->set_limit_angle_pitch(limits);
    limits.angle_min = params_.roll_min * RAD_TO_DEG;
    limits.angle_max = params_.roll_max * RAD_TO_DEG;
    gimbal_interface_->set_limit_angle_roll(limits);
  }

  void set_gimbal_mode(int mode)
  {
    uint8_t timeout = 0;
    Gimbal_Protocol::result_t res;
    switch (mode) {
      case 1:
        do{
          res = gimbal_interface_->set_gimbal_lock_mode_sync();
          if(timeout++ > TIMEOUT_TRY_NUM) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gimbal lock mode");
            return;
          }
        }while(res != Gimbal_Protocol::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Gimbal lock mode set");
        break;
      case 2:
        do{
          res = gimbal_interface_->set_gimbal_follow_mode_sync();
          if(timeout++ > TIMEOUT_TRY_NUM) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gimbal follow mode");
            return;
          }
        }while(res != Gimbal_Protocol::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Gimbal follow mode set");
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "Trying to set invalid gimbal mode");
        break;
    }
  }

  void gimbal_state_timer_callback()
  {
    rclcpp::Time stamp = this->get_clock()->now();

    // Create and publish IMU message
    auto imu_raw = gimbal_interface_->get_gimbal_raw_imu();
    sensor_msgs::msg::Imu imu_ros_msg = convert_mavlink_imu_to_ros_msg(imu_raw);
    imu_ros_msg.header.stamp = stamp;
    imu_pub_->publish(imu_ros_msg);

    // Read and publish local motor pose (in radians)
    auto attitude = gimbal_interface_->get_gimbal_attitude();
    geometry_msgs::msg::Vector3Stamped rotation;

    if (abs(attitude.roll) > GIMBAL_ATTITUDE_LIMIT) {
      attitude.roll -= copysign(GIMBAL_ATTITUDE_LIMIT * 2, attitude.roll);

      attitude.pitch = copysign(GIMBAL_ATTITUDE_LIMIT * 2, attitude.pitch) - attitude.pitch;
    }

    // Pitch axis in inverted in our robot model
    rotation.vector.x = attitude.roll * DEG_TO_RAD;
    rotation.vector.y = -1 * attitude.pitch * DEG_TO_RAD;
    rotation.vector.z = attitude.yaw * DEG_TO_RAD;
    rotation.header.stamp = stamp;
    rotation.header.frame_id = "gremsy_base_link";
    gimbal_rotation_pub_->publish(rotation);

    // Publish joint state based on global encoder readings
    auto encoder_attitude = gimbal_interface_->get_gimbal_encoder();
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name = {"gremsy_tilt_joint", "gremsy_roll_joint"};
    joint_state.position = {get_angle_from_encoder(encoder_attitude.pitch, ENCODER_OFFSET_TILT),
      get_angle_from_encoder(encoder_attitude.roll, ENCODER_OFFSET_ROLL)};
    joint_state_pub_->publish(joint_state);
  }

  void gimbal_goal_timer_callback()
  {
    if (!goal_) {
      return;
    }

    // Pitch, roll, yaw
    Gimbal_Protocol::result_t res = gimbal_interface_->set_gimbal_rotation_sync(goal_->vector.y,
        goal_->vector.x, goal_->vector.z);

    if (res == Gimbal_Protocol::SUCCESS) {
      attitude<float> cur_attitude = gimbal_interface_->get_gimbal_attitude();
      if ((abs(cur_attitude.pitch - goal_->vector.y) <= 0.5f) &&
        (abs(cur_attitude.roll - goal_->vector.x) <= 0.5f))
      {
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        goal_ = nullptr;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to control gimbal");
    }
  }

  sensor_msgs::msg::Imu convert_mavlink_imu_to_ros_msg(Gimbal_Interface::imu_t raw_imu)
  {
    sensor_msgs::msg::Imu imu_msg;
    // Convert accelerometer data (from milli-g to m/s²)
    imu_msg.linear_acceleration.x = (raw_imu.accel.x) * GIMBAL_IMU_ACCEL_SCALE;
    imu_msg.linear_acceleration.y = (raw_imu.accel.y) * GIMBAL_IMU_ACCEL_SCALE;
    imu_msg.linear_acceleration.z = (raw_imu.accel.z) * GIMBAL_IMU_ACCEL_SCALE;

    // Convert gyroscope data (from milliradians/s to radians/s)
    imu_msg.angular_velocity.x = (raw_imu.gyro.x) / 1000.0;
    imu_msg.angular_velocity.y = (raw_imu.gyro.y) / 1000.0;
    imu_msg.angular_velocity.z = (raw_imu.gyro.z) / 1000.0;

    // Imu does not provide orientation - fill with defaults
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1;

    return imu_msg;
  }

  // In Rad
  double get_angle_from_encoder(int16_t encoder_value, int16_t encoder_offset)
  {
    int offset_value = encoder_value + encoder_offset;
    // Make sure that offset value is within the encoder resolution
    offset_value = (offset_value + ENCODER_RESOLUTION) % (2 * ENCODER_RESOLUTION) -
      ENCODER_RESOLUTION;

    return PI * static_cast<double>(offset_value) / static_cast<double>(ENCODER_RESOLUTION);
  }

  void desired_rotation_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    // Limit roll and tilt
    msg->vector.x = std::fmin(std::fmax(msg->vector.x, params_.roll_min), params_.roll_max);
    msg->vector.y = std::fmin(std::fmax(msg->vector.y, params_.tilt_min), params_.tilt_max);
    msg->vector.z = 0;  // Pan axis not used in tilt-roll 2-axis gimbal

    RCLCPP_INFO(this->get_logger(), "New goal received: x: '%.2f', y: '%.2f', z: '%.2f'",
        msg->vector.x, msg->vector.y, msg->vector.z);

    msg->vector.x *= RAD_TO_DEG;
    msg->vector.y *= RAD_TO_DEG;
    msg->vector.z *= RAD_TO_DEG;
    goal_ = msg;
  }

  void desired_velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    // PI rad/s is the maximum speed for the gimbal
    msg->vector.x = std::fmin(std::fmax(msg->vector.x, -PI), PI);
    msg->vector.y = std::fmin(std::fmax(msg->vector.y, -PI), PI);
    msg->vector.z = 0;

    RCLCPP_INFO(this->get_logger(),
        "New joint velocities requested: x: '%.2f', y: '%.2f', z: '%.2f'", msg->vector.x,
        msg->vector.y, msg->vector.z);
    // Disable goal if velocity is set
    goal_ = nullptr;

    gimbal_interface_->set_gimbal_rotation_rate_sync(msg->vector.y * RAD_TO_DEG,
      msg->vector.x * RAD_TO_DEG, msg->vector.z * RAD_TO_DEG);
  }

  void enable_lock_mode_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    int new_mode = request->data ? 1 : 2;

    if (new_mode == gimbal_mode_) {
      response->success = true;
      response->message = "Gimbal is already in requested mode.";

      RCLCPP_INFO(this->get_logger(), "Gimbal mode unchanged, is already in %s mode.",
          gimbal_mode_ == 1 ? "lock" : "follow");
    } else {
      gimbal_mode_ = new_mode;
      this->set_gimbal_mode(gimbal_mode_);

      response->success = true;
      response->message = "Gimbal mode successfully changed.";

      RCLCPP_INFO(this->get_logger(), "Changing gimbal mode to %s.",
          gimbal_mode_ == 1 ? "lock" : "follow");
    }
  }

  void return_home_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    goal_ = nullptr;
    Gimbal_Protocol::result_t res = gimbal_interface_->set_gimbal_return_home_sync();

    if (res == Gimbal_Protocol::SUCCESS) {
      attitude<float> cur_attitude = gimbal_interface_->get_gimbal_attitude();
      int timeout = 0;

      while (cur_attitude.pitch > 0.5f || cur_attitude.roll > 0.5f) {
        if (timeout++ > TIMEOUT_TRY_NUM) {
          response->success = false;
          response->message = "Failed to return gimbal to home position in time";
          RCLCPP_ERROR(this->get_logger(), "Failed to return gimbal to home position in time");
          return;
        }
        rclcpp::sleep_for(500ms);
        cur_attitude = gimbal_interface_->get_gimbal_attitude();
      }
      response->success = true;
      response->message = "Gimbal returned to home position.";
      RCLCPP_INFO(this->get_logger(), "Gimbal returned to home position.");
    } else {
      response->success = false;
      response->message = "Failed to send return home command .";
      RCLCPP_ERROR(this->get_logger(), "Failed to send return home command.");
    }
  }

  void reboot_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (gimbal_interface_->set_gimbal_reboot() == Gimbal_Protocol::SUCCESS) {
      // Wait for reboot to complete
      while (gimbal_interface_->get_gimbal_status().state != Gimbal_Interface::GIMBAL_STATE_ON) {
        rclcpp::sleep_for(500ms);
      }
      goal_ = nullptr;

      response->success = true;
      response->message = "Gimbal rebooted.";
      RCLCPP_INFO(this->get_logger(), "Gimbal rebooted.");
    } else {
      response->success = false;
      response->message = "Failed to send reboot command.";
      RCLCPP_ERROR(this->get_logger(), "Failed to send reboot command.");
    }
  }
};

}  // namespace gremsy_wrapper

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gremsy_wrapper::GremsyWrapper)
