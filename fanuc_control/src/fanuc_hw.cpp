#include "fanuc_control/fanuc_hw.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fanuc
{
namespace
{
void to_lower_inplace(std::string & text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
}

std::string get_hw_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback = "")
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return it->second;
}
}  // namespace

JointComms::JointComms() : Node("fanuc_hw")
{
  cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/cmd_j_pos", 10);
  fb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/fb_j_pos", 10);
  cart_fb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/fb_c_pos", 10);
}

CallbackReturn FanucHw::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(logger_, "init fanuc_hw");
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  std::string ro = get_hw_param(info_, "read_only", "false");
  to_lower_inplace(ro);
  RCLCPP_INFO_STREAM(logger_, "\n RO::" << ro);
  read_only_ = (ro == "true");
  if (read_only_) {
    RCLCPP_INFO_STREAM(
      logger_,
      "\n read only mode active. the robot can be moved from this hardware interface ");
  }

  std::string rmi = get_hw_param(info_, "use_rmi", "false");
  to_lower_inplace(rmi);
  useRMI_ = (rmi == "true");

  if (useRMI_) {
    RCLCPP_INFO_STREAM(logger_, "\n Using RMI to control the robot !   ");
  } else {
    RCLCPP_INFO_STREAM(logger_, "\n Using Ethernet/IP to control the robot !   ");
  }

  const std::string robot_ip = get_hw_param(info_, "robot_ip");
  RCLCPP_INFO_STREAM(logger_, "\n\n\nIP : " << robot_ip << "\n\n\n\n ");

  if (useRMI_) {
    if (!rmi_driver_.init(robot_ip, 6)) {
      RCLCPP_ERROR_STREAM(logger_, "RMI non initialized. Robot ip: " << robot_ip);
    }
    RCLCPP_INFO_STREAM(logger_, "RMI  initialized. Robot ip: " << robot_ip);
  } else {
    EIP_driver_ = std::make_shared<fanuc_eth_ip>(robot_ip);
    RCLCPP_INFO_STREAM(logger_, "Initialized EIP driver at ip: " << robot_ip);
  }

  std::vector<double> j_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  joint_position_.resize(6, 0.0);
  joint_position_prev_.resize(6, 0.0);
  joint_velocities_.resize(6, 0.0);
  joint_position_command_.resize(6, 0.0);

  joint_names_.resize(joint_position_.size());
  for (size_t j = 0; j < joint_position_.size(); ++j) {
    joint_names_.at(j) = info_.joints[j].name;
    RCLCPP_DEBUG_STREAM(logger_, info_.joints[j].name);
  }

  if (useRMI_) {
    j_pos = rmi_driver_.getPosition();
    for (size_t i = 0; i < j_pos.size(); i++) {
      RCLCPP_WARN_STREAM(logger_, j_pos.at(i));
    }
  } else {
    j_pos = EIP_driver_->get_current_joint_pos();
    EIP_driver_->setCurrentPos();
  }

  for (size_t i = 0; i < joint_position_.size(); i++) {
    joint_position_command_.at(i) = j_pos.at(i);
  }

  if (useRMI_) {
    rmi_driver_.setTargetPosition(joint_position_command_);
  } else {
    EIP_driver_->write_register(1, 1);
  }

  comms_ = std::make_shared<JointComms>();
  if (auto exec = params.executor.lock()) {
    exec->add_node(comms_);
  } else {
    executor_.add_node(comms_);
    std::thread([this]() { executor_.spin(); }).detach();
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FanucHw::export_state_interfaces()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < 6; ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_[i]);
  }
  for (size_t i = 0; i < 6; ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_velocities_[i]);
  }
  return state_interfaces;
#pragma GCC diagnostic pop
}

std::vector<hardware_interface::CommandInterface> FanucHw::export_command_interfaces()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_command_[i]);
  }
  return command_interfaces;
#pragma GCC diagnostic pop
}

return_type FanucHw::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const auto begin = std::chrono::steady_clock::now();
  std::vector<double> jp(6, 0.0);
  std::vector<double> cp(6, 0.0);

  if (useRMI_) {
    jp = rmi_driver_.getPosition();

    rclcpp::Rate rate2(100);
    while (!rmi_driver_.instruction_parsed_ && rclcpp::ok()) {
      rate2.sleep();
    }
    rmi_driver_.instruction_parsed_ = false;
  } else {
    jp = EIP_driver_->get_current_joint_pos();
    cp = EIP_driver_->get_current_pose();
  }

  const double dt = period.seconds();
  for (size_t j = 0; j < joint_position_command_.size(); ++j) {
    joint_position_[j] = jp[j];
    joint_velocities_[j] = (dt > 0.0) ? (jp[j] - joint_position_prev_[j]) / dt : 0.0;
    joint_position_prev_[j] = jp[j];
    RCLCPP_DEBUG_STREAM(logger_, jp[j]);
  }

  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = comms_->get_clock()->now();
    msg.name = joint_names_;
    msg.position = joint_position_;
    msg.velocity = joint_velocities_;
    comms_->fb_pub_->publish(msg);
  }
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = comms_->get_clock()->now();
    msg.name = {"x", "y", "z", "Rx", "Ry", "Rz"};
    msg.position = cp;
    comms_->cart_fb_pub_->publish(msg);
  }

  const auto end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG_STREAM(
    logger_, "READ time:  = "
               << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
               << "[microseconds]");

  return return_type::OK;
}

return_type FanucHw::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto begin = std::chrono::steady_clock::now();
  if (!read_only_) {
    if (useRMI_) {
      rmi_driver_.setTargetPosition(joint_position_command_);
    } else {
      if (EIP_driver_->read_register(RegisterEnum::MotionStatus) == StatusEnum::Ros) {
        RCLCPP_DEBUG_STREAM(logger_, "DPM INACTIVE");
        EIP_driver_->write_pos_register(joint_position_command_);
      } else {
        joint_position_command_ = joint_position_;
        EIP_driver_->write_pos_register(joint_position_command_);
        RCLCPP_DEBUG_STREAM(logger_, "DPM ACTIVE");
      }
    }
  }

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = comms_->get_clock()->now();
  msg.name = joint_names_;
  msg.position = joint_position_command_;
  comms_->cmd_pub_->publish(msg);
  const auto end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG_STREAM(
    logger_, "WRITE time:  = "
               << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
               << "[microseconds]");

  return return_type::OK;
}

}  // namespace fanuc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fanuc::FanucHw, hardware_interface::SystemInterface)
