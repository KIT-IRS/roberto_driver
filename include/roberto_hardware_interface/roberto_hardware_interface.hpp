#ifndef ROBERTO_HARDWARE_INTERFACE_HPP_
#define ROBERTO_HARDWARE_INTERFACE_HPP_

#include "tendon.hpp"
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace roberto_hardware_interface {

class RobertoHardwareInterface : public hardware_interface::SystemInterface {

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobertoHardwareInterface)

  RobertoHardwareInterface()
      : logger_(rclcpp::get_logger("roberto_hardware_interface")) {};

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  inline rclcpp::Logger get_logger() const { return logger_; };

private:
  void subscriber_callback(std_msgs::msg::Float64MultiArray msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      uros_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      uros_subscription_;

  std::vector<double> last_joint_state_ = {1.0, 1.0, 1.0};
};

} // namespace roberto_hardware_interface

#endif
