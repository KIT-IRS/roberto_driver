#include "irs_softrobot_hardware_interface.hpp"

namespace irs_softrobot_hardware_interface {

// IRSSoftrobotHardwareInterface::IRSSoftrobotHardwareInterface()
//     : hardware_interface::SystemInterface() {};

hardware_interface::CallbackReturn IRSSoftrobotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  logger_ = rclcpp::get_logger(info.name);
  RCLCPP_DEBUG(get_logger(), "Got %ld joints", info.joints.size());

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(get_logger(), "Error initialising base interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(), "Hardware interface has %ld joints. 1 expected.",
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {

    // Check if all joints are valid
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Declare interfaces
    CommandState item;
    item.name = joint.name;

    command_state_.push_back(item);

    // Create node
    rclcpp::NodeOptions options;
    options.arguments(
        {"--ros-args", "-r", "__node:=topic_based_ros2_control_" + info.name});
    node_ = rclcpp::Node::make_shared("_", options);

    // Create QoS for BEST_EFFORT reliability
    rclcpp::QoS qos_best_effort(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_best_effort.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Create publisher and subscriber for µROS
    std::string publisher_topic = "/softrobot/" + item.name + "/commands";
    std::string subscription_topic = "/softrobot/" + item.name + "/sensor_data";
    RCLCPP_DEBUG(node_->get_logger(), "Publishing to topic: %s",
                 publisher_topic.c_str());
    RCLCPP_DEBUG(node_->get_logger(), "Subscribing to topic: %s",
                 subscription_topic.c_str());
    using namespace std::placeholders;
    uros_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        publisher_topic, 1);
    uros_subscription_ =
        node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            subscription_topic, qos_best_effort,
            std::bind(&IRSSoftrobotHardwareInterface::subscriber_callback, this,
                      _1));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn IRSSoftrobotHardwareInterface::on_configure(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  // Actual configuration
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn IRSSoftrobotHardwareInterface::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn IRSSoftrobotHardwareInterface::on_deactivate(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn IRSSoftrobotHardwareInterface::on_cleanup(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IRSSoftrobotHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Expose state interfaces
  for (auto &joint : command_state_) {

    RCLCPP_DEBUG(get_logger(), "Found joint: %s", joint.name.c_str());

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "kappa", &joint.state.kappa));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, "phi", &joint.state.phi));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint.name, "l", &joint.state.l));
  }
  return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
IRSSoftrobotHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Expose command interfaces
  for (auto &joint : command_state_) {

    RCLCPP_DEBUG(get_logger(), "Found joint: %s", joint.name.c_str());

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "kappa", &joint.command.kappa));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "phi", &joint.command.phi));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, "l", &joint.command.l));
  }
  return command_interfaces;
};

hardware_interface::return_type IRSSoftrobotHardwareInterface::read(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
  // Read motor positions from last_joint_state_
  // and convert them to the continuum robot conf space
  for (auto &joint : command_state_) {
    std::vector<double> state_lengths = last_joint_state_;

    SoftrobotJointState current_state = joint_state_from_length(state_lengths);

    joint.state.kappa = current_state.kappa;
    joint.state.phi = current_state.phi;
    joint.state.l = current_state.l;
  }
  return hardware_interface::return_type::OK;
};

hardware_interface::return_type IRSSoftrobotHardwareInterface::write(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
  // Publish motor positions for each joint
  for (auto &joint : command_state_) {
    SoftrobotJointState command = {joint.command.kappa, joint.command.phi,
                                   joint.command.l};
    std::vector<double> command_lengths = length_from_softjoint_state(command);

    // Write and read from/to hardware
    std_msgs::msg::Float64MultiArray msg;
    msg.data = command_lengths;

    uros_publisher_->publish(msg);
  }

  // Spin node to send the ROS messages
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
};

void IRSSoftrobotHardwareInterface::subscriber_callback(
    std_msgs::msg::Float64MultiArray msg) {
  // Temorarily save the received message in last_joint_state_
  if (msg.data.size() == 3) {
    last_joint_state_ = msg.data;
  } else {
    RCLCPP_WARN(get_logger(), "Length (%ld) does not match 3", msg.data.size());
  }
};

} // namespace irs_softrobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    irs_softrobot_hardware_interface::IRSSoftrobotHardwareInterface,
    hardware_interface::SystemInterface)
