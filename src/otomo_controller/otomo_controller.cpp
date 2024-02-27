#include "otomo_plugins/otomo_controller.hpp"
#include <memory>

namespace otomo_plugins::controllers {

using std::placeholders::_1;

OtomoController::OtomoController()
  : controller_interface::ControllerInterface() {}

ci_return OtomoController::init(const std::string& controller_name) {
  auto ret = ControllerInterface::init(controller_name);
  if (ret != ci_return::OK) {
    return ret;
  }

  try {
    // default to nothing, 
    auto_declare<double>("pid_default_p_term", 0.0);
    auto_declare<double>("pid_default_i_term", 0.0);
    auto_declare<double>("pid_default_d_term", 0.0);

    auto_declare<std::vector<std::string>>("pid_controllers", std::vector<std::string>>);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception during %s init stage: %s", controller_name.c_str(), e.what());
    return ci_return::ERROR;
  }

  return ci_return::OK;
}

interface_return OtomoController::command_interface_configuration() const {
  std::vector<std::string> command_interfaces;

  for (const auto& pid_name : pid_controllers_) {
    command_interfaces.push_back(pid_name.first + "/pid/p");
    command_interfaces.push_back(pid_name.first + "/pid/i");
    command_interfaces.push_back(pid_name.first + "/pid/d");
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces};
}

interface_return OtomoController::state_interface_configuration() const {
  std::vector state_interfaces;
  return {controller_interface::interface_configuration_type::NONE, state_interfaces};
}

cb_return OtomoController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  auto logger = node_->get_logger();

  std::vector<std::string> controller_names;
  controller_names = node_->get_parameter("pid_controllers").as_string_array();
  double default_p = node_->get_parameter("pid_default_p_term").as_double();
  double default_i = node_->get_parameter("pid_default_i_term").as_double();
  double default_d = node_->get_parameter("pid_default_d_term").as_double();

  for (const auto& name : controller_names) {
    if (pid_controllers_.find(name) == pid_controllers_.end()) {
      PidParams param {
        .p = default_p,
        .i = default_i,
        .d = default_d,
        .update_pid = false,
      };
      auto fn std::bind(&OtomoController::pid_update_cb, this, _1);
      pid_controllers_.insert(name, param);

    }
  }

  return cb_return::OK;

}

ci_return OtomoController::update() {
  auto logger = node_->get_logger();

  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted_) {
      is_halted_ = true;
      halt();
    }

    return ci_return::OK;
  }

  if (update_pid_) {
    update_pid_ = false;
    RCLCPP_INFO_STREAM(logger, "Updating PID params: p: " << pid_params_.p, << ", i: " << pid_params_.i << ", d" << pid_params_.d);
  }

  return ci_return::OK;
}



}