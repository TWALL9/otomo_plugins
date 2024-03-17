#include "otomo_plugins/otomo_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"

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

    auto_declare<std::vector<std::string>>("pid_controllers", {});

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception during %s init stage: %s", controller_name.c_str(), e.what());
    return ci_return::ERROR;
  }

  return ci_return::OK;
}

interface_return OtomoController::command_interface_configuration() const {
  std::vector<std::string> command_interfaces;

  if (pid_controllers_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "command interface empty controller map");
  }

  for (const auto& pid_name : pid_controllers_) {
    command_interfaces.push_back(pid_name.first + "/" + HW_IF_PROPORTIONAL);
    command_interfaces.push_back(pid_name.first + "/" + HW_IF_INTEGRAL);
    command_interfaces.push_back(pid_name.first + "/" + HW_IF_DERIVATIVE);
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces };
}

interface_return OtomoController::state_interface_configuration() const {
  std::vector<std::string> state_interfaces;
  return { controller_interface::interface_configuration_type::NONE, state_interfaces };
}

cb_return OtomoController::on_configure(const rclcpp_lifecycle::State&) {
  auto logger = node_->get_logger();

  std::vector<std::string> controller_names;
  controller_names = node_->get_parameter("pid_controllers").as_string_array();
  double default_p = node_->get_parameter("pid_default_p_term").as_double();
  double default_i = node_->get_parameter("pid_default_i_term").as_double();
  double default_d = node_->get_parameter("pid_default_d_term").as_double();

  // PidParams default_param { default_p, default_i, default_d, false };
  // pid_controllers_.insert({ "default", default_param });

  if (controller_names.empty()) {
    RCLCPP_WARN(logger, "No PID controllers!");
    return cb_return::ERROR;
  }

  for (const auto& name : controller_names) {
    RCLCPP_ERROR(logger, "finding params for %s", name.c_str());
    if (pid_controllers_.find(name) == pid_controllers_.end()) {
      PidParams param { default_p, default_i, default_d, false };
      // auto fn std::bind(&OtomoController::pid_update_cb, this, _1);
      pid_controllers_.insert({ name, param });
    }
  }

  // todo create vec of subs for individual pid controllers
  pid_subs_ = node_->create_subscription<otomo_msgs::msg::Pid>(
    "/otomo_controller/set_pid/default",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<otomo_msgs::msg::Pid> pid) -> void {
      RCLCPP_WARN(node_->get_logger(), "Received PID update for default");
      PidParams new_pid;
      new_pid.p = pid->p;
      new_pid.i = pid->i;
      new_pid.d = pid->d;
      new_pid.update_pid = true;
      pid_controllers_.at("default") = new_pid;
    });

  return cb_return::SUCCESS;
}

cb_return OtomoController::on_activate(const rclcpp_lifecycle::State&) {
  auto logger = node_->get_logger();

  if (pid_controllers_.empty()) {
    RCLCPP_ERROR(logger, "no controllers are present");
    return cb_return::ERROR;
  } else {
    for (const auto& ctrl : pid_controllers_) {
      const auto controller_name = ctrl.first;
      const auto command_handle_p = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name](const auto& interface) {
          return interface.get_name() == controller_name &&
            interface.get_interface_name() == HW_IF_PROPORTIONAL;
        }
      );

      if (command_handle_p == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle P for %s", controller_name);
        return cb_return::ERROR;
      }

      const auto command_handle_i = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name](const auto& interface) {
          return interface.get_name() == controller_name &&
            interface.get_interface_name() == HW_IF_INTEGRAL;
        }
      );

      if (command_handle_i == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle I for %s", controller_name);
        return cb_return::ERROR;
      }

      const auto command_handle_d = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name](const auto& interface) {
          return interface.get_name() == controller_name &&
            interface.get_interface_name() == HW_IF_DERIVATIVE;
        }
      );

      if (command_handle_d == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle D for %s", controller_name);
        return cb_return::ERROR;
      }

      registered_pid_handles_.insert({ controller_name,
        PidHandle{
          std::ref(*command_handle_p), std::ref(*command_handle_i), std::ref(*command_handle_d)
        }});
    }

    RCLCPP_DEBUG(logger, "Starting PID controllers");
    is_halted_ = false;
    return cb_return::SUCCESS;
  }
}

cb_return OtomoController::on_deactivate(const rclcpp_lifecycle::State&) {
  pid_controllers_.clear();
  is_halted_ = true;

  return cb_return::SUCCESS;
}

cb_return OtomoController::on_error(const rclcpp_lifecycle::State& previous_state) {
  return on_deactivate(previous_state);
}

cb_return OtomoController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
  return on_deactivate(previous_state);
}

cb_return OtomoController::on_shutdown(const rclcpp_lifecycle::State&) {
  return cb_return::SUCCESS;
}

ci_return OtomoController::update() {
  auto logger = node_->get_logger();

  bool inactive = get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

  if (inactive) {
    if (!is_halted_) {
      is_halted_ = true;
    }
    return ci_return::OK;
  }

  for (auto &ctrl : pid_controllers_) {
    auto name = ctrl.first;
    auto& pid = ctrl.second;
    if (pid.update_pid) {
      pid.update_pid = false;
      RCLCPP_INFO_STREAM(logger, "Updating " << name << " PID params: p: " << pid.p << ", i: " << pid.i << ", d: " << pid.d);
      auto handle = registered_pid_handles_.at(name);
      handle.p.get().set_value(pid.p);
      handle.i.get().set_value(pid.i);
      handle.d.get().set_value(pid.d);
    }
  }

  return ci_return::OK;
}



} // namespace otomo_plugins::controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  otomo_plugins::controllers::OtomoController, controller_interface::ControllerInterface)
