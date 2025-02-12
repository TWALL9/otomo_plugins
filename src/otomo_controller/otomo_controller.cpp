#include <memory>

#include "otomo_plugins/otomo_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace otomo_plugins::controllers {

using std::placeholders::_1;

std::vector<std::string> find_by_split(std::string s, std::string delim = " ") {
  std::vector<std::string> v;

  int start = 0;
  int end = -1 * delim.size();
  do {
    start = end + delim.size();
    end = s.find(delim, start);
    v.push_back(s.substr(start, end - start));
  } while (end != -1);

  return v;
}

OtomoController::OtomoController() : controller_interface::ControllerInterface() {}

cb_return OtomoController::on_init() {
  try {
    // default to nothing,
    auto_declare<double>("pid_default_p_term", 0.0);
    auto_declare<double>("pid_default_i_term", 0.0);
    auto_declare<double>("pid_default_d_term", 0.0);

    auto_declare<std::vector<std::string>>("pid_controllers", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Exception during otomo_controller init stage: %s",
      e.what()
    );
    return cb_return::ERROR;
  }

  return cb_return::SUCCESS;
}

interface_return OtomoController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf = { controller_interface::interface_configuration_type::INDIVIDUAL, {} };

  if (pid_controllers_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "command interface empty controller map");
  }

  conf.names.reserve(pid_controllers_.size() * 3);

  for (const auto& pid_name : pid_controllers_) {
    conf.names.push_back(pid_name.first + "/" + HW_IF_PROPORTIONAL);
    conf.names.push_back(pid_name.first + "/" + HW_IF_INTEGRAL);
    conf.names.push_back(pid_name.first + "/" + HW_IF_DERIVATIVE);
  }

  return conf;
}

interface_return OtomoController::state_interface_configuration() const {
  std::vector<std::string> state_interfaces;
  return { controller_interface::interface_configuration_type::NONE, state_interfaces };
}

cb_return OtomoController::on_configure(const rclcpp_lifecycle::State&) {
  auto logger = get_node()->get_logger();

  std::vector<std::string> controller_names;
  controller_names = get_node()->get_parameter("pid_controllers").as_string_array();
  double default_p = get_node()->get_parameter("pid_default_p_term").as_double();
  double default_i = get_node()->get_parameter("pid_default_i_term").as_double();
  double default_d = get_node()->get_parameter("pid_default_d_term").as_double();

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
  pid_subs_ = get_node()->create_subscription<otomo_msgs::msg::Pid>(
    "/otomo_controller/set_pid/default",
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<otomo_msgs::msg::Pid> pid) -> void {
      RCLCPP_WARN(get_node()->get_logger(), "Received PID update for default");
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
  auto logger = get_node()->get_logger();

  registered_pid_handles_.clear();

  if (pid_controllers_.empty()) {
    RCLCPP_ERROR(logger, "no controllers are present");
    return cb_return::ERROR;
  } else {
    for (const auto& ctrl : pid_controllers_) {
      const auto controller_name = ctrl.first;
      const auto command_handle_p = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name, &logger](const auto& interface) {
          auto interface_name = find_by_split(interface.get_name(), "/").front();
          return interface_name == controller_name &&
            interface.get_interface_name() == HW_IF_PROPORTIONAL;
        }
      );

      if (command_handle_p == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle P for %s", controller_name.c_str());
        return cb_return::ERROR;
      }

      const auto command_handle_i = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name](const auto& interface) {
          auto interface_name = find_by_split(interface.get_name(), "/").front();
          return interface_name == controller_name &&
            interface.get_interface_name() == HW_IF_INTEGRAL;
        }
      );

      if (command_handle_i == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle I for %s", controller_name.c_str());
        return cb_return::ERROR;
      }

      const auto command_handle_d = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&controller_name](const auto& interface) {
          auto interface_name = find_by_split(interface.get_name(), "/").front();
          return interface_name == controller_name &&
            interface.get_interface_name() == HW_IF_DERIVATIVE;
        }
      );

      if (command_handle_d == command_interfaces_.end()) {
        RCLCPP_ERROR(logger, "Unable to obtain command handle D for %s", controller_name.c_str());
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

ci_return OtomoController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  auto logger = get_node()->get_logger();

  bool inactive = get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

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
      (void)handle.p.get().set_value(pid.p);
      (void)handle.i.get().set_value(pid.i);
      (void)handle.d.get().set_value(pid.d);
    }
  }

  return ci_return::OK;
}



} // namespace otomo_plugins::controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  otomo_plugins::controllers::OtomoController, controller_interface::ControllerInterface)
