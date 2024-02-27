
#ifndef OTOMO_PLUGINS__OTOMO_CONTROLLER_HPP_
#define OTOMO_PLUGINS__OTOMO_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/base_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "otomo_msgs/msg/pid.hpp"

#include <chrono>
#include <string>
#include <vector>
#include <map>

namespace otomo_plugins::controllers {

using ci_return = controller_interface::return_type;
using interface_return = controller_interface::InterfaceConfiguration;
using cb_return = rlcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OtomoController : public controller_interface::ControllerInterface {

public:
  OtomoController();

  ci_return init(const std::string& controller_name) override;
  interface_return command_interface_configuration() const override;
  interface_return state_interface_configuration() const override;
  ci_return update() override;

  cb_return on_configure(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_activate(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_error(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
  void pid_update_cb(const otomo_msgs::msg::Pid::SharedPtr msg);

  struct PidParams {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    bool update_pid = false;
  };
  std::map<std::string, PidParams> pid_controllers_;

  bool is_halted_ {false};

  std::vector<rclcpp::Subscription<otomo_msgs::msg::Pid>::SharedPtr> pid_subs_;
};

}

#endif // OTOMO_PLUGINS__OTOMO_CONTROLLER_HPP_