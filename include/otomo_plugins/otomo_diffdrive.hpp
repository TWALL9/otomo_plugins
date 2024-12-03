#pragma once

#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "async_serial/serial_port.hpp"
#include "async_serial/kiss_tnc.hpp"

#include "otomo_plugins/config.hpp"
#include "otomo_plugins/wheel.hpp"
#include "otomo_plugins/pid_params.hpp"

#include "otomo_msgs/msg/diffdrive.hpp"

namespace otomo_plugins::controllers {

class OtomoDiffdrive : public hardware_interface::SystemInterface {
public:
  using cb_return = hardware_interface::CallbackReturn;
  using hwi_return = hardware_interface::return_type;

  RCLCPP_SHARED_PTR_DEFINITIONS(OtomoDiffdrive)

  OtomoDiffdrive() = default;

  cb_return on_init(const hardware_interface::HardwareInfo& info) override;
  cb_return on_activate(const rclcpp_lifecycle::State& previous_state) override;
  cb_return on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hwi_return read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hwi_return write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void async_serial_callback(const std::vector<uint8_t>& buf, size_t num_received);

  DiffdriveConfiguration config_;
  std::shared_ptr<async_serial::SerialPort> serial_port_;
  Wheel l_wheel_;
  Wheel r_wheel_;

  PidParams pid_;
  PidParams old_pid_;

  std::chrono::time_point<std::chrono::system_clock> time_;
  async_serial::KissInputStream recv_buf_;
};

}
