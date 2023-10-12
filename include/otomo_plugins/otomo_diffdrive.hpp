#pragma once

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "async_serial/serial_port.hpp"
#include "async_serial/kiss_tnc.hpp"

#include "otomo_plugins/config.hpp"
#include "otomo_plugins/wheel.hpp"

#include <vector>
#include <chrono>

namespace otomo_plugins
{
namespace controllers
{

using hwi_return = hardware_interface::return_type;

class OtomoDiffdrive : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{

public:
  OtomoDiffdrive();

  hwi_return configure(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hwi_return start() override;
  hwi_return stop() override;
  hwi_return read() override;
  hwi_return write() override;

private:
  void asyncSerialCallback(const std::vector<uint8_t>& buf, size_t num_received);

  DiffdriveConfiguration config_;
  std::shared_ptr<async_serial::SerialPort> serial_port_;
  Wheel l_wheel_;
  Wheel r_wheel_;

  rclcpp::Logger logger_;
  std::chrono::time_point<std::chrono::system_clock> time_;
  async_serial::KissInputStream recv_buf_;
  
};

}
}