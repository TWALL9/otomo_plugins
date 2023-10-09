#include "otomo_plugins/otomo_diffdrive.hpp"
#include "otomo_msgs/otomo.pb.h"

namespace otomo_plugins
{
namespace controllers
{

bool encodeMessage(KissOutputStream& out_kiss, otomo::TopMsg& msg)
{
  std::string out_string;
  if (!msg.SerializeToString(&out_string))
  {
    return false;
  }

  const char* out_c = out_string.c_str();
  size_t len = out_string.size();

  for (uint8_t i = 0; i < len; i++)
  {
    out_kiss.addByte(out_c[i]);
  }

  return true;
}

OtomoDiffdrive::OtomoDiffdrive()
: logger_(rclcpp::get_logger("OtomoDiffdrive"))
{

}

hwi_return OtomoDiffdrive::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != hwi_return::OK)
  {
    return hwi_return::ERROR;
  }

  time_ = std::chrono::system_clock::now();

  config_.left_wheel_name_ = info_.hardware_parameters["left_wheel_name"];
  config_.right_wheel_name_ = info_.hardware_parameters["right_wheel_name"];
  config_.encoder_count_ = std::stoi(info_.hardware_parameters["encoder_count"]);
  l_wheel_ = Wheel(config_.left_wheel_name_, config_.encoder_count_);
  r_wheel_ = Wheel(config_.right_wheel_name_, config_.encoder_count_);

  config_.serial_name_ = info_.hardware_parameters["serial_port"];
  config_.baud_rate_ = std::stoi(info_.hardware_parameters["serial_baud_rate"]);

  config_.timeout_ms_ = std::stoi(info_.hardware_parameters["timeout"]);
  config_.loop_rate_ = std::stof(info_.hardware_parameters["loop_rate"]);

  // init serial port
  serial_port_ = std::shared_ptr<async_serial::SerialPort>(new async_serial::SerialPort(config_.serial_name_, config_.baud_rate_));

  status_ = hardware_interface::status::CONFIGURED;

  return hwi_return::OK;  
}

std::vector<hardware_interface::StateInterface> OtomoDiffdrive::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.push_back(hardware_interface::StateInterface(l_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel_));
  state_interfaces.push_back(hardware_interface::StateInterface(l_wheel_.name(), hardware_interface::HW_IF_POSITION, &l_wheel_.pos_));
  state_interfaces.push_back(hardware_interface::StateInterface(r_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel_));
  state_interfaces.push_back(hardware_interface::StateInterface(r_wheel_.name(), hardware_interface::HW_IF_POSITION, &r_wheel_.pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OtomoDiffdrive::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.push_back(hardware_interface::CommandInterface(l_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd_));
  command_interfaces.push_back(hardware_interface::CommandInterface(r_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd_));

  return command_interfaces;
}

hwi_return OtomoDiffdrive::start()
{
  RCLCPP_INFO(logger_, "Starting OtomoDiffdrive controller");

  if (!serial_port_->open())
  {
    RCLCPP_ERROR(logger_, "Cannot open serial port!");
    return hwi_return::ERROR;
  }

  serial_port_->addReceiveCallback(std::bind(
    &OtomoDiffdrive::asyncSerialCallback, this, std::placeholders::_1, std::placeholders::_2
  ));

  status_ = hardware_interface::status::STARTED;

  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::stop()
{
  RCLCPP_INFO(logger_, "Stopping OtomoDiffdrive controller");
  status_ = hardware_interface::status::STOPPED;

  serial_port_->close();

  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::read()
{
  // Most reading is done in the asyncSerialCallback
  if (!serial_port_->isOpen())
  {
    return hwi_return::ERROR;
  }
  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::write()
{
  if (!serial_port_->isOpen())
  {
    return hwi_return::ERROR;
  }

  auto l_cmd = l_wheel_.cmd_ / l_wheel_.rads_per_count() / config_.loop_rate_;
  auto r_cmd = r_wheel_.cmd_ / r_wheel_.rads_per_count() / config_.loop_rate_;

  // Create and send command to robot
  otomo::TopMsg msg;
  otomo::FanControl * fan = new otomo::FanControl();
  fan->set_on(true);
  msg.set_allocated_fan(fan);

  KissOutputStream out_kiss;
  if (!encodeMessage(out_kiss, msg))
  {
    RCLCPP_ERROR(logger_, "Cannot serialize fan msg to string");
  }

  auto buf = out_kiss.getBuffer();
  serial_port_->send(buf);

  return hwi_return::OK;
}

void OtomoDiffdrive::asyncSerialCallback(const std::vector<uint8_t>& buf, size_t num_received)
{
  (void)num_received;

  for (const auto& b : buf)
  {
    int ret = recv_buf_.addByte(b);
    if (ret != 0)
    {
      RCLCPP_WARN_STREAM(logger_, "receive buffer error: " << ret);
      recv_buf_.init();
    }
    else if (recv_buf_.isReady())
    {
      uint8_t port;
      std::vector<uint8_t> in_proto(recv_buf_.getBuffer(ret, port));
      recv_buf_.init();

      otomo::TopMsg proto_msg;
      if (!proto_msg.ParseFromArray((const void *)&in_proto[0], in_proto.size()))
      {
        RCLCPP_ERROR(logger_, "Could not deserialize proto msg from mcu!, 0x%x, %d", in_proto.front(), in_proto.size());
      }
      else if (proto_msg.has_state())
      {
        RCLCPP_INFO(logger_, "Got robot state!");
        const auto& state = proto_msg.state();
        l_wheel_.vel_ = state.left_motor().angular_velocity();
        r_wheel_.vel_ = state.right_motor().angular_velocity();
      }
    }
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  OtomoDiffdrive,
  hardware_interface::SystemInterface
)

}
}
