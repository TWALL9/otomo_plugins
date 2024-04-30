#include "otomo_plugins/otomo_diffdrive.hpp"
#include "otomo_msgs/otomo.pb.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace otomo_plugins::controllers {

bool encode_message(async_serial::KissOutputStream& out_kiss, otomo::TopMsg& msg) {
  std::string out_string;
  if (!msg.SerializeToString(&out_string)) {
    return false;
  }

  const char* out_c = out_string.c_str();
  size_t len = out_string.size();

  for (uint8_t i = 0; i < len; i++) {
    out_kiss.add_byte(out_c[i]);
  }

  return true;
}

OtomoDiffdrive::OtomoDiffdrive()
: logger_(rclcpp::get_logger("OtomoDiffdrive")) { }

hwi_return OtomoDiffdrive::configure(const hardware_interface::HardwareInfo& info) {
  if (configure_default(info) != hwi_return::OK) {
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

  auto node = rclcpp::Node("dummy");

  left_pub_ = node.create_publisher<otomo_msgs::msg::Diffdrive>("/left_motor", 10);
  right_pub_ = node.create_publisher<otomo_msgs::msg::Diffdrive>("/right_motor", 10);

  return hwi_return::OK;
}

std::vector<hardware_interface::StateInterface> OtomoDiffdrive::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.push_back(hardware_interface::StateInterface(l_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel_));
  state_interfaces.push_back(hardware_interface::StateInterface(l_wheel_.name(), hardware_interface::HW_IF_POSITION, &l_wheel_.pos_));
  state_interfaces.push_back(hardware_interface::StateInterface(r_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel_));
  state_interfaces.push_back(hardware_interface::StateInterface(r_wheel_.name(), hardware_interface::HW_IF_POSITION, &r_wheel_.pos_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OtomoDiffdrive::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.push_back(hardware_interface::CommandInterface(l_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd_));
  command_interfaces.push_back(hardware_interface::CommandInterface(r_wheel_.name(), hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd_));

  command_interfaces.push_back(hardware_interface::CommandInterface("default", HW_IF_PROPORTIONAL, &pid_.p));
  command_interfaces.push_back(hardware_interface::CommandInterface("default", HW_IF_INTEGRAL, &pid_.i));
  command_interfaces.push_back(hardware_interface::CommandInterface("default", HW_IF_DERIVATIVE, &pid_.d));

  return command_interfaces;
}

hwi_return OtomoDiffdrive::start() {
  RCLCPP_INFO(logger_, "Starting OtomoDiffdrive controller");

  if (!serial_port_->open()) {
    RCLCPP_ERROR(logger_, "Cannot open serial port!");
    return hwi_return::ERROR;
  }

  serial_port_->add_receive_callback(std::bind(
    &OtomoDiffdrive::async_serial_callback, this, std::placeholders::_1, std::placeholders::_2
  ));

  status_ = hardware_interface::status::STARTED;

  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::stop() {
  RCLCPP_INFO(logger_, "Stopping OtomoDiffdrive controller");
  status_ = hardware_interface::status::STOPPED;

  serial_port_->close();

  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::read() {
  // Most reading is done in the async_serial_callback
  if (!serial_port_->is_open()) {
    return hwi_return::ERROR;
  }
  return hwi_return::OK;
}

hwi_return OtomoDiffdrive::write() {
  if (!serial_port_->is_open()) {
    return hwi_return::ERROR;
  }

  auto l_cmd = l_wheel_.cmd_;  // rad/s
  auto r_cmd = r_wheel_.cmd_;

  // RCLCPP_INFO_STREAM(logger_, "cmd: " << l_wheel_.cmd_ << ", " << r_wheel_.cmd_);

  // Create and send command to robot
  otomo::TopMsg msg;
  otomo::DiffDrive * diff_drive = new otomo::DiffDrive();
  diff_drive->set_left_motor(l_cmd);
  diff_drive->set_right_motor(r_cmd);
  msg.set_allocated_diff_drive(diff_drive);

  async_serial::KissOutputStream out_kiss;
  if (encode_message(out_kiss, msg)) {
    auto buf = out_kiss.get_buffer();
    serial_port_->send(buf);
  } else {
    RCLCPP_ERROR(logger_, "Cannot serialize diffdrive msg to string");
  }

  // Update PID params
  if (old_pid_.p != pid_.p || old_pid_.i != pid_.i || old_pid_.d != pid_.d) {
    old_pid_.p = pid_.p;
    old_pid_.i = pid_.i;
    old_pid_.d = pid_.d;

    RCLCPP_WARN(logger_, "Updating PID: %f, %f, %f", pid_.p, pid_.i, pid_.d);

    otomo::TopMsg pid_msg;
    otomo::Pid * pid = new otomo::Pid();
    pid->set_p(pid_.p);
    pid->set_i(pid_.i);
    pid->set_d(pid_.d);

    pid_msg.set_allocated_pid(pid);
    async_serial::KissOutputStream out_pid;
    if (encode_message(out_pid, pid_msg)) {
      auto buf_pid = out_pid.get_buffer();
      serial_port_->send(buf_pid);
    } else {
      RCLCPP_ERROR(logger_, "Cannot serialize pid msg to string");
    }
  }

  return hwi_return::OK;
}

void OtomoDiffdrive::async_serial_callback(const std::vector<uint8_t>& buf, size_t num_received) {
  for (size_t i = 0; i < num_received; i++) {
    int ret = recv_buf_.add_byte(buf[i]);
    if (ret != 0) {
      if (ret != -1) {
        // RCLCPP_WARN_STREAM(logger_, "receive buffer error: " << ret);
      }
      recv_buf_.init();
    } else if (recv_buf_.is_ready()) {
      uint8_t port;
      std::vector<uint8_t> in_proto = recv_buf_.get_buffer(ret, port);

      otomo::TopMsg proto_msg;
      if (!proto_msg.ParseFromArray((const void *)&in_proto[0], in_proto.size())) {
        RCLCPP_ERROR(logger_, "Could not deserialize proto msg from mcu!, 0x%x, %d", in_proto.front(), in_proto.size());
      } else if (proto_msg.has_state()) {
        const auto& state = proto_msg.state();
        l_wheel_.vel_ = state.left_motor().angular_velocity();
        l_wheel_.pos_ = state.left_motor().encoder();
        r_wheel_.vel_ = state.right_motor().angular_velocity();
        r_wheel_.pos_ = state.right_motor().encoder();

        auto left_msg = otomo_msgs::msg::Diffdrive();
        // left_msg.header.stamp = node_->get_clock()->now();
        left_msg.left = l_wheel_.cmd_;
        left_msg.right = l_wheel_.vel_;
        left_pub_->publish(left_msg);

        auto right_msg = otomo_msgs::msg::Diffdrive();
        // right_msg.header.stamp = left_msg.header.stamp;
        right_msg.right = r_wheel_.cmd_;
        right_msg.right = r_wheel_.vel_;
        right_pub_->publish(right_msg);

        const auto now = std::chrono::system_clock::now();
        double stamp = static_cast<double>(now.time_since_epoch().count()) / 1000000000;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(5);
        ss << stamp << ", " << l_wheel_.cmd_ << ", " << l_wheel_.vel_ << ", " << r_wheel_.cmd_ << ", " << r_wheel_.vel_;
        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
      } else if (proto_msg.has_drive_response()) {
        RCLCPP_INFO_STREAM(logger_, "Got robot response");
      }
      recv_buf_.init();
    }
  }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  otomo_plugins::controllers::OtomoDiffdrive,
  hardware_interface::SystemInterface
)
