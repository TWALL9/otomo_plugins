#include "otomo_msgs/otomo.pb.h"
#include "otomo_msgs/msg/diffdrive.hpp"

#include "async_serial/kiss_tnc.hpp"
#include "async_serial/serial_port.hpp"

#include "rclcpp/rclcpp.hpp"

namespace otomo_plugins::diffdrive_test_node {

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

class DiffdriveTestNode : public rclcpp::Node {
public:
  DiffdriveTestNode() : Node("diff_drive_test_node"), logger_(rclcpp::get_logger("DiffdriveTestNode")) {
    pub_ = this->create_publisher<otomo_msgs::msg::Diffdrive>("robot_diff", 10);
    repub_ = this->create_publisher<otomo_msgs::msg::Diffdrive>("cmd_repub", 10);
    sub_ = this->create_subscription<otomo_msgs::msg::Diffdrive>("cmd_robot_diff",
      10, std::bind(&DiffdriveTestNode::cmd_callback, this, std::placeholders::_1));

    serial_port_ = std::shared_ptr<async_serial::SerialPort>(new async_serial::SerialPort("/dev/ttyOtomo", 115200));

    if (!serial_port_->open()) {
      RCLCPP_ERROR(logger_, "Cannot open serial port!");
    }

    serial_port_->add_receive_callback(std::bind(
      &DiffdriveTestNode::serial_callback, this, std::placeholders::_1, std::placeholders::_2
    ));
  }

private:

  void cmd_callback(const otomo_msgs::msg::Diffdrive::SharedPtr msg) {
    otomo::TopMsg out_msg;
    otomo::DiffDrive * diff_drive = new otomo::DiffDrive();
    diff_drive->set_left_motor(msg->left);
    diff_drive->set_right_motor(msg->right);
    out_msg.set_allocated_diff_drive(diff_drive);

    async_serial::KissOutputStream out_kiss;
    if (!encode_message(out_kiss, out_msg)) {
      RCLCPP_ERROR(logger_, "Cannot serialize fan msg to string");
    }

    auto buf = out_kiss.get_buffer();
    serial_port_->send(buf);

    auto dd_msg = otomo_msgs::msg::Diffdrive();
    dd_msg.header.stamp = this->get_clock()->now();
    dd_msg.left = msg->left;
    dd_msg.right = msg->right;
    repub_->publish(dd_msg);
  }

  void serial_callback(const std::vector<uint8_t>& buf, size_t num_received) {
    for (size_t i = 0; i < num_received; i++) {
      int ret = recv_buf_.add_byte(buf[i]);
      if (ret != 0)
      {
        recv_buf_.init();
      } else if (recv_buf_.is_ready()) {
        uint8_t port;
        std::vector<uint8_t> in_proto = recv_buf_.get_buffer(ret, port);

        otomo::TopMsg proto_msg;
        if (!proto_msg.ParseFromArray((const void *)&in_proto[0], in_proto.size())) {
          RCLCPP_WARN(logger_, "Could not deserialize proto msg from mcu!, 0x%x, %ld", in_proto.front(), in_proto.size());
        } else if (proto_msg.has_state()) {
          const auto& state = proto_msg.state();
          auto dd_msg = otomo_msgs::msg::Diffdrive();
          dd_msg.header.stamp = this->get_clock()->now();
          dd_msg.left = state.left_motor().angular_velocity();
          dd_msg.right = state.right_motor().angular_velocity();
          RCLCPP_INFO_STREAM(logger_, "Got robot state! " << dd_msg.left << ", " << dd_msg.right);

          pub_->publish(dd_msg);
        }
        recv_buf_.init();
      }
    }
  }

  rclcpp::Logger logger_;
  async_serial::KissInputStream recv_buf_;
  std::shared_ptr<async_serial::SerialPort> serial_port_;

  rclcpp::Subscription<otomo_msgs::msg::Diffdrive>::SharedPtr sub_;
  rclcpp::Publisher<otomo_msgs::msg::Diffdrive>::SharedPtr pub_;
  rclcpp::Publisher<otomo_msgs::msg::Diffdrive>::SharedPtr repub_;

};

}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<otomo_plugins::diffdrive_test_node::DiffdriveTestNode>());
  rclcpp::shutdown();

  return 0;
}