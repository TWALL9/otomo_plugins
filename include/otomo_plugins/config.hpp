#pragma once

#include <string>

namespace otomo_plugins
{
namespace controllers
{

struct DiffdriveConfiguration
{
  std::string left_wheel_name_ { "left_wheel" };
  std::string right_wheel_name_ { "right_wheel" };

  std::string serial_name_ { "/dev/ttyUSB0" };
  int baud_rate_ { 115200 };

  int encoder_count_ { 1000 }; // ??? find in motor datasheet
  
  int timeout_ms_ { 1000 };
  float loop_rate_ { 30.0 };
};

}

}