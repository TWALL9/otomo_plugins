#pragma once

#include <string>
#include<cmath>

namespace otomo_plugins::controllers {

class Wheel {
public:
  Wheel() = default;

  Wheel(const std::string& name, int counts_per_rev)
  : name_(name), rads_per_count_(2 * M_PI / counts_per_rev) { }

  double calc_encoder_angle() {
    return enc_ * rads_per_count_;
  }

  const std::string& name() const {
    return name_;
  }

  double rads_per_count() const {
    return rads_per_count_;
  }

  double vel_ { 0.0 };
  double pos_ { 0.0 };
  double cmd_ { 0.0 };
  double effort_ { 0.0 };

private:
  std::string name_;
  double rads_per_count_;
  int enc_ { 0 };

};

}
