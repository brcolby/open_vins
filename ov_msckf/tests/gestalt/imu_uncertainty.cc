// Numerical regression of the actual native propagation and interpolation.
#include "state/Propagator.h"
#include "state/State.h"
#include <iostream>
#include <stdexcept>
extern "C" void gestalt_vio_visual_update() {}
extern "C" unsigned long long gestalt_vio_visual_count() { return 0; }
struct Probe : ov_msckf::Propagator {
  using ov_msckf::Propagator::Propagator;
  using ov_msckf::Propagator::predict_and_compute;
};
int main() try {
  ov_msckf::StateOptions options;
  auto first = std::make_shared<ov_msckf::State>(options);
  auto second = std::make_shared<ov_msckf::State>(options);
  Probe p(ov_msckf::NoiseManager{}, 9.80665);
  ov_core::ImuData left, right;
  left.timestamp = 1; right.timestamp = 1.027;
  left.wm = right.wm = Eigen::Vector3d(0.1, -0.3, 0.2);
  left.am = right.am = Eigen::Vector3d(0.2, 0.4, 9.8);
  Eigen::MatrixXd f1, f2, q1, q2;
  p.predict_and_compute(first, left, right, f1, q1);
  right.noise_variance_scale = 9;
  p.predict_and_compute(second, left, right, f2, q2);
  if (!f1.isApprox(f2, 1e-12) || !q2.isApprox(9*q1, 1e-12) ||
      !first->_imu->value().isApprox(second->_imu->value(), 1e-12))
    throw std::runtime_error("gap noise changed means/Jacobians or failed to scale covariance");
  const auto cut = Probe::interpolate_data(left, right, 1.01);
  if (cut.noise_variance_scale != 9 || cut.timestamp != 1.01)
    throw std::runtime_error("interpolation lost acquisition uncertainty");
  right.noise_variance_scale = 0;
  bool rejected = false;
  try { Probe::interpolate_data(left, right, 1.01); }
  catch (const std::invalid_argument &) { rejected = true; }
  if (!rejected) throw std::runtime_error("invalid uncertainty accepted");
  std::cout << "IMU means, Jacobians, interval covariance and interpolation passed\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n'; return 1;
}
