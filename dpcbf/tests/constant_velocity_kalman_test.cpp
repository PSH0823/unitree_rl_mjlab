#include "dpcbf/constant_velocity_kalman.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

void ExpectNear(double actual, double expected, double tolerance,
                const char* label) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(label) + " mismatch");
  }
}

}  // namespace

int main() {
  dpcbf::ConstantVelocityKalman2D filter;
  filter.Configure({0.01, 2.0, 0.5, 5.0});
  filter.Reset(0.0, 0.0);

  constexpr double dt = 0.01;
  constexpr double vx = 0.8;
  constexpr double vy = -0.3;
  for (int i = 1; i <= 500; ++i) {
    const double noise_x = (i % 2 == 0 ? 1.0 : -1.0) * 0.003;
    const double noise_y = (i % 3 == 0 ? 1.0 : -1.0) * 0.002;
    if (!filter.Update(vx * i * dt + noise_x,
                       vy * i * dt + noise_y, dt)) {
      throw std::runtime_error("nominal Kalman measurement was rejected");
    }
  }
  ExpectNear(filter.vx(), vx, 0.04, "vx convergence");
  ExpectNear(filter.vy(), vy, 0.04, "vy convergence");

  const double vx_before = filter.vx();
  const double vy_before = filter.vy();
  if (filter.Update(100.0, -100.0, dt)) {
    throw std::runtime_error("Kalman outlier gate accepted a gross jump");
  }
  ExpectNear(filter.vx(), vx_before, 0.02, "vx outlier prediction");
  ExpectNear(filter.vy(), vy_before, 0.02, "vy outlier prediction");
  return 0;
}
