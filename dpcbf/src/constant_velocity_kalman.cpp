#include "dpcbf/constant_velocity_kalman.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dpcbf {

void ConstantVelocityKalman2D::Configure(const Config& config) {
  if (!std::isfinite(config.position_measurement_std) ||
      !std::isfinite(config.process_acceleration_std) ||
      !std::isfinite(config.initial_velocity_std) ||
      !std::isfinite(config.innovation_gate_sigma) ||
      config.position_measurement_std <= 0.0 ||
      config.process_acceleration_std <= 0.0 ||
      config.initial_velocity_std <= 0.0 ||
      config.innovation_gate_sigma <= 0.0) {
    throw std::invalid_argument("invalid constant-velocity Kalman config");
  }
  config_ = config;
  configured_ = true;
  initialized_ = false;
}

void ConstantVelocityKalman2D::Reset(double x, double y) {
  if (!configured_) {
    throw std::logic_error("Kalman filter must be configured before Reset");
  }
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("non-finite Kalman reset position");
  }
  const double position_variance =
      config_.position_measurement_std * config_.position_measurement_std;
  const double velocity_variance =
      config_.initial_velocity_std * config_.initial_velocity_std;
  x_ = {x, 0.0, position_variance, 0.0, 0.0, velocity_variance};
  y_ = {y, 0.0, position_variance, 0.0, 0.0, velocity_variance};
  initialized_ = true;
}

void ConstantVelocityKalman2D::Predict(AxisState* state, double dt) const {
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  const double acceleration_variance =
      config_.process_acceleration_std * config_.process_acceleration_std;

  state->position += dt * state->velocity;
  const double p00 = state->p00 + dt * (state->p01 + state->p10) +
                     dt2 * state->p11 +
                     0.25 * acceleration_variance * dt4;
  const double p01 = state->p01 + dt * state->p11 +
                     0.5 * acceleration_variance * dt3;
  const double p10 = state->p10 + dt * state->p11 +
                     0.5 * acceleration_variance * dt3;
  const double p11 = state->p11 + acceleration_variance * dt2;
  state->p00 = p00;
  state->p01 = p01;
  state->p10 = p10;
  state->p11 = p11;
}

void ConstantVelocityKalman2D::Correct(
    AxisState* state, double innovation, double innovation_variance) {
  const double old_p00 = state->p00;
  const double old_p01 = state->p01;
  const double old_p10 = state->p10;
  const double old_p11 = state->p11;
  const double k_position = old_p00 / innovation_variance;
  const double k_velocity = old_p10 / innovation_variance;

  state->position += k_position * innovation;
  state->velocity += k_velocity * innovation;
  state->p00 = (1.0 - k_position) * old_p00;
  state->p01 = (1.0 - k_position) * old_p01;
  state->p10 = old_p10 - k_velocity * old_p00;
  state->p11 = old_p11 - k_velocity * old_p01;

  // Keep round-off from slowly breaking covariance symmetry/positivity.
  const double cross = 0.5 * (state->p01 + state->p10);
  state->p01 = cross;
  state->p10 = cross;
  state->p00 = std::max(state->p00, 1.0e-15);
  state->p11 = std::max(state->p11, 1.0e-15);
}

bool ConstantVelocityKalman2D::Update(double x, double y, double dt) {
  if (!initialized_) {
    throw std::logic_error("Kalman filter must be reset before Update");
  }
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(dt) ||
      dt <= 0.0) {
    throw std::invalid_argument("invalid Kalman update input");
  }

  Predict(&x_, dt);
  Predict(&y_, dt);
  const double measurement_variance =
      config_.position_measurement_std * config_.position_measurement_std;
  const double innovation_x = x - x_.position;
  const double innovation_y = y - y_.position;
  const double variance_x = x_.p00 + measurement_variance;
  const double variance_y = y_.p00 + measurement_variance;
  const double mahalanobis_squared =
      innovation_x * innovation_x / variance_x +
      innovation_y * innovation_y / variance_y;
  const double gate_squared =
      config_.innovation_gate_sigma * config_.innovation_gate_sigma;
  if (mahalanobis_squared > gate_squared) return false;

  Correct(&x_, innovation_x, variance_x);
  Correct(&y_, innovation_y, variance_y);
  return true;
}

}  // namespace dpcbf
