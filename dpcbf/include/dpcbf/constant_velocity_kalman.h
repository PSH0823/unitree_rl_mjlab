#pragma once

namespace dpcbf {

// Position-measurement 2-D constant-velocity Kalman filter.
//
// State:       [x, y, vx, vy]
// Measurement: [x, y]
// Process:     constant velocity with white acceleration noise
//
// The caller supplies the measurement-to-measurement dt. This deliberately
// keeps ROS clocks and transport timing out of the reusable estimator.
class ConstantVelocityKalman2D {
 public:
  struct Config {
    double position_measurement_std = 0.01;
    double process_acceleration_std = 2.0;
    double initial_velocity_std = 0.5;
    double innovation_gate_sigma = 5.0;
  };

  void Configure(const Config& config);
  void Reset(double x, double y);

  // Predicts by dt and incorporates the position measurement. Returns false
  // when the 2-D innovation is outside the configured Mahalanobis gate; the
  // predicted state is retained in that case.
  bool Update(double x, double y, double dt);

  bool initialized() const { return initialized_; }
  double x() const { return x_.position; }
  double y() const { return y_.position; }
  double vx() const { return x_.velocity; }
  double vy() const { return y_.velocity; }

 private:
  struct AxisState {
    double position = 0.0;
    double velocity = 0.0;
    double p00 = 0.0;
    double p01 = 0.0;
    double p10 = 0.0;
    double p11 = 0.0;
  };

  void Predict(AxisState* state, double dt) const;
  static void Correct(AxisState* state, double innovation,
                      double innovation_variance);

  Config config_{};
  AxisState x_{};
  AxisState y_{};
  bool configured_ = false;
  bool initialized_ = false;
};

}  // namespace dpcbf
