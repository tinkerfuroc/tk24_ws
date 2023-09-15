/*
    Author: Weibingchuan
    2023.9.5
*/
#ifndef TINKER_CONTROLLER__ODOMETRY_HPP__
#define TINKER_CONTROLLER__ODOMETRY_HPP__

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcpputils/rcppmath/rolling_mean_accumulator.hpp"

namespace tinker_chassis_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double left_front_vel, double left_rear_vel, double right_front_vel, double right_rear_vel, const rclcpp::Time & time);
  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double wheel_base, double wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_; //width of the base
  double wheel_base_; //length of the base
  double wheel_radius_;

  // Previous wheel position/state [rad]:
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace tinker_chassis_controller

#endif  // TINKER_CONTROLLER__ODOMETRY_HPP__