#include "tinker_chassis_controller/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "tinker_chassis_controller/odometry.hpp"
#include "tinker_chassis_controller/tinker_chassis_controller.hpp"

namespace tinker_chassis_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheel_base_(0.0),
  wheel_radius_(0.0),
  left_wheel_old_pos_(0.0),
  right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double left_front_vel, double left_rear_vel, double right_front_vel, double right_rear_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  // right_rear_vel = - right_rear_vel;
  // left_rear_vel = -left_rear_vel;
  //Compute linear and angular diff:
  const double linear_x = wheel_radius_ * (left_front_vel + right_front_vel - left_rear_vel -  right_rear_vel) * 0.25;
  const double linear_y = wheel_radius_ * (-left_front_vel + right_front_vel + left_rear_vel - right_rear_vel) * 0.25;
  const double angular = (1 / (2 * (wheel_base_ + wheel_separation_))) * (-left_front_vel - right_front_vel - left_rear_vel - right_rear_vel) * wheel_radius_;
  RCLCPP_INFO(rclcpp::get_logger("Odometry"), "linear x : %lf", linear_x);
  RCLCPP_INFO(rclcpp::get_logger("Odometry_wheel_radius"), "wheel_radius : %lf", wheel_radius_);
  RCLCPP_INFO(rclcpp::get_logger("Odometry_lf"), "lf_vel : %lf", left_front_vel);
  RCLCPP_INFO(rclcpp::get_logger("Odometry_rf"), "rf_vel : %lf", right_front_vel);
  RCLCPP_INFO(rclcpp::get_logger("Odometry_lr"), "lr_vel : %lf", left_rear_vel);
  RCLCPP_INFO(rclcpp::get_logger("Odometry_rr"), "rr_vel : %lf", right_rear_vel);


  //Integrate odometry:
  integrateExact(linear_x, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear_x / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double wheel_base, double wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheel_base_ = wheel_base;
  wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace tinker_chassis_controller