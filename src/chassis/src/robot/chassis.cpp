#include "chassis.hpp"

double YawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
{
  tf2::Quaternion q_orient(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m_orient(q_orient);

  double roll, pitch, yaw;
  m_orient.getRPY(roll, pitch, yaw);

  return yaw;
}

double YawFromQuaternion(const tf2::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(quat);
  m.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

Chassis::Chassis() : Node("chassis_node"){

    // Setup Variables
    // Setup Motors
    for (int i = 0; i < 4; i++)
    {
      motors[i] = new Motor(MOTOR_CHASSIS_ID_START + i, &MOTOR_CHASSIS, MOTOR_CHASSIS_PARAMTER);
    }

i   // Setup Parameters
    node_priv.set

    // Setup Reconfigurable Paramters

    // Setup communication
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    twist_sub = this->create_subscription<geometry::msg::Twist>(
            "cmd_vel", 10, std::bind(&Chassis::CallbackVelocity, this, _1));
    

}

void Chassis::CallbackVelocity(const geometry_msgs::msg::Twist &twist)
{
  // reset watchdog
  motorWatchdog = rclcpp::Time::now();

  // set motor power
  double vx = twist->linear.x;
  double vy = twist->linear.y;
  double vw = twist->angular.z;

  double a = CHASSIS_LENGTH_A + CHASSIS_LENGTH_B;

  // swap w[0] and w[1]
  double w[4] = { ((-a * vw + vx - vy) / CHASSIS_WHEEL_R), -((a * vw + vx + vy) / CHASSIS_WHEEL_R),
                  (-a * vw + vx + vy) / CHASSIS_WHEEL_R, -((a * vw + vx - vy) / CHASSIS_WHEEL_R) };

  // Velocity Limitation
  double maxVel = 0.0;
  for (int i = 0; i < 4; i++)
    maxVel = std::max(maxVel, std::abs(w[i]));

  if (maxVel > Dyn_Config_MaxVel)
  {
    double factor = Dyn_Config_MaxVel / maxVel;
    for (int i = 0; i < 4; i++)
      w[i] *= factor;
  }

  // Send Velocity
  for (int i = 0; i < 4; i++)
    motors[i]->Setpoint = w[i];
}