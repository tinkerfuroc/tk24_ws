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

Chassis::Chassis(){
    rclcpp::Node node_priv;

    // Setup Variables
    // Setup Motors
    for (int i = 0; i < 4; i++)
    {
      motors[i] = new Motor(MOTOR_CHASSIS_ID_START + i, &MOTOR_CHASSIS, MOTOR_CHASSIS_PARAMTER);
    }

    // Setup Reconfigurable Paramters

}