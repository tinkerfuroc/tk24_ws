#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "config.hpp"
#include "hardware.hpp"
#include "motor.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <chrono>
#include <functional>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float64_multi_array.hpp"


class Chassis : public rclcpp::Node{
public:
    Chassis();
    ~Chassis();

    /*Update Funcs*/
    void update();
    void UpdateOdometry();
    void UpdateWatchdog();
    void UpdateDebug();

    /* Publish Funcs */
    void PublishPosition();

    /* Callback Funcs */
    void CallbackVelocity(const geometry_msgs::Twist::ConstPtr& twist);

private:
    /* Config */
    bool Config_IsDebug;

    /* Handles */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<geometry_msgs::msg::twist>::SharedPtr twist_sub;

    /* Motor */
    Motor* motors[4];
    rclcpp::Time motorWatchdog;

    /* Odometry */
    double x, y, theta;
    double last_position[4];
    double vx_local, vy_local, vtheta_local;
    rclcpp::Time odom_last_time;
    uint32_t odom_seq;

    
};


#endif