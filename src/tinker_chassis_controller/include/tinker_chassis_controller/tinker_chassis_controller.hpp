#ifndef __TINKER_CHASSIS_CONTROLLER_H__
#define __TINKER_CHASSIS_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include "realtime_tools/realtime_box.h"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include <string>
#include <chrono>
#include <queue>
#include <vector>
#include "tinker_chassis_controller/chassis_motor.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace tinker_chassis_controller
{
    using Twist = geometry_msgs::msg::TwistStamped;

    class TinkerChassisController
        : public controller_interface::ControllerInterface
    {
    public:
         
        TinkerChassisController();
        
         
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

         
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

         
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

         
        controller_interface::CallbackReturn on_init() override;

         
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

         
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

         
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

         
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

         
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

         
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    
    protected:
        std::shared_ptr<ChassisMotor> get_wheel(const std::string & wheel_joint_name);

        bool reset();

    protected:
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
            velocity_command_unstamped_subscriber_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> motor_state_publisher_ = nullptr;
        realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> velocity_command_ptr_;
        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        std::shared_ptr<ChassisMotor> fl_wheel_;
        std::shared_ptr<ChassisMotor> fr_wheel_;
        std::shared_ptr<ChassisMotor> rl_wheel_;
        std::shared_ptr<ChassisMotor> rr_wheel_;
        std::string fl_wheel_joint_name_;
        std::string fr_wheel_joint_name_;
        std::string rl_wheel_joint_name_;
        std::string rr_wheel_joint_name_;
        double linear_scale_;
        double radial_scale_;
        double wheel_radius_;
        double wheel_distance_width_;
        double wheel_distance_length_;
        double wheel_separation_width_;
        double wheel_separation_length_;
        bool subscriber_is_active_;
        bool use_stamped_vel_ = false;
        std::chrono::milliseconds cmd_vel_timeout_{500};
        
        std::queue<Twist> previous_commands_;  // last two commands
        std::vector<double> debug_data = {0,0,0,0};
        std_msgs::msg::Float64MultiArray debug_message;

        // odom
        Odometry odometry_;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
            realtime_odometry_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
            realtime_odometry_transform_publisher_ = nullptr;
        size_t velocity_rolling_window_size_;
        // time stamp
        rclcpp::Time previous_update_timestamp_{0};
        // publish rate limiter
        double publish_rate_ = 50.0;
        rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
        rclcpp::Time previous_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};

    };
}


#endif // __TINKER_CHASSIS_CONTROLLER_H__