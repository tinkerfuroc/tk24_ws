#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/state.hpp"

#include "tinker_chassis_controller/tinker_chassis_controller.hpp"

using namespace tinker_chassis_controller;
using lifecycle_msgs::msg::State;
namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel_unstamped";
}  // namespace
TinkerChassisController::TinkerChassisController()
    : controller_interface::ControllerInterface()
    , velocity_command_subscriber_(nullptr)
    , velocity_command_ptr_(nullptr)
{

}

controller_interface::InterfaceConfiguration TinkerChassisController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    RCLCPP_INFO(get_node()->get_logger(), "Configure TinkerChassisController");

    command_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TinkerChassisController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

    return state_interfaces_config;
}

controller_interface::CallbackReturn TinkerChassisController::on_init()
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TinkerChassisController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    auto logger = get_node()->get_logger();
    if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
        return controller_interface::return_type::OK;
    }
    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);
    if (last_command_msg == nullptr)
    {
      RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
      return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
      last_command_msg->twist.linear.x = 0.0;
      last_command_msg->twist.angular.z = 0.0;
    }
    Twist command = *last_command_msg;
    // Calculate the wheel velocity
    // See: http://robotsforroboticists.com/drive-kinematics/
    const auto twist = command.twist;

    RCLCPP_INFO(logger, "velocity message received:x: %lf, y:%lf", twist.linear.x, twist.linear.y);
    double fl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double fr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rr_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);

    fl_wheel_->set_velocity(fl_wheel_velocity);
    fr_wheel_->set_velocity(fr_wheel_velocity);
    rl_wheel_->set_velocity(rl_wheel_velocity);
    rr_wheel_->set_velocity(rr_wheel_velocity);

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn TinkerChassisController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configure TinkerChassisController");

    fl_wheel_joint_name_ = get_node()->get_parameter("fl_wheel_joint_name").as_string();
    fr_wheel_joint_name_ = get_node()->get_parameter("fr_wheel_joint_name").as_string();
    rl_wheel_joint_name_ = get_node()->get_parameter("rl_wheel_joint_name").as_string();
    rr_wheel_joint_name_ = get_node()->get_parameter("rr_wheel_joint_name").as_string();
    // Change message type of stamped or not here
    // **** //
    if (fl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fl_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (fr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'fr_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (rl_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rl_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (rr_wheel_joint_name_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'rr_wheel_joint_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "'fl_wheel_joint_name' parameter : %s", fl_wheel_joint_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "'fr_wheel_joint_name' parameter : %s", fr_wheel_joint_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "'rl_wheel_joint_name' parameter : %s", rl_wheel_joint_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "'rr_wheel_joint_name' parameter : %s", rr_wheel_joint_name_.c_str());
    wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
    wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
    wheel_distance_length_ = get_node()->get_parameter("wheel_distance.length").as_double();
    if (wheel_radius_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_radius' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_width_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.width' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (wheel_distance_length_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'wheel_distance.length' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }
    wheel_separation_width_ = wheel_distance_width_ / 2;
    wheel_separation_length_ = wheel_distance_length_ / 2;

    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    // initialize command subscriber
    if (use_stamped_vel_)
    {
      velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }
          if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
          {
            RCLCPP_WARN_ONCE(
              get_node()->get_logger(),
              "Received TwistStamped with zero timestamp, setting it to current "
              "time, this message will only be shown once");
            msg->header.stamp = get_node()->get_clock()->now();
          }
          received_velocity_msg_ptr_.set(std::move(msg));
        });
    }
    else
    {
      velocity_command_unstamped_subscriber_ =
        get_node()->create_subscription<geometry_msgs::msg::Twist>(
          DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
          {
            if (!subscriber_is_active_)
            {
              RCLCPP_WARN(
                get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
              return;
            }

            // Write fake header in the stored stamped command
            std::shared_ptr<Twist> twist_stamped;
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = get_node()->get_clock()->now();
          });
    }
    

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TinkerChassisController::on_activate(const rclcpp_lifecycle::State &)
{
    // Initialize the wheels
    fl_wheel_ = get_wheel(fl_wheel_joint_name_);
    fr_wheel_ = get_wheel(fr_wheel_joint_name_);
    rl_wheel_ = get_wheel(rl_wheel_joint_name_);
    rr_wheel_ = get_wheel(rr_wheel_joint_name_);
    if (!fl_wheel_ || !fr_wheel_ || !rl_wheel_ || !rr_wheel_) {
        return controller_interface::CallbackReturn::ERROR;
    }
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TinkerChassisController::on_deactivate(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TinkerChassisController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TinkerChassisController::on_error(const rclcpp_lifecycle::State &)
{
    if (!reset()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TinkerChassisController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

std::shared_ptr<ChassisMotor> TinkerChassisController::get_wheel(const std::string & wheel_joint_name)
{
    // Lookup the position state interface
    const auto position_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), 
        [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
        {
            return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
        }
    );
    if (position_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity state interface
    const auto velocity_state = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
    {
        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_state == state_interfaces_.cend()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Lookup the velocity command interface
    const auto velocity_command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
    {
        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
    });
    if (velocity_command == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
    }

    // Create the wheel instance
    return std::make_shared<ChassisMotor>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command)
        );
}

bool TinkerChassisController::reset()
{
    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();

    fl_wheel_.reset();
    fr_wheel_.reset();
    rl_wheel_.reset();
    rr_wheel_.reset();

    return true;
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    tinker_chassis_controller::TinkerChassisController,
    controller_interface::ControllerInterface
)