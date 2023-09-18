#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "tinker_chassis_controller/odometry.hpp"
#include "tinker_chassis_controller/tinker_chassis_controller.hpp"

using namespace tinker_chassis_controller;
using lifecycle_msgs::msg::State;
namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_DEBUG_TOPIC = "/motor_state";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
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
      // last_command_msg->twist.linear.x = 0.0;
      // last_command_msg->twist.angular.z = 0.0;
    }
    Twist command = *last_command_msg;
    // Calculate the wheel velocity
    // See: http://robotsforroboticists.com/drive-kinematics/
    const auto twist = command.twist;

    double fl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x - twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double fr_wheel_velocity = -(1 / wheel_radius_) * (twist.linear.x + twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rl_wheel_velocity = (1 / wheel_radius_) * (twist.linear.x + twist.linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    double rr_wheel_velocity = -(1 / wheel_radius_) * (twist.linear.x - twist.linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist.angular.z);
    // RCLCPP_INFO(logger, "fl_wheel: %lf",fl_wheel_velocity);
    // RCLCPP_INFO(logger, "fl_wheel: %lf",fr_wheel_velocity);
    // RCLCPP_INFO(logger, "fl_wheel: %lf",rl_wheel_velocity);
    // RCLCPP_INFO(logger, "fl_wheel: %lf",rr_wheel_velocity);
    // RCLCPP_INFO(logger, "command: %lf",twist.linear.x);
    debug_data[0] = fl_wheel_velocity;
    debug_data[1] = fr_wheel_velocity;
    debug_data[2] = rl_wheel_velocity;
    debug_data[3] = rr_wheel_velocity;
    debug_message.data = debug_data;
    motor_state_publisher_->publish(debug_message);

    fl_wheel_->set_velocity(fl_wheel_velocity);
    fr_wheel_->set_velocity(fr_wheel_velocity);
    rl_wheel_->set_velocity(rl_wheel_velocity);
    rr_wheel_->set_velocity(rr_wheel_velocity);


    // odom
    if (get_node()->get_parameter("open_loop").as_bool()){
      odometry_.updateOpenLoop(twist.linear.x,  twist.angular.z, time);
    }
    else { // Use velocity 
      // RCLCPP_INFO(logger, "fl_wheel: %lf",fl_wheel_->get_velocity());
      // RCLCPP_INFO(logger, "fl_wheel: %lf",fr_wheel_->get_velocity());
      // RCLCPP_INFO(logger, "fl_wheel: %lf",rl_wheel_->get_velocity());
      // RCLCPP_INFO(logger, "fl_wheel: %lf",rr_wheel_->get_velocity());
      odometry_.update(fl_wheel_->get_velocity(), fr_wheel_->get_velocity(), rl_wheel_->get_velocity(), rr_wheel_->get_velocity(), time);
    }

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    bool should_publish = false;
    try
    {
      if (previous_publish_timestamp_ + publish_period_ < time)
      {
        previous_publish_timestamp_ += publish_period_;
        should_publish = true;
      }
    }
    catch (const std::runtime_error &)
    {
      // Handle exceptions when the time source changes and initialize publish timestamp
      previous_publish_timestamp_ = time;
      should_publish = true;
    }

    if (should_publish)
    {
      if (realtime_odometry_publisher_->trylock())
      {
        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.stamp = time;
        odometry_message.pose.pose.position.x = odometry_.getX();
        odometry_message.pose.pose.position.y = odometry_.getY();
        odometry_message.pose.pose.orientation.x = orientation.x();
        odometry_message.pose.pose.orientation.y = orientation.y();
        odometry_message.pose.pose.orientation.z = orientation.z();
        odometry_message.pose.pose.orientation.w = orientation.w();
        odometry_message.twist.twist.linear.x = odometry_.getLinear();
        odometry_message.twist.twist.angular.z = odometry_.getAngular();
        realtime_odometry_publisher_->unlockAndPublish();
      }

      if (get_node()->get_parameter("enable_odom_tf").as_bool() && realtime_odometry_transform_publisher_->trylock())
      {
        auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odometry_.getX();
        transform.transform.translation.y = odometry_.getY();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        realtime_odometry_transform_publisher_->unlockAndPublish();
      }
    }


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
    velocity_rolling_window_size_ = get_node()->get_parameter("velocity_rolling_window_size").as_int();
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
    if (velocity_rolling_window_size_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "'velocity_rolling_window_size_' parameter cannot be zero or less");
        return controller_interface::CallbackReturn::ERROR;
    }

    odometry_.setWheelParams(wheel_distance_width_, wheel_distance_length_, wheel_radius_);
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size_);
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
    
    motor_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(DEFAULT_DEBUG_TOPIC, rclcpp::SystemDefaultsQoS());

    // initialize odometry publisher and messasge
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =  std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);
    std::string controller_namespace = std::string(get_node()->get_namespace());

    if (controller_namespace == "/")
    {
      controller_namespace = "";
    }
    else
    {
      controller_namespace = controller_namespace + "/";
    }

    const auto odom_frame_id = controller_namespace + get_node()->get_parameter("odom_frame_id").as_string();
    const auto base_frame_id = controller_namespace + get_node()->get_parameter("base_frame_id").as_string();

    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = controller_namespace + odom_frame_id;
    odometry_message.child_frame_id = controller_namespace + base_frame_id;

    // limit the publication on the topics /odom and /tf
    publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

    // initialize odom values zeros
    odometry_message.twist =
      geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    std::vector<double> pose_covariance_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
    std::vector<double> twist_covariance_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
    for (size_t index = 0; index < 6; ++index)
    {
      // 0, 7, 14, 21, 28, 35
      const size_t diagonal_index = NUM_DIMENSIONS * index + index;
      odometry_message.pose.covariance[diagonal_index] = pose_covariance_diagonal[index];
      odometry_message.twist.covariance[diagonal_index] = twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);

    // keeping track of odom and base_link transforms only
    auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

    previous_update_timestamp_ = get_node()->get_clock()->now();

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
    subscriber_is_active_ = true;
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