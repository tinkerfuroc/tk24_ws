#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class LivoxMsgDecoder : public rclcpp::Node
{
public:
   LivoxMsgDecoder(std::string name);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr command_subscribe_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr command_publish_;
    void subscribe_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timer_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    sensor_msgs::msg::PointCloud::SharedPtr converted_PointCloud;
    rclcpp::TimerBase::SharedPtr timer_;
}

