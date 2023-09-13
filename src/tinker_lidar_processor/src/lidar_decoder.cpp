#include "tinker_lidar_processor/lidar_decoder.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
LivoxMsgDecoder::LivoxMsgDecoder(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "%s node has started", name.c_str());
    command_subscribe_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("livox/lidar", 10, std::bind(&LivoxMsgDecoder::subscribe_callback, this, std::placeholders::_1));
    command_publish_ = this->create_publisher<sensor_msgs::msg::PointCloud>("converted_lidar_pointcloud", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LivoxMsgDecoder::timer_callback, this));
}

void LivoxMsgDecoder::subscribe_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, converted_PointCloud);
}

void LivoxMsgDecoder::timer_callback()
{
    command_publish_->publish(converted_PointCloud);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<LivoxMsgDecoder>("Hello");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}