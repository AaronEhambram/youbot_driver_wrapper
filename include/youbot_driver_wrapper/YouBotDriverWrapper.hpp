#include "rclcpp/rclcpp.hpp"
#include "youbot/YouBotBase.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace youbot;

class YouBotDriverWrapper : public rclcpp::Node
{
  public:
    YouBotDriverWrapper();
  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);

    // velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

    // odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void odom_publisher_callback();
    nav_msgs::msg::Odometry odom_msg_ ; 

    // YouBot specific
    std::string youbot_configuration_dir = "/home/aaron/workspace/libraries/youbot_driver/config";
    YouBotBase* myYouBotBase = 0;
    bool youBotHasBase = false;
};