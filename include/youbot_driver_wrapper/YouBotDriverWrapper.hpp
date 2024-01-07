#include "rclcpp/rclcpp.hpp"
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

using std::placeholders::_1;
using namespace youbot;

class YouBotDriverWrapper : public rclcpp::Node
{
  public:
    YouBotDriverWrapper();
  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void arm_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr arm_state_msg); 

    // velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

    // arm joints subsciber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_subscription_;

    // odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void odom_publisher_callback();
    nav_msgs::msg::Odometry odom_msg_; 

    // tf2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;

    // joint publisher
    std::thread joint_publisher_thread_; 
    sensor_msgs::msg::JointState joint_state_msg_; 
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    void publish_joint_state(); 

    // YouBot specific
    std::string youbot_configuration_dir = "/home/aaron/workspace/libraries/youbot_driver/config";
    YouBotBase* myYouBotBase = 0;
    bool youBotHasBase = false;
    YouBotManipulator* myYouBotManipulator = 0; 
    bool youBotHasArm = false; 
};