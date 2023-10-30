#include <memory>
#include "youbot_driver_wrapper/YouBotDriverWrapper.hpp"

using std::placeholders::_1;
using namespace youbot;

YouBotDriverWrapper::YouBotDriverWrapper() : Node("youbot_driver_wrapper_node")
{
  // setup the subscriber
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&YouBotDriverWrapper::twist_callback, this, _1));

  // setup the publisher
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&YouBotDriverWrapper::odom_publisher_callback, this));

  // create handles for youBot base and manipulator (if available)
  try {
    myYouBotBase = new YouBotBase("youbot-base", youbot_configuration_dir);
    myYouBotBase->doJointCommutation();
    youBotHasBase = true;
  } catch (std::exception& e) {
    LOG(warning) << e.what();
    youBotHasBase = false;
  }
}

void YouBotDriverWrapper::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  quantity<si::velocity> longitudinalVelocity = (twist_msg->linear.x) * meter_per_second;
  quantity<si::velocity> transversalVelocity = (twist_msg->linear.y) * meter_per_second;
  quantity<si::angular_velocity> angularVelocity = (twist_msg->angular.z) * radian_per_second;
  myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}

void YouBotDriverWrapper::odom_publisher_callback()
{
  quantity<si::length> longitudinalPosition;
  quantity<si::length> transversalPosition;
  quantity<plane_angle> orientation;
  myYouBotBase->getBasePosition(longitudinalPosition,transversalPosition,orientation);
  odom_msg_.header.frame_id = "odom";
  odom_msg_.header.stamp = now(); 
  odom_msg_.child_frame_id = "youbot_base";

  // pose
  odom_msg_.pose.pose.position.x = longitudinalPosition.value();
  odom_msg_.pose.pose.position.y = transversalPosition.value();
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.w = cos(orientation.value()*0.5);
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = sin(orientation.value()*0.5);

  // twist
  quantity<si::velocity> longitudinal_velocity;
  quantity<si::velocity> transversal_velocity;
  quantity<si::angular_velocity> angular_velocity;
  myYouBotBase->getBaseVelocity(longitudinal_velocity,transversal_velocity,angular_velocity);
  odom_msg_.twist.twist.linear.x = longitudinal_velocity.value(); 
  odom_msg_.twist.twist.linear.y = transversal_velocity.value();
  odom_msg_.twist.twist.linear.z = 0.0; 
  odom_msg_.twist.twist.angular.x = 0;
  odom_msg_.twist.twist.angular.y = 0;
  odom_msg_.twist.twist.angular.z = angular_velocity.value();

  odom_publisher_->publish(odom_msg_); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YouBotDriverWrapper>());
  rclcpp::shutdown();
  return 0;
}