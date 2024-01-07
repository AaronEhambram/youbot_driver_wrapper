#include <memory>
#include "youbot_driver_wrapper/YouBotDriverWrapper.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using namespace youbot;

YouBotDriverWrapper::YouBotDriverWrapper() : Node("youbot_driver_wrapper_node")
{
  // setup the subscriber
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&YouBotDriverWrapper::twist_callback, this, _1));
  arm_joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("arm_joint_state", 1, std::bind(&YouBotDriverWrapper::arm_joint_state_callback, this, _1));

  // setup the publisher
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&YouBotDriverWrapper::odom_publisher_callback, this));

  // setup tf broadcaster
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // create handles for youBot base and manipulator (if available)
  try {
    myYouBotBase = new YouBotBase("youbot-base", youbot_configuration_dir);
    myYouBotBase->doJointCommutation();
    youBotHasBase = true;
  } catch (std::exception& e) {
    LOG(warning) << e.what();
    youBotHasBase = false;
  }

  try {
		myYouBotManipulator = new YouBotManipulator("youbot-manipulator", youbot_configuration_dir);
		myYouBotManipulator->doJointCommutation();
		myYouBotManipulator->calibrateManipulator();
		myYouBotManipulator->calibrateGripper();
		youBotHasArm = true;
	} catch (std::exception& e) {
		LOG(warning) << e.what();
		youBotHasArm = false;
	}

  // start the joint state publishing in a seperate thread
  joint_state_msg_.name.resize(11);
  joint_state_msg_.position.resize(11); 
  // base wheels
  joint_state_msg_.name[0] = "wheel_joint_fl"; joint_state_msg_.name[1] = "wheel_joint_fr"; joint_state_msg_.name[2] = "wheel_joint_bl";
  joint_state_msg_.name[3] = "wheel_joint_br";
  // arm
  joint_state_msg_.name[4] = "arm_joint_1"; joint_state_msg_.name[5] = "arm_joint_2"; joint_state_msg_.name[6] = "arm_joint_3";
  joint_state_msg_.name[7] = "arm_joint_4"; joint_state_msg_.name[8] = "arm_joint_5"; 
  // gripper
  joint_state_msg_.name[9] = "gripper_finger_joint_l"; joint_state_msg_.name[10] = "gripper_finger_joint_r"; 
  // create teh message object
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  // start the seperate thread
  std::thread publisher_thread(&YouBotDriverWrapper::publish_joint_state, this);
  joint_publisher_thread_ = std::move(publisher_thread); 
}

void YouBotDriverWrapper::publish_joint_state()
{
  rclcpp::Rate loop_rate(15);
  while(rclcpp::ok())
  {
    std::vector<JointSensedAngle> base_data;
    myYouBotBase->getJointData(base_data);
    std::vector<JointSensedAngle> arm_data;
    myYouBotManipulator->getJointData(arm_data);

    joint_state_msg_.header.stamp = now(); 
    joint_state_msg_.header.frame_id = "";

    // base wheels
    joint_state_msg_.position[0] = -base_data[0].angle.value();
    joint_state_msg_.position[1] = base_data[1].angle.value();
    joint_state_msg_.position[2] = -base_data[2].angle.value();
    joint_state_msg_.position[3] = base_data[3].angle.value();
    
    // arm joints
    for(int i = 0; i < arm_data.size(); ++i)
    {
      joint_state_msg_.position[i+4] = arm_data[i].angle.value();
    }

    /*// gripper data this code leads to crash, as the gripper value cannot be accessed
    GripperSensedBarPosition bar_left;
    myYouBotManipulator->getArmGripper().getGripperBar1().getData(bar_left);
    joint_state_msg_.position[9] = bar_left.barPosition.value();
    GripperSensedBarPosition bar_right;
    myYouBotManipulator->getArmGripper().getGripperBar2().getData(bar_right);
    joint_state_msg_.position[10] = bar_right.barPosition.value();*/

    joint_state_publisher_->publish(joint_state_msg_);
    loop_rate.sleep();
  }
}

void YouBotDriverWrapper::twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  quantity<si::velocity> longitudinalVelocity = (twist_msg->linear.x) * meter_per_second;
  quantity<si::velocity> transversalVelocity = (twist_msg->linear.y) * meter_per_second;
  quantity<si::angular_velocity> angularVelocity = (twist_msg->angular.z) * radian_per_second;
  myYouBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}

void YouBotDriverWrapper::arm_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr arm_state_msg)
{
  JointAngleSetpoint desiredJointAngle;
  desiredJointAngle.angle = arm_state_msg->position[0] * radian;
  myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

  desiredJointAngle.angle = arm_state_msg->position[1] * radian;
  myYouBotManipulator->getArmJoint(2).setData(desiredJointAngle);

  desiredJointAngle.angle = arm_state_msg->position[2] * radian;
  myYouBotManipulator->getArmJoint(3).setData(desiredJointAngle);

  desiredJointAngle.angle = arm_state_msg->position[3] * radian;
  myYouBotManipulator->getArmJoint(4).setData(desiredJointAngle);

  desiredJointAngle.angle = arm_state_msg->position[4] * radian;
  myYouBotManipulator->getArmJoint(5).setData(desiredJointAngle);

  GripperBarPositionSetPoint desired_gripper_bar; 
  desired_gripper_bar.barPosition = arm_state_msg->position[5] * meter;
  myYouBotManipulator->getArmGripper().getGripperBar1().setData(desired_gripper_bar);

  desired_gripper_bar.barPosition = arm_state_msg->position[6] * meter;
  myYouBotManipulator->getArmGripper().getGripperBar2().setData(desired_gripper_bar);
}

void YouBotDriverWrapper::odom_publisher_callback()
{
  quantity<si::length> longitudinalPosition;
  quantity<si::length> transversalPosition;
  quantity<plane_angle> orientation;
  myYouBotBase->getBasePosition(longitudinalPosition,transversalPosition,orientation);
  odom_msg_.header.frame_id = "odom";
  odom_msg_.header.stamp = now(); 
  odom_msg_.child_frame_id = "base_footprint";

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

  // create broadcaster for tf-tree
  geometry_msgs::msg::TransformStamped tfs_;
  tfs_.header = odom_msg_.header;
  tfs_.header.stamp = odom_msg_.header.stamp;
  tfs_.header.frame_id = odom_msg_.header.frame_id;
  tfs_.child_frame_id = odom_msg_.child_frame_id;
  tfs_.transform.translation.x = odom_msg_.pose.pose.position.x;
  tfs_.transform.translation.y = odom_msg_.pose.pose.position.y;
  tfs_.transform.translation.z = odom_msg_.pose.pose.position.z;
  tfs_.transform.rotation = odom_msg_.pose.pose.orientation;
  tfb_->sendTransform(tfs_);

  odom_publisher_->publish(odom_msg_); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YouBotDriverWrapper>());
  rclcpp::shutdown();
  return 0;
}