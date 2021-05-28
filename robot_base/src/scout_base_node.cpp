#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>


#include "robot_base/scout_messenger.hpp"

using namespace westonrobot;

std::shared_ptr<MobileBase> robot;
bool keep_run = true;

void DetachRobot(int signal) {
  if(robot->SdkHasControlToken())
  robot->RenounceControl();
keep_run = false;
  
}

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  std::signal(SIGINT, DetachRobot);

  robot = std::make_shared<MobileBase>();
  // instantiate a robot object
  
  ScoutROSMessenger messenger(robot.get(), &node);
  ROS_INFO("ScoutROSMessenger initialised");

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("vcan1"));

  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                  std::string("odom"));

  ROS_INFO("Parameters fetched");

  if (!messenger.simulated_robot_) {
    ROS_INFO("not a simulated robot");
    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos) {
      ROS_INFO("can detected");
      
      robot->Connect(port_name);
      ROS_INFO("Using CAN bus to talk with the robot");
    } else {
      
      ROS_INFO("Please connect using CAN");
    }
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate(50);
  while (keep_run) {
    
      if (!messenger.simulated_robot_) {
        messenger.PublishStateToROS();
      } else {
        ZVector3 linear = {0};
        ZVector3 angular = {0};
        messenger.GetCurrentMotionCmdForSim(linear.x, angular.z);
        messenger.PublishSimStateToROS(linear.x, angular.z);
      }
      ros::spinOnce();
      rate.sleep();
    
  }

  return 0;
}