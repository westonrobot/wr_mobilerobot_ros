/* 
 * scout_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MESSENGER_HPP
#define SCOUT_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "scout_msgs/LightMode.h"
#include "scout_msgs/LightControlType.h"
#include "scout_msgs/SysCtrlState.h"
#include "scout_msgs/SysOperState.h"
#include "scout_msgs/SysErrorCode.h"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"

namespace westonrobot
{
class ScoutROSMessenger
{
public:
    explicit ScoutROSMessenger(ros::NodeHandle *nh);
    ScoutROSMessenger(MobileBase *scout, ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();
    bool GetControlToken();

    void PublishStateToROS();
    void PublishSimStateToROS(float linear, float angular);

    void GetCurrentMotionCmdForSim(float &linear, float &angular);

private:
    MobileBase *scout_;
    ros::NodeHandle *nh_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Publisher system_state_publisher_;
    ros::Publisher light_state_publisher_;
    ros::Publisher motion_state_publisher_;
    ros::Publisher actuator_state_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber light_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // speed variables
    float linear_speed_ = 0.0;
    float angular_speed_ = 0.0;
    float position_x_ = 0.0;
    float position_y_ = 0.0;
    float theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void LightCmdCallback(const scout_msgs::LightControlType::ConstPtr &msg);
    void PublishOdometryToROS(float linear, float angular, float dt);
};
} // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */
