/*
 * scout_messenger.cpp
 *
 * Created on: Apr 26, 2019 22:14
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "robot_base/scout_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "robot_msgs/RobotState.h"

namespace westonrobot {
ScoutROSMessenger::ScoutROSMessenger(ros::NodeHandle *nh)
    : scout_(nullptr), nh_(nh) {}

ScoutROSMessenger::ScoutROSMessenger(MobileBase *scout, ros::NodeHandle *nh)
    : scout_(scout), nh_(nh) {}

void ScoutROSMessenger::SetupSubscription() {
  // odometry publisher
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_name_, 50);
  system_state_publisher_ = nh_->advertise<robot_msgs::SystemStateMsg>(
      "/mobilebase_system_status", 10);

  motion_state_publisher_ = nh_->advertise<robot_msgs::MotionStateMsg>(
      "/mobilebase_motion_status", 10);

  light_state_publisher_ = nh_->advertise<robot_msgs::LightControlType>(
      "/mobilebase_light_status", 10);

  actuator_state_publisher_ = nh_->advertise<robot_msgs::ActuatorStateMsg>(
      "/mobilebase_actuator_status", 10);

  // cmd subscriber
  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &ScoutROSMessenger::TwistCmdCallback, this);
  light_cmd_subscriber_ = nh_->subscribe<robot_msgs::LightControlType>(
      "/scout_light_control", 5, &ScoutROSMessenger::LightCmdCallback, this);
}

void ScoutROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  ZVector3 linear = {0};
  ZVector3 angular = {0};
  linear.x = msg->linear.x;
  angular.z = msg->angular.z;

  if (!simulated_robot_) {
    if (GetControlToken()) {
      scout_->SetMotionCommand(linear, angular);
    }
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
  // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void ScoutROSMessenger::GetCurrentMotionCmdForSim(float &linear,
                                                  float &angular) {
  std::lock_guard<std::mutex> guard(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

void ScoutROSMessenger::LightCmdCallback(
    const robot_msgs::LightControlType::ConstPtr &msg) {
  if (!simulated_robot_) {
    if (msg->enable_cmd_light_control) {
      LightCommandMsg cmd;

      switch (msg->light_mode) {
        case robot_msgs::LightMode::LIGHT_MODE_CONST_OFF: {
          cmd.command.light_mode = LIGHT_MODE::LIGHT_MODE_CONST_OFF;
          break;
        }
        case robot_msgs::LightMode::LIGHT_MODE_CONST_ON: {
          cmd.command.light_mode = LIGHT_MODE::LIGHT_MODE_CONST_ON;
          break;
        }
        case robot_msgs::LightMode::LIGHT_MODE_BREATH: {
          cmd.command.light_mode = LIGHT_MODE::LIGHT_MODE_BREATH;
          break;
        }
        case robot_msgs::LightMode::LIGHT_MODE_CUSTOM: {
          cmd.command.light_mode = LIGHT_MODE::LIGHT_MODE_CUSTOM;
          cmd.command.light_custom = msg->light_custom;
          break;
        }
      }

      scout_->SetLightCommand(cmd);
    }
  } else {
    std::cout << "simulated robot received light control cmd" << std::endl;
  }
}

void ScoutROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  float dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto motion_state = scout_->GetMotionState();
  auto actuator_state = scout_->GetActuatorState();
  auto light_state = scout_->GetLightState();
  auto system_state = scout_->GetSystemState();

  // publish scout state message

  robot_msgs::SystemStateMsg system_state_msg;
  robot_msgs::MotionStateMsg motion_state_msg;
  robot_msgs::LightControlType light_state_msg;
  robot_msgs::ActuatorStateMsg actuator_state_msg;

  system_state_msg.header.stamp = current_time_;
  motion_state_msg.header.stamp = current_time_;
  light_state_msg.header.stamp = current_time_;
  actuator_state_msg.header.stamp = current_time_;
  // motion state
  // status_msg.motion_state.desired_motion.linear_x =
  // motion_state.desired_motion.linear.x;
  // status_msg.motion_state.desired_motion.linear_y =
  // motion_state.desired_motion.linear.y;
  // status_msg.motion_state.desired_motion.linear_z =
  // motion_state.desired_motion.linear.z;
  // status_msg.motion_state.desired_motion.angular_x =
  // motion_state.desired_motion.angular.x;
  // status_msg.motion_state.desired_motion.angular_y =
  // motion_state.desired_motion.angular.y;
  // status_msg.motion_state.desired_motion.angular_z =
  // motion_state.desired_motion.angular.z;

  // status_msg.motion_state.actual_motion.linear_x =
  // motion_state.actual_motion.linear.x;
  // status_msg.motion_state.actual_motion.linear_y =
  // motion_state.actual_motion.linear.y;
  // status_msg.motion_state.actual_motion.linear_z =
  // motion_state.actual_motion.linear.z;
  // status_msg.motion_state.actual_motion.angular_x =
  // motion_state.actual_motion.angular.x;
  // status_msg.motion_state.actual_motion.angular_y =
  // motion_state.actual_motion.angular.y;
  // status_msg.motion_state.actual_motion.angular_z =
  // motion_state.actual_motion.angular.z;

  motion_state_msg.desired_motion.linear_x =
      motion_state.desired_motion.linear.x;
  motion_state_msg.desired_motion.linear_y =
      motion_state.desired_motion.linear.y;
  motion_state_msg.desired_motion.linear_z =
      motion_state.desired_motion.linear.z;
  motion_state_msg.desired_motion.angular_x =
      motion_state.desired_motion.angular.x;
  motion_state_msg.desired_motion.angular_y =
      motion_state.desired_motion.angular.y;
  motion_state_msg.desired_motion.angular_z =
      motion_state.desired_motion.angular.z;

  motion_state_msg.actual_motion.linear_x = motion_state.actual_motion.linear.x;
  motion_state_msg.actual_motion.linear_y = motion_state.actual_motion.linear.y;
  motion_state_msg.actual_motion.linear_z = motion_state.actual_motion.linear.z;
  motion_state_msg.actual_motion.angular_x =
      motion_state.actual_motion.angular.x;
  motion_state_msg.actual_motion.angular_y =
      motion_state.actual_motion.angular.y;
  motion_state_msg.actual_motion.angular_z =
      motion_state.actual_motion.angular.z;

  // status_msg.motion_state.collision_detected =
  // motion_state.collision_detected;
  // status_msg.motion_state.odometry.left_wheel =
  // motion_state.odometry.left_wheel;
  // status_msg.motion_state.odometry.right_wheel =
  // motion_state.odometry.right_wheel;

  motion_state_msg.collision_detected = motion_state.collision_detected;
  motion_state_msg.odometry.left_wheel = motion_state.odometry.left_wheel;
  motion_state_msg.odometry.right_wheel = motion_state.odometry.right_wheel;

  // system state
  system_state_msg.error_code = static_cast<uint32_t>(system_state.error_code);
  system_state_msg.operational_state = static_cast<uint32_t>(system_state.operational_state);
  system_state_msg.ctrl_state = static_cast<uint32_t>(system_state.ctrl_state);

  system_state_msg.battery_state.voltage = system_state.battery_state.voltage;
  system_state_msg.rc_connected = system_state.rc_connected;
  // actuator state
  for (int i = 0; i < 4; ++i) {
    actuator_state_msg.actuator_state[i].id = actuator_state[i].id;
    actuator_state_msg.actuator_state[i].motor.rpm =
        actuator_state[i].motor.rpm;
    actuator_state_msg.actuator_state[i].motor.current =
        actuator_state[i].motor.current;
    actuator_state_msg.actuator_state[i].motor.pulse_count =
        actuator_state[i].motor.pulse_count;

    actuator_state_msg.actuator_state[i].driver.driver_temperature =
        actuator_state[i].driver.driver_temperature;
    actuator_state_msg.actuator_state[i].driver.motor_temperature =
        actuator_state[i].driver.motor_temperature;
    actuator_state_msg.actuator_state[i].driver.driver_voltage =
        actuator_state[i].driver.driver_voltage;
    actuator_state_msg.actuator_state[i].driver.driver_state =
        actuator_state[i].driver.driver_state;
  }
  // status_msg.motion_state = motion_state;
  // status_msg.system_state = system_state;
  // actuator_state_msgs = actuator_state;

  // light state
  switch (light_state.state.light_mode) {
    case LIGHT_MODE::LIGHT_MODE_BREATH: {
      light_state_msg.light_mode = robot_msgs::LightMode::LIGHT_MODE_BREATH;
      break;
    }
    case LIGHT_MODE::LIGHT_MODE_CONST_OFF: {
      light_state_msg.light_mode = robot_msgs::LightMode::LIGHT_MODE_CONST_OFF;
      break;
    }
    case LIGHT_MODE::LIGHT_MODE_CONST_ON: {
      light_state_msg.light_mode = robot_msgs::LightMode::LIGHT_MODE_CONST_ON;
      break;
    }
    case LIGHT_MODE::LIGHT_MODE_CUSTOM: {
      light_state_msg.light_mode = robot_msgs::LightMode::LIGHT_MODE_CUSTOM;
      break;
    }
  }

  light_state_msg.light_custom = light_state.state.light_custom;

  system_state_publisher_.publish(system_state_msg);
  motion_state_publisher_.publish(motion_state_msg);
  actuator_state_publisher_.publish(actuator_state_msg);
  light_state_publisher_.publish(light_state_msg);

  // publish odometry and tf
  if (system_state.operational_state !=
      SYS_OPER_STATE::SYS_OPER_STATE_ESTOP_ACTIVATED)
    PublishOdometryToROS(motion_state.actual_motion.linear.x,
                         motion_state.actual_motion.angular.z, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void ScoutROSMessenger::PublishSimStateToROS(float linear, float angular) {
  current_time_ = ros::Time::now();

  float dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  // publish scout state message

  robot_msgs::SystemStateMsg system_state_msg;
  robot_msgs::MotionStateMsg motion_state_msg;
  robot_msgs::LightControlType light_state_msg;

  system_state_msg.header.stamp = current_time_;
  motion_state_msg.header.stamp = current_time_;
  light_state_msg.header.stamp = current_time_;

  // status_msg.motion_state.actual_motion.linear_x = linear;
  // status_msg.motion_state.actual_motion.angular_z = angular;

  system_state_msg.operational_state =
      robot_msgs::SysOperState::SYS_OPER_STATE_OPERATIONAL;
  system_state_msg.ctrl_state =
      robot_msgs::SysCtrlState::SYS_CTRL_STATE_CAN_COMMAND_CONTROL;
  system_state_msg.error_code = robot_msgs::SysErrorCode::SYS_ERROR_CODE_NONE;
  system_state_msg.battery_state.voltage = 29.5;

  // for (int i = 0; i < 4; ++i)
  // {
  //     status_msg.motor_states[i].current = state.motor_states[i].current;
  //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
  //     status_msg.motor_states[i].temperature =
  //     state.motor_states[i].temperature;
  // }

  light_state_msg.enable_cmd_light_control = false;
  // status_msg.front_light_state.mode = state.front_light_state.mode;
  // status_msg.front_light_state.custom_value =
  // state.front_light_state.custom_value; status_msg.rear_light_state.mode =
  // state.rear_light_state.mode; status_msg.rear_light_state.custom_value =
  // state.front_light_state.custom_value;

  system_state_publisher_.publish(system_state_msg);
  motion_state_publisher_.publish(motion_state_msg);

  light_state_publisher_.publish(light_state_msg);

  // publish odometry and tf
  PublishOdometryToROS(linear, angular, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void ScoutROSMessenger::PublishOdometryToROS(float linear, float angular,
                                             float dt) {
  // perform numerical integration to get an estimation of pose
  linear_speed_ = linear;
  angular_speed_ = angular;

  float d_x = linear_speed_ * std::cos(theta_) * dt;
  float d_y = linear_speed_ * std::sin(theta_) * dt;
  float d_theta = angular_speed_ * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time_;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;

  tf_msg.transform.translation.x = position_x_;
  tf_msg.transform.translation.y = position_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_quat;

  tf_broadcaster_.sendTransform(tf_msg);

  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_speed_;

  odom_publisher_.publish(odom_msg);
}

bool ScoutROSMessenger::GetControlToken() {
  if (scout_->SdkHasControlToken()) {
    return true;
  }
  HandshakeResultType feedback;
  feedback = scout_->RequestControl();
  if (feedback.code == HANDSHAKE_RESULT::HANDSHAKE_RESULT_CONTROL_ACQUIRED) {
    return true;
  } else if (feedback.code == HANDSHAKE_RESULT_ALREADY_GAINED_CONTROL) {
    return true;
  }
  return false;
}

}  // namespace westonrobot