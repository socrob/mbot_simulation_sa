/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * provides with topic and actionlib interface to actuate
 * gripper in simulation
 * 
 */

#ifndef MBOT_GRIPPER_CONTROLLER_SIM_SIM_GRIPPER_CONTROLLER_H_
#define MBOT_GRIPPER_CONTROLLER_SIM_SIM_GRIPPER_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <mcr_manipulation_msgs/GripperCommand.h>
#include <limits.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>

class MbotSimGripperInterface
{
public:
    explicit MbotSimGripperInterface(ros::NodeHandle &nh);
    ~MbotSimGripperInterface();

private:
    ros::Duration wait_duration_;

    void jointStatesCallback(const sensor_msgs::JointState::Ptr &msg);
    void gripperCommandCallback(const mcr_manipulation_msgs::GripperCommand::Ptr &msg);
    void followJointTrajectoryGoalCallback();
    void gripperCommandGoalCallback();

    void moveGripper(double position);

    ros::NodeHandle nh_;
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_gripper_command_;
    
    ros::Publisher pub_gripper_joint_command_;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_action_server_;
    control_msgs::GripperCommandFeedback gripper_feedback_;
    control_msgs::GripperCommandResult gripper_result_;

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> trajectory_action_server_;
    control_msgs::FollowJointTrajectoryFeedback trajectory_feedback_;
    control_msgs::FollowJointTrajectoryResult trajectory_result_;

    sensor_msgs::JointState::Ptr joint_states_;
    bool joint_states_received_;

    double gripper_configuration_open_;
    double gripper_configuration_close_;

    // to store the index of gripper joint in /joint_sttates
    int gripper_joint_index_;
};

#endif  // MBOT_GRIPPER_CONTROLLER_SIM_SIM_GRIPPER_CONTROLLER_H_

