/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * provides with topic and actionlib interface to actuate
 * gripper in simulation
 * 
 */

#include <string>
#include <vector>

#include <mbot_gripper_controller_sim/sim_gripper_controller.h>

MbotSimGripperInterface::MbotSimGripperInterface(ros::NodeHandle &nh) :
    joint_states_received_(false),
    gripper_action_server_(nh, "gripper_controller/gripper_command", false),
    trajectory_action_server_(nh, "gripper_controller/follow_joint_trajectory", false)
{
    sub_gripper_command_ = nh_.subscribe("gripper_command", 1,
                                         &MbotSimGripperInterface::gripperCommandCallback, this);

    sub_joint_states_ = nh_.subscribe("joint_states", 10,
                                  &MbotSimGripperInterface::jointStatesCallback, this);

    pub_gripper_joint_command_ = nh_.advertise<std_msgs::Float64>("gripper_actuation_sim", 1);

    // read parameters
    ROS_INFO("Parameters:");
    ros::NodeHandle nh_prv("~");

    nh_prv.param<double>("gripper_configuration_open", gripper_configuration_open_, 0.0);
    ROS_INFO_STREAM("\tgripper configuration <open>: " << gripper_configuration_open_);

    nh_prv.param<double>("gripper_configuration_close", gripper_configuration_close_, 1.0);
    ROS_INFO_STREAM("\tgripper configuration <close>: " << gripper_configuration_close_);

    // ensure gripper param is set
    if (!nh_prv.hasParam("gripper_joint_name"))
    {
        ROS_ERROR("gripper joint name param not set, node will not be able to actuate gripper in simulation!!");
        ros::shutdown();
    }

    std::string gripper_joint_name;

    nh_prv.param<std::string>("gripper_joint_name", gripper_joint_name, "my_gripper_joint_name");
    ROS_INFO_STREAM("\tgripper_joint_name :" << gripper_joint_name);

    // start action server
    trajectory_action_server_.registerGoalCallback(boost::bind(&MbotSimGripperInterface::followJointTrajectoryGoalCallback, this));
    trajectory_action_server_.start();
    gripper_action_server_.registerGoalCallback(boost::bind(&MbotSimGripperInterface::gripperCommandGoalCallback, this));
    gripper_action_server_.start();

    ros::Rate loop_rate(30);

    // get the gripper index in /joint_states
    while (!joint_states_received_ && ros::ok())
    {
        // listen to callbacks to update joint states
        ros::spinOnce();
        loop_rate.sleep();
    }

    // find gripper joint name inside joint_states
    std::vector<std::string>::iterator it = std::find(joint_states_->name.begin(), joint_states_->name.end(), gripper_joint_name);
    if (it == joint_states_->name.end())
    {
        // name not in vector
        ROS_ERROR("gripper joint name (%s) is not inside /joint_sates topic! node will terminate now...", gripper_joint_name.c_str());
        ros::shutdown();
    }
    else
    {
        // find index of gripper joint name inside /joint_states topic
        gripper_joint_index_ = std::distance(joint_states_->name.begin(), it);
    }
    
    ROS_DEBUG("found gripper index : %d", gripper_joint_index_);
}

MbotSimGripperInterface::~MbotSimGripperInterface()
{
    sub_gripper_command_.shutdown();
    pub_gripper_joint_command_.shutdown();
}

void MbotSimGripperInterface::jointStatesCallback(const sensor_msgs::JointState::Ptr &msg)
{
    joint_states_ = msg;
    joint_states_received_ = true;
}

void MbotSimGripperInterface::gripperCommandCallback(const mcr_manipulation_msgs::GripperCommand::Ptr &msg)
{
    if (msg->command == mcr_manipulation_msgs::GripperCommand::OPEN)
    {
        moveGripper(gripper_configuration_open_);
    }
    else if (msg->command == mcr_manipulation_msgs::GripperCommand::CLOSE)
    {
        moveGripper(gripper_configuration_close_);
    }
    else
    {
        ROS_ERROR_STREAM("Unsupported gripper command: " << msg->command);
        return;
    }
}

void MbotSimGripperInterface::moveGripper(double position)
{
    ROS_DEBUG("moving sim gripper");
    
    ros::Rate loop_rate(30);

    // publish goal position
    std_msgs::Float64 gripper_pos;
    gripper_pos.data = position;
    pub_gripper_joint_command_.publish(gripper_pos);
    joint_states_received_ = false;

    ROS_INFO("Waiting for gripper sim to stop moving...");
    while (ros::ok())
    {
        while (!joint_states_received_ && ros::ok())
        {
            // listen to callbacks to update joint states
            ros::spinOnce();
            loop_rate.sleep();
        }

        joint_states_received_ = false;
        
        // check if gripper is still moving
        bool is_gripper_moving = false;
        
        if (joint_states_->velocity[gripper_joint_index_] < 0.0001)
        {
            // gripper joint speed is close to zero, so it stopped moving
            break;
        }
    }
    ROS_INFO("Gripper sim stopped moving, publishing result");

    //gripper_result_.position = joint_states_->current_pos; //TODO
    gripper_result_.effort = 0.0;
    gripper_result_.stalled = false;
    gripper_result_.reached_goal = true;

    trajectory_result_.error_code = trajectory_result_.SUCCESSFUL;
}

void MbotSimGripperInterface::gripperCommandGoalCallback()
{
    double set_pos = gripper_action_server_.acceptNewGoal()->command.position;
    moveGripper(set_pos);
    gripper_action_server_.setSucceeded(gripper_result_);
}

void MbotSimGripperInterface::followJointTrajectoryGoalCallback()
{
    control_msgs::FollowJointTrajectoryGoal::ConstPtr goal = trajectory_action_server_.acceptNewGoal();

    const std::vector<trajectory_msgs::JointTrajectoryPoint> *points = &goal->trajectory.points;
    const trajectory_msgs::JointTrajectoryPoint *point = &points->back();
    double set_pos = point->positions.back();
    moveGripper(set_pos);

    trajectory_action_server_.setSucceeded(trajectory_result_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mbot_sim_gripper_interface");
    ros::NodeHandle nh;

    MbotSimGripperInterface gripper(nh);

    ros::spin();

    return 0;
}
