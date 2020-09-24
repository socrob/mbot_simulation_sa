/*
 * Copyright [2017] <Instituto Superior Tecnico>
 *
 * Author: Guilherme Lawless (guilherme.lawless@tecnico.ulisboa.pt)
 *
 * Listens to head_interface/cmd topic for a semantic command with information about where to move the robots head
 * Movement direction and speed are supported
 * More information in monarch_msgs/HeadControlSemantic.msg
 *
 */

#include <mbot_semantic_head_controller/mbot_head_definitions.h>
#include <mbot_semantic_head_controller/semantic_head_controller.h>

namespace semantic_head_controller
{

SemanticHeadControllerServer::SemanticHeadControllerServer()
  : nh_("~"), paramsDir_(3), paramsSpeed_(3)
{
  // ROS subscribers
  highLevelSub_ = nh_.subscribe(
        "cmd", 1, &SemanticHeadControllerServer::semanticRequestCallback, this);

  // ROS publishers
  lowLevelPub_ = nh_.advertise<std_msgs::UInt8MultiArray>("low_level_cmd", 1);

  // Get parameters from ROS parameter server
  getParams();

  // Populate the class map variables, so it is easier to access later in the
  // callback function
  populateMaps();
}

void SemanticHeadControllerServer::getParams()
{
  // Get speed params
  // Be sure that it does not go above max speed
  nh_.param<double>("speed/slow", paramsSpeed_[OFFSET_SLOW],
                    command_speed_default::slow);
  nh_.param<double>("speed/normal", paramsSpeed_[OFFSET_NORMAL],
                    command_speed_default::normal);
  nh_.param<double>("speed/fast", paramsSpeed_[OFFSET_FAST],
                    command_speed_default::fast);

  nh_.param<int>("directions/center", paramsDir_[OFFSET_CENTER],
                    command_directions_default::center);
  nh_.param<int>("directions/left", paramsDir_[OFFSET_LEFT],
                    command_directions_default::left);
  nh_.param<int>("directions/right", paramsDir_[OFFSET_RIGHT],
                    command_directions_default::right);

  // Report parameters to user
  ROS_DEBUG("Using parameter speed/slow = %f", paramsSpeed_[OFFSET_SLOW]);
  ROS_DEBUG("Using parameter speed/normal = %f", paramsSpeed_[OFFSET_NORMAL]);
  ROS_DEBUG("Using parameter speed/fast = %f", paramsSpeed_[OFFSET_FAST]);
  ROS_DEBUG("Using parameter directions/center = %d",
           paramsDir_[OFFSET_CENTER]);
  ROS_DEBUG("Using parameter directions/left = %d", paramsDir_[OFFSET_LEFT]);
  ROS_DEBUG("Using parameter directions/right = %d", paramsDir_[OFFSET_RIGHT]);
}

void SemanticHeadControllerServer::populateMaps()
{
  using monarch_msgs::HeadControlSemantic;

  // Populate the maps with available commands
  // Speed commands and info messages
  speedMap[HeadControlSemantic::SLOW] = paramsSpeed_[OFFSET_SLOW];
  speedMessageMap[HeadControlSemantic::SLOW] = "SLOW";

  speedMap[HeadControlSemantic::NORMAL] = paramsSpeed_[OFFSET_NORMAL];
  speedMessageMap[HeadControlSemantic::NORMAL] = "NORMAL";

  speedMap[HeadControlSemantic::FAST] = paramsSpeed_[OFFSET_FAST];
  speedMessageMap[HeadControlSemantic::FAST] = "FAST";

  // Cardinal direction commands and info messages
  posMap["N"] = paramsDir_[OFFSET_CENTER];
  posMap["NORTH"] = paramsDir_[OFFSET_CENTER];
  posMessageMap["N"] = "NORTH";
  posMessageMap["NORTH"] = "NORTH";

  posMap["W"] = paramsDir_[OFFSET_LEFT];
  posMap["WEST"] = paramsDir_[OFFSET_LEFT];
  posMessageMap["W"] = "WEST";
  posMessageMap["WEST"] = "WEST";

  posMap["E"] = paramsDir_[OFFSET_RIGHT];
  posMap["EAST"] = paramsDir_[OFFSET_RIGHT];
  posMessageMap["E"] = "EAST";
  posMessageMap["EAST"] = "EAST";

  posMap["NW"] = (paramsDir_[OFFSET_CENTER] + paramsDir_[OFFSET_LEFT]) / 2.0;
  posMap["NORTHWEST"] =
      (paramsDir_[OFFSET_CENTER] + paramsDir_[OFFSET_LEFT]) / 2.0;
  posMessageMap["NW"] = "NORTHWEST";
  posMessageMap["NORTHWEST"] = "NORTHWEST";

  posMap["NE"] = (paramsDir_[OFFSET_CENTER] + paramsDir_[OFFSET_RIGHT]) / 2.0;
  posMap["NORTHEAST"] =
      (paramsDir_[OFFSET_CENTER] + paramsDir_[OFFSET_RIGHT]) / 2.0;
  posMessageMap["NE"] = "NORTHEAST";
  posMessageMap["NORTHEAST"] = "NORTHEAST";
}

void SemanticHeadControllerServer::semanticRequestCallback(
    const monarch_msgs::HeadControlSemanticConstPtr& msg)
{
  std_msgs::UInt8MultiArray controlMsg;
  controlMsg.data.resize(2);

  std::string pos_msg;
  std::string speed_msg;

  try
  {
    controlMsg.data[1] = speedMap.at(msg->speed);
    speed_msg = speedMessageMap.at(msg->speed);
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("Unsupported speed %d", msg->speed);
    return;
  }

  try
  {
    controlMsg.data[0] = posMap.at(msg->cardinal_direction);
    pos_msg = posMessageMap.at(msg->cardinal_direction);
  }

  catch (const std::out_of_range& oor)
  {
    ROS_ERROR_STREAM("Unsupported cardinal direction "
                     << msg->cardinal_direction);
    return;
  }

  ROS_INFO_STREAM("Turning head to " << pos_msg << " with " << speed_msg
                  << " speed");

  lowLevelPub_.publish(controlMsg);
}

// End of namespace semantic_head_controller
}

int main(int argc, char** argv)
{
  // Node init with default name (launch file will override)
  ros::init(argc, argv, "semantic_head_controller_server");

  ros::NodeHandle nh;

  // Default frequency at 10.0Hz, enough for a semantic server
  ros::Rate rrate(10);

  ROS_INFO("Initializing semantic head controller server node");

  // create object of the node class (SemanticHeadController)
  semantic_head_controller::SemanticHeadControllerServer server_node;

  ROS_INFO("Node initialized.");

  while (ros::ok())
  {
    ros::spinOnce();
    rrate.sleep();
  }
  return EXIT_SUCCESS;
}
