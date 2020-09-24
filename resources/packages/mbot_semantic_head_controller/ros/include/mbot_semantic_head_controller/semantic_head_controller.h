#ifndef SEMANTIC_HEAD_CONTROLLER_H
#define SEMANTIC_HEAD_CONTROLLER_H

#include <ros/ros.h>
#include <monarch_msgs/HeadControlSemantic.h>
#include <std_msgs/UInt8MultiArray.h>
#include <map>

namespace semantic_head_controller
{
#define OFFSET_SLOW 0
#define OFFSET_NORMAL 1
#define OFFSET_FAST 2

#define OFFSET_CENTER 0
#define OFFSET_LEFT 1
#define OFFSET_RIGHT 2

class SemanticHeadControllerServer
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber highLevelSub_;
  ros::Publisher lowLevelPub_;

  std::vector<double> paramsSpeed_;
  std::vector<int> paramsDir_;

  void getParams();
  void populateMaps();

protected:
  std::map<uint8_t, float> speedMap;
  std::map<uint8_t, std::string> speedMessageMap;
  std::map<std::string, uint8_t> posMap;
  std::map<std::string, std::string> posMessageMap;

public:
  /**
    * @brief SemanticHeadControllerServer - class constructor
    */
  SemanticHeadControllerServer();

  /**
    * @brief semanticRequestCallback - callback for semantic message requests
    * @param msg
    */
  void
  semanticRequestCallback(const monarch_msgs::HeadControlSemanticConstPtr& msg);
};
// End of namespace semantic_head_controller
}

#endif
