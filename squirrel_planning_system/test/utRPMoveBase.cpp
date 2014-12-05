#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "move_base_msgs/MoveBaseAction.h"

class TestRPMoveBase
{
protected:

  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; 
  std::string action_name_;

  // create messages that are used to published feedback/result
  move_base_msgs::MoveBaseFeedback feedback_;
  move_base_msgs::MoveBaseResult result_;

public:

  TestRPMoveBase(std::string name) :
    as_(nh_, name, boost::bind(&TestRPMoveBase::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~TestRPMoveBase(void)
  {
  }

  void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;


    // publish info to the console for the user
    ROS_INFO("Executing");

    if(success)
    {
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");

  TestRPMoveBase trpmb(ros::this_node::getName());
  ros::spin();

  return 0;
}
