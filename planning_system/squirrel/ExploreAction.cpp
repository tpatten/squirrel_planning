#include "perceptionInterface.h"

/**
 * ROS node stub for SQUIRREL Summer School: EXPLORE
 * (this code is just a suggestion)
 */
namespace SQUIRREL_summerschool_perception {

	void PerceptionInterface::executeExplore(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		usleep(1000000);
		executeObserve(msg);

	}

} // close namespace
