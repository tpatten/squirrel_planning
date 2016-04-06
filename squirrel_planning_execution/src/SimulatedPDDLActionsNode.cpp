#include <ros/ros.h>

#include "pddl_actions/GotoPDDLAction.h"
#include "pddl_actions/ExploreWaypointPDDLAction.h"
#include "pddl_actions/ClearObjectPDDLAction.h"
#include "pddl_actions/ClassifyObjectPDDLAction.h"
#include "pddl_actions/PutObjectInBoxPDDLAction.h"
#include "pddl_actions/TidyObjectPDDLAction.h"
#include "pddl_actions/PickupPDDLAction.h"
#include "pddl_actions/PushObjectPDDLAction.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_SimluatedPDDLActionsNode");
	ros::NodeHandle nh;
	
	// Setup all the simulated actions.
	KCL_rosplan::GotoPDDLAction goto_action(nh);
	KCL_rosplan::ExploreWaypointPDDLAction explore_waypoint_action(nh);
	KCL_rosplan::ClearObjectPDDLAction clear_object_action(nh);
	KCL_rosplan::ClassifyObjectPDDLAction classify_object_action(nh, 0.5f);
	KCL_rosplan::PutObjectInBoxPDDLAction pub_object_in_box_action(nh);
	KCL_rosplan::TidyObjectPDDLAction tidy_object_action(nh);
	KCL_rosplan::PickupPDDLAction pickup_action(nh);
	KCL_rosplan::PushObjectPDDLAction push_object_action(nh);

	ROS_INFO("KCL: (SimulatedPDDLActionsNode) All simulated actions are ready to receive.");
	
	ros::spin();
	return 0;
}
