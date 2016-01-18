#include "squirrel_planning_execution/RPSquirrelRecursion.h"

/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/* constructor */
	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh) : message_store(nh) {
		
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback */
	void RPSquirrelRecursion::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ros::NodeHandle nh;

		// ignore actions
		if(0!=msg->name.compare("classify_object")
				|| 0!=msg->name.compare("examine_area")
				|| 0!=msg->name.compare("explore_area")
				|| 0!=msg->name.compare("tidy_area"))
			return;

		ROS_INFO("KCL: (RPSquirrelRecursion) action recieved");

		// publish feedback (enabled)
		ROS_INFO("KCL: (RPSquirrelRecursion) Started: %s", msg->name.c_str());
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// create new planning system
		PlanningSystem planningSystem;
		SimplePlanDispatcher spd;
		planningSystem.plan_dispatcher = &spd;

		// publishers and such
		planningSystem.state_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/system_state", 5, true);
		planningSystem.plan_publisher = nh.advertise<rosplan_dispatch_msgs::CompletePlan>("/kcl_rosplan/plan", 5, true);
		planningSystem.plan_dispatcher->action_publisher = nh.advertise<rosplan_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		planningSystem.plan_dispatcher->action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 5, true);
		ros::Subscriber feedback_sub = nh.subscribe("/kcl_rosplan/action_feedback", 10, &KCL_rosplan::PlanDispatcher::feedbackCallback, planningSystem.plan_dispatcher);
		ros::Subscriber command_sub = nh.subscribe("/kcl_rosplan/planning_commands", 10, &KCL_rosplan::PlanningSystem::commandCallback, &planningSystem);
		planningSystem.filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/planning_filter", 10, true);
		ros::Subscriber notification_sub = nh.subscribe("/kcl_rosplan/notification", 10, &KCL_rosplan::PlanningSystem::notificationCallBack, &planningSystem);

		
		// HERE GOES PROBLEM CONSTRUCTION AND CALLING R?UNPLAN

		// publish feedback (achieved)
		ROS_INFO("KCL: (RPSquirrelRecursion) Ended: %s", msg->name.c_str());
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_RPSquirrelRecursion");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPSquirrelRecursion rpsr(nh);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPSquirrelRecursion::dispatchCallback, &rpsr);
		ROS_INFO("KCL: (RPSquirrelRecursion) Ready to receive");

		ros::spin();
		return 0;
	}
