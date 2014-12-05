/**
 * This file includes the main loop of the planning node.
 * Three methods define the loop.  The first "runPlanningServer" iterates through actions and dispatches them.
 * Within this loop there are calls to "generatePlanningProblem" and "runPlanner".
 * Also in this file are the methods for pushing the planning filter (a list of objects and facts that the plan relies
 * upon) and the callback for notifications (when something in the filter changes, or new information becomes available.)
 * The latter is a hook to add domain-specific reasoning to the system.
 */

#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "squirrel_planning_knowledge_msgs/Notification.h"
#include "squirrel_planning_knowledge_msgs/Filter.h"
#include "squirrel_planning_dispatch_msgs/ActionFeedback.h"

#include "squirrel_planning_system/Utilities.h"
#include "squirrel_planning_system/PlanningEnvironment.h"
#include "squirrel_planning_system/ActionDispatch.h"

#include "PDDLProblemGenerator.cpp"
#include "ActionFeedback.cpp"
#include "PostProcess.cpp"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>

namespace KCL_rosplan {

	/**
	 * Runs external commands
	 */
	std::string runCommand(std::string cmd)
	{
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	void pauseDispatchCallback(const std_msgs::Empty::ConstPtr& msg) {
		if(!KCL_rosplan::dispatchPaused) {
			ROS_INFO("KCL: Pausing dispatch");
			dispatchPaused = true;
		} else {
			ROS_INFO("KCL: Resuming dispatch");
			dispatchPaused = false;
		}
	}

	/*-----------*/
	/* Knowledge */
	/*-----------*/

	/**
	 * listen to and process "/kcl_rosplan/notification" topic.
	 * TODO this method should be made a clean interface to user code.  It is here that the real reasoning takes place.
	 * The return type of this method should be a boolean (replan).
	 */
	void notificationCallBack(const squirrel_planning_knowledge_msgs::Notification::ConstPtr& msg) {
		ROS_INFO("KCL: Notification received; plan invalidated; replanning.");
		replanRequested = true;
	}

	/**
	 * pushes the filter: the list of items the planner cares about in its plan.
	 */
	void publishFilter() {

		// clear the old filter
		squirrel_planning_knowledge_msgs::Filter filterMessage;
		filterMessage.function = squirrel_planning_knowledge_msgs::Filter::CLEAR;
		filterPublisher.publish(filterMessage);

		// push the new filter
		ROS_INFO("KCL: Clean and update knowledge filter");
		filterMessage.function = squirrel_planning_knowledge_msgs::Filter::ADD;
		filterMessage.knowledge_items = knowledgeFilter;
		filterPublisher.publish(filterMessage);
	}

	/*----------*/
	/* Planning */
	/*----------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 * This method is popf-specific.
	 */
	bool runPlanner(std::string &dataPath, std::string &domainPath, std::string &problemPath, std::string &plannerCommand)
	{
		// save previous plan
		KCL_rosplan::planningAttempts = KCL_rosplan::planningAttempts + 1;
		if(KCL_rosplan::actionList.size() > 0) {
			std::vector< squirrel_planning_dispatch_msgs::ActionDispatch> oldplan(KCL_rosplan::actionList);
			KCL_rosplan::planList.push_back(oldplan);
			KCL_rosplan::planListLastAction.push_back(currentAction);
		}

		replanRequested = false;

		// run the planner
		std::string commandString = plannerCommand + " "
			 + domainPath + " " + problemPath + " > "
			 + dataPath + "plan.pddl";

		ROS_INFO("KCL: Running: %s", commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: Planning complete");

		// check the planner solved the problem
		std::ifstream planfile;
		planfile.open((dataPath + "plan.pddl").c_str());
		std::string line;
		while(!planfile.eof()) {
			getline(planfile, line);
			if (line.substr(0,22).compare(";; Problem unsolvable!")==0) { 
				planfile.close();
				ROS_INFO("Plan was unsolvable.");
				return false;
			}
		}
		planfile.close();
		
		ROS_INFO("KCL: Post processing plan");

		// trim the end of any existing plan
		while(KCL_rosplan::actionList.size() > KCL_rosplan::currentAction) {
			KCL_rosplan::actionList.pop_back();
		}
		KCL_rosplan::freeActionID = KCL_rosplan::currentAction;

		// Convert plan into message list for dispatch
		preparePlan(dataPath);

		// publish the filter to the knowledge base
		publishFilter();

		return true;
	}

	/**
	 * requests objects; generates the PDDL problem.
	 */
	bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath)
	{
		// update the environment from the ontology
		ROS_INFO("KCL: Fetching objects");
		updateEnvironment(nh);

		// generate PDDL problem
		generatePDDLProblemFile(problemPath);

		return true;
	}

	/**
	 * sets up the ROS node; prepares planning; main loop.
	 */
	void runPlanningServer()
	{
		ros::NodeHandle nh("~");

		// setup environment
		std::string dataPath;
		std::string domainPath;
		std::string problemPath;
		std::string plannerCommand;
		nh.param("data_path", dataPath, std::string("common/"));
		nh.param("domain_path", domainPath, std::string("common/domain.pddl"));
		nh.param("problem_path", problemPath, std::string("common/problem.pddl"));
		nh.param("planner_command", plannerCommand, std::string("timeout 10 common/bin/popf"));
		ROS_INFO("KCL: Using data path: %s", dataPath.c_str());
		KCL_rosplan::parseDomain(domainPath);

		// publishing "action_dispatch"; listening "action_feedback"
		actionPublisher = nh.advertise< squirrel_planning_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		feedbackSub = nh.subscribe("/kcl_rosplan/action_feedback", 1000, KCL_rosplan::feedbackCallback);
		ros::Subscriber pauseDispatchSub = nh.subscribe("/kcl_rosplan/pause_dispatch", 1000, KCL_rosplan::pauseDispatchCallback);
		ros::Rate loop_rate(10);

		// publishing "/kcl_rosplan/filter"; listening "/kcl_rosplan/notification"
		filterPublisher = nh.advertise<squirrel_planning_knowledge_msgs::Filter>("/kcl_rosplan/filter", 10, true);
		notificationSub = nh.subscribe("/kcl_rosplan/notification", 100, KCL_rosplan::notificationCallBack);
	
		// generate PDDL problem and run planner
		generatePlanningProblem(nh, problemPath);
		runPlanner(dataPath, domainPath, problemPath, plannerCommand);

		// Loop through and publish planned actions
		while (ros::ok() && KCL_rosplan::actionList.size() > KCL_rosplan::currentAction) {

			squirrel_planning_dispatch_msgs::ActionDispatch currentMessage = KCL_rosplan::actionList[KCL_rosplan::currentAction];
			if((unsigned int)currentMessage.action_id != KCL_rosplan::currentAction)
				ROS_INFO("KCL: ERROR message action_id [%d] does not meet expected [%zu]", currentMessage.action_id, KCL_rosplan::currentAction);

			// loop while dispatch is paused
			while(ros::ok() && KCL_rosplan::dispatchPaused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// dispatch action
			ROS_INFO("KCL: Dispatching action: [%i, %s, %f]", currentMessage.action_id, currentMessage.name.c_str(), currentMessage.duration);
			actionPublisher.publish(currentMessage);

			// callback and sleep
			bool sentCancel = false;
			while (ros::ok() && !KCL_rosplan::actionCompleted[KCL_rosplan::currentAction]) {
				ros::spinOnce();
				loop_rate.sleep();

				if(replanRequested && !sentCancel) {
					// cancel current action
					ROS_INFO("KCL: Cancelling action: [%i, %s]", currentMessage.action_id, currentMessage.name.c_str());
					 squirrel_planning_dispatch_msgs::ActionDispatch cancelMessage;
					cancelMessage.action_id = KCL_rosplan::currentAction;
					cancelMessage.name = "cancel_action";
					actionPublisher.publish(cancelMessage);
					sentCancel = true;
				}
			}

			// get ready for next action
			KCL_rosplan::currentAction++;
			KCL_rosplan::actionReceived[KCL_rosplan::currentAction] = false;
			KCL_rosplan::actionCompleted[KCL_rosplan::currentAction] = false;

			// generate PDDL problem and (re)run planner
			if(replanRequested || KCL_rosplan::actionList.size() <= KCL_rosplan::currentAction) {
				generatePlanningProblem(nh, problemPath);
				runPlanner(dataPath, domainPath, problemPath, plannerCommand);
			}
		}
		ROS_INFO("KCL: Planning Complete");
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {
		ros::init(argc,argv,"rosplan_planning_system");
		srand (static_cast <unsigned> (time(0)));
		KCL_rosplan::runPlanningServer();
		return 0;
	}
