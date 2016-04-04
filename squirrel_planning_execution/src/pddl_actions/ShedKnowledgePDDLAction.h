#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_SHEDKNOWLEDGEPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_SHEDKNOWLEDGEPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

namespace KCL_rosplan
{

/**
 * An instance of this class gets called whenever the PDDL action 'shed_knowledge' is
 * dispatched. It is an action only intended for managing the contingency planning translation
 * and is not connected to any activity to be performed by the robot.
 */
class ShedKnowledgePDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 */
	ShedKnowledgePDDLAction(ros::NodeHandle& node_handle);
	
	/**
	 * Destructor
	 */
	~ShedKnowledgePDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	ros::Publisher action_feedback_pub_;         // Publisher that communicates feedback to ROSPlan.
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
};

};

#endif
