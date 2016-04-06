#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_FINALISECLASSIFYOBJECTPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_FINALISECLASSIFYOBJECTPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

namespace KCL_rosplan
{

/**
 * An instance of this class gets called whenever the PDDL action 'finalise_classify*' (or variants thereof) is
 * dispatched. These actions do nothing but internal book keeping for the planner.
 */
class FinaliseClassificationPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 */
	FinaliseClassificationPDDLAction(ros::NodeHandle& node_handle);
	
	/**
	 * Destructor
	 */
	~FinaliseClassificationPDDLAction();
	
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
