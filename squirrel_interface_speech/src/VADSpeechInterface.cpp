#include "squirrel_interface_speech/VADSpeechAction.h"

/* The implementation of RPSpeechAction.h */
namespace KCL_rosplan {

	/* constructor */
	VADSpeechAction::VADSpeechAction(ros::NodeHandle &nh)
		: node_handle_(&nh), message_store_(nh)
	{
		command_stream_ = nh.subscribe<squirrel_vad_msgs::vad>("/voice_detector", 1, &VADSpeechAction::processVADSpeech, this);
	}

	/* action dispatch callback */
	void VADSpeechAction::processVADSpeech(const squirrel_vad_msgs::vad::ConstPtr& msg)
	{
		message_store_.insertNamed("vad", *msg);
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_vad");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::VADSpeechAction rpga(nh);

	ROS_INFO("KCL: (VADSpeechInterface) Ready to receive");

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
