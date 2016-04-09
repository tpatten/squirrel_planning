#include <iostream>

#include <ros/ros.h>
#include <squirrel_speech_msgs/RecognizedCommand.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "speech_simulator");
	ros::NodeHandle nh;

	ros::Publisher command_pub = nh.advertise<squirrel_speech_msgs::RecognizedCommand>("/squirel_speech_rec/squirrel_speech_recognized_commands", 1);

	ROS_INFO("KCL: (SpeechSimulator) Ready to receive");

	while (ros::ok())
	{
		std::string command;
		std::cin >> command;
		
		ROS_INFO("KCL: (SpeechSimulator) Publish the command %s", command.c_str());
		squirrel_speech_msgs::RecognizedCommand rc;
		rc.header.stamp = ros::Time::now();
		rc.int_command = command;
		rc.recognized_speech = command;
		rc.parsed_speech = command;
		rc.is_command = true;
		rc.speaker_ID = 0;
		
		command_pub.publish(rc);
		
		ros::spinOnce();
	}
	return 0;
}
