#include "cobot.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "cobot");	
	ros::NodeHandle n("~");
	int cobot_id;
	n.getParam("cobot_id", cobot_id);
	ROS_INFO("Cobot with ID: %d spawned",cobot_id);
	ccobot robot(cobot_id);	
	ros::spin();
	return 0;
}