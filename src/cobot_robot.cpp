#include "cobot.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "cobot");	
	ros::NodeHandle n("~");
	int cobot_id;
	std::string cobot_type;
	n.getParam("cobot_id", cobot_id);
	n.getParam("cobot_type", cobot_type);
	ROS_INFO("Cobot with ID: %d spawned", cobot_id);
	ccobot robot(cobot_id, cobot_type);	
	ros::spin();
	return 0;
}