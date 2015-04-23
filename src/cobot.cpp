#include <cobot.h>

ccobot::ccobot(){
	task_client = nh.serviceClient<robot_negotiation::GetTasks>("task_generator");
	if (task_client.call(srv))
	{
		std::cout<<srv.response.tasks.task_list[0];
	}
	else
	{
		ROS_ERROR("Failed to call service get services");
	}	
}

int main(int argc, char **argv){
	ros::init(argc, argv, "cobot");
	ccobot robot;
	ros::spin();
	return 0;
}