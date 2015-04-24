#include <cobot.h>

ccobot::ccobot(){
	task_client = nh.serviceClient<robot_negotiation::GetTasks>("task_generator");
	if (task_client.call(srv))
	{
		std::cout<<srv.response.tasks.task_list[0];
	}
	else
	{
		ROS_ERROR("Failed to call service task_generator");
	}	

	// ros::ServiceServer distance_service =nh.advertiseService<robot_negotiation::GetDistance::Request, robot_negotiation::GetDistance::Response>("/get_distance", boost::bind(getDistance, _1, _2, pWorld));

}

int main(int argc, char **argv){
	ros::init(argc, argv, "cobot");
	ccobot robot;
	ros::spin();
	return 0;
}