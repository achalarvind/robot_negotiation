#include <cobot.h>

ccobot::ccobot(uint cobotId){
	id = cobotId;
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

std::vector<double> ccobot::vote(std::vector<std::vector<DeliveryOrderSeq>> plan_list)
{
	std::vector<double> costs;
	for(int i =0; i < plan_list.size(); i++)
	{
		costs.push_back(plan_cost(plan_list[i]));
	}
}

double ccobot::plan_cost(std::vector<DeliveryOrderSeq> plan)
{
	for(int i =0; i < plan.size(); i++)
	{
		if(plan.m_iCobotNum == id) break;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "cobot");
	ccobot robot;
	ros::spin();
	return 0;
}