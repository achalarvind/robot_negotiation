#include <cobot.h>
#include <CSPSolver.h>

ccobot::ccobot(uint cobotId){
	id = cobotId;
	task_client = nh.serviceClient<robot_negotiation::GetTasks>("../task_generator");
	if (task_client.call(srv))
	{
		ROS_INFO("Cobot tasks list populated");
		tasks=srv.response.tasks;
		// std::cout<<tasks.task_list[0];
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
 	return costs;
 }

double ccobot::plan_cost(std::vector<DeliveryOrderSeq> plan)
{

	std::string s;
	DeliveryOrderSeq a(1,s,0.0,0.0);
	for(int i =0; i < plan.size(); i++)
	{
		//if(plan[i].m_iCobotNum == id) {a.set(plan[i]);  break;}
	}
	//count_missed_deadlines(a.m_dExpectedTime,tasks);
	// std::string s;
	// CSPSolver::DeliveryOrderSeq a(1,s,0.0,0.0);
	// for(int i =0; i < plan.size(); i++)
	// {
	// 	if(plan[i].m_iCobotNum == id) {a = plan[i];  break;}
	// }
        return 0;	
}

//int count_missed_deadlines(a.m_dExpectedTime,tasks)
//{
//	return 0;
//}




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
