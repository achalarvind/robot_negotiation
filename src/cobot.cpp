#include <cobot.h>
#include <CSPSolver.h>

ccobot::ccobot(uint cobotId){
	id = cobotId;
	std::cout<<"Attempting to call service\n";
	task_client = nh.serviceClient<robot_negotiation::GetTasks>("/task_generator");
	std::cout<<"Service attempted\n";
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

int count_missed_deadlines(double start, std::vector<robot_negotiation::Task> tasks)  //given the start time of the schedule, how many deadlines are missed in the best case?
{
	int count = 0;
	std::vector<double> deadlines;
	std::vector<double> end_times;
	double last_end = start;
	double curr_end = start;
	//store end times for each task and compare them to deadlines of each task
	for(int i = 0; i < tasks.size(); i++)
	{
		curr_end = last_end + tasks[i].est_time;
		end_times.push_back(curr_end);
		std::cout<<curr_end<<",";
		deadlines.push_back(tasks[i].deadline);
		std::cout<<tasks[i].deadline<<std::endl;
		if(curr_end > tasks[i].deadline) count += 1;
		last_end = curr_end; 
	}
	return count;
}


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

/*int main()
{
	robot_negotiation::TaskList tasks;
    robot_negotiation::Task t1;
    t1.est_time = 1;
    t1.deadline = 3;
    robot_negotiation::Task t2;
    t2.est_time = 3;
    t2.deadline = 3;
    robot_negotiation::Task t3;
    t3.est_time = 2;
    t3.deadline = 3;
    robot_negotiation::Task t4;
    t4.est_time = 4;
    t4.deadline = 10;
    tasks.task_list.push_back(t1);
    tasks.task_list.push_back(t2);
    tasks.task_list.push_back(t3);
    tasks.task_list.push_back(t4);
    std::cout<<count_missed_deadlines(1, tasks.task_list)<<std::endl;
	return 0;
}*/