#include <Environment.h>
#include "ros/ros.h"
#include "robot_negotiation/GetTasks.h"
#include "robot_negotiation/Task.h"
#include "robot_negotiation/TaskList.h"
#include <cstdlib>
#include <CSPSolver.h>

class ccobot{
	private:
		ros::NodeHandle nh;
		ros::Publisher negotiation_pub;
		ros::ServiceClient task_client, distance_client;
		robot_negotiation::GetTasks srv;
		uint id;
		int state;
		int location;
		bool has_object;


	public:
		ccobot(uint cobotId);

		robot_negotiation::TaskList tasks;
		std::vector<double> vote(std::vector<std::vector<DeliveryOrderSeq>> plan_list);
		double plan_cost(std::vector<DeliveryOrderSeq> plan);
		//int count_missed_deadlines(double,robot_negotiation::TaskList);


		
};	
