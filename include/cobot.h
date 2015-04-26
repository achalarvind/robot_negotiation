#include <Environment.h>
#include "ros/ros.h"
#include "robot_negotiation/GetTasks.h"
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
		std::vector<DeliveryOrderSeq> taskList;

	public:
		std::vector<double> vote(std::vector<DeliveryOrderSeq>); //gives list of costs for each plan
		double plan_cost(std::vector<DeliveryOrderSeq> plan)
		ccobot(uint cobotId);	
};	