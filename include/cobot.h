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
<<<<<<< HEAD
		robot_negotiation::TaskList tasks;

	public:
		std::vector<double> vote(std::vector<DeliveryOrderSeq>);
		double plan_cost(std::vector<DeliveryOrderSeq> plan);
=======
		std::vector<DeliveryOrderSeq> taskList;

	public:
		std::vector<double> vote(std::vector<DeliveryOrderSeq>); //gives list of costs for each plan
		double plan_cost(std::vector<DeliveryOrderSeq> plan)
>>>>>>> b28ac47d3996ab4e4d8dd9c4cdbf009d7e4cad27
		ccobot(uint cobotId);	
};	