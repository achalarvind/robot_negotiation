#include <Environment.h>
#include "ros/ros.h"
#include "robot_negotiation/GetTasks.h"
#include <cstdlib>

class ccobot{
	private:
		ros::NodeHandle nh;
		ros::Publisher negotiation_pub;
		ros::ServiceClient task_client, distance_client;
		robot_negotiation::GetTasks srv;
		int state;
		int location;
		bool has_object;

	public:
		void start_negotiation();
		ccobot();	
};	