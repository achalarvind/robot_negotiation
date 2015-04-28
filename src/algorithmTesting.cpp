#include <WorldMap.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/bind.hpp>
#include <iostream>
// #include "Temp.h"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "robot_negotiation/GetDistance.h"
#include "robot_negotiation/DeSerializeEnvironmentPlan.h"
#include "robot_negotiation/Action.h"
#include "robot_negotiation/Plan.h"
#include "robot_negotiation/GetTasks.h"

// #include "robot_negotiation/GetDistanceStochastic.h"
#include <CobotQueueHandler.h>

using namespace std;

const int NCobots = 10;

std::string baxter_location;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
 	ros::NodeHandle n("~");
    //create multiple cobots in queue
    std::vector<uint> cobotIds;
    for(int i = 0; i < NCobots; i++)
    {
    	cobotIds.push_back(i);
    }
    cobotQueue queue(cobotIds,NCobots);

    ros::ServiceClient planning_client=n.serviceClient<robot_negotiation::DeSerializeEnvironmentPlan>("/deserialize_environment");
    robot_negotiation::DeSerializeEnvironmentPlan srv_planning;

    //gather first task of each cobot
    for(int i = 0; i < NCobots; i++)
    {
    	robot_negotiation::Action a;
    	a.cobot_id = i;
    	a.object_type = queue.cobots[i].tasks[0].object_id;
    	a.deadline = queue.cobots[i].tasks[0].deadline;
    	srv_planning.request.plan.push_back(a);
    }
    srv_planning.request.baxter_location=baxter_location;
    srv_planning.request.start_time=ros::Time::now().toSec()+10.0f;

    if (planning_client.call(srv_planning))
    {
        ROS_INFO("planning complete");
        // std::cout<<tasks.task_list[0];
    }
    else
    {
        ROS_ERROR("Failed to call service deserialize_environment. No plan generated");
    }   

    robot_negotiation::TaskList tasks;
    robot_negotiation::GetTasks srv;
    ros::ServiceClient task_client = n.serviceClient<robot_negotiation::GetTasks>("/task_generator");
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



	ROS_INFO("started_node");
    ros::spin();

	return 1;
}

//int main()
//{
//	Temp obj;
//	obj.Initialize();
//	
//
//	//save file
//	std::ofstream ofs("worldmap.txt");
//	boost::archive::text_oarchive oa(ofs);
//	oa & obj;
//	ofs.close();
//
//	Temp newobj;
//	std::ifstream ifs("worldmap.txt");
//	boost::archive::text_iarchive ia(ifs);
//	ia >> newobj;
//
//
//	/*for (int icount = 0; icount < newobj.m_DoubleVector.size(); icount++)
//	{
//		cout << newobj.m_DoubleVector[icount];
//	}*/
//
//	for (int icount = 0; icount < newobj.m_DoubleHash.size(); icount++)
//	{
//		cout << newobj.m_DoubleHash.find(icount)->second;
//	}
//
//	return 1;
//}
