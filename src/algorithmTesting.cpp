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
// #include "robot_negotiation/GetDistanceStochastic.h"
#include <CobotQueueHandler.h>

using namespace std;

const int NCobots = 10;




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

    //gather first task of each cobot
    std::vector<robot_negotiation::Action> tasks_to_plan;
    for(int i = 0; i < NCobots; i++)
    {
    	Action a;
    	a.cobot_id = i;
    	a.object_type = queue.cobots[i].tasks[0].object_id;
    	a.deadline = queue.cobots[i].tasks[0].deadline;
    	tasks_to_plan.push_back(a);
    }












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



	ROS_INFO("started_node");
    ros::spin();
	delete(pWorld);

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
