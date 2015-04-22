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
#include <test.cpp>

using namespace std;

bool getDistance(robot_negotiation::GetDistance::Request  &req,
         robot_negotiation::GetDistance::Response &res,
         WorldMap *pWorld)
{
	res.distance=pWorld->GetDistance(req.source,req.destination);
	return true;
}

bool GetDistanceStochastic(robot_negotiation::GetDistance::Request  &req,
         robot_negotiation::GetDistance::Response &res,
         WorldMap *pWorld)
{
	std::random_device rd;
	std::default_random_engine generator(rd());
	std::poisson_distribution <int> delay (10);
	double delay_factor=(1+(delay(generator)/100.0f));
	ROS_INFO("delay factor is %f",delay_factor);
	res.distance=(delay_factor*pWorld->GetDistance(req.source,req.destination));
	return true;
}

int main(int argc, char** argv)
{
	//Serialization for WorldMap
	//WorldMap *pWorld = new WorldMap();

	////Save file
	//std::ofstream ofs("WorldMap.txt");
	//boost::archive::text_oarchive oa(ofs);
	//oa & *pWorld;
	//ofs.close();

	////Load from file
	//WorldMap obNewWorld;
	//std::ifstream ifs("WorldMap.txt");
	//boost::archive::text_iarchive ia(ifs);
	//ia >> obNewWorld;
	//ifs.close();
	ros::init(argc, argv, "world_node");
 	ros::NodeHandle n;

	std::ifstream stVtxPath;
	stVtxPath.open("../data_files/MapVertices.txt");

	std::ifstream stEdges;
	stEdges.open("../data_files/MapEdges.txt");

	WorldMap *pWorld = new WorldMap(stVtxPath, stEdges);
	pWorld->ComputeAllPairsShortestPath();

	// std::random_device rd;
	// std::default_random_engine generator(rd());

	ros::ServiceServer distance_service =n.advertiseService<robot_negotiation::GetDistance::Request, robot_negotiation::GetDistance::Response>("/get_distance", boost::bind(getDistance, _1, _2, pWorld));
	ros::ServiceServer stochastic_distance_service =n.advertiseService<robot_negotiation::GetDistance::Request, robot_negotiation::GetDistance::Response>("/get_distance_stochastic", boost::bind(GetDistanceStochastic, _1, _2, pWorld));
    // ros::ServiceServer service = nh.advertiseService<epsilon::Pose::Request, epsilon::Pose::Response>("/epsilon/get_pose", boost::bind(&PoseServer::compute_Pose, server, _1, _2));

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
