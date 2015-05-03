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
#include "CobotQueueHandler.h" 
    

using namespace std;

const int NCobots = 10;
const int NThreads = 1;

std::string baxter_location;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
 	ros::NodeHandle n("~");
    //create multiple cobots in queue
    std::vector<uint> cobotIds;
    cobotQueue queue;
    for(int i = 0; i < NCobots; i++)
    {
        queue.add_cobot(i);
    }
    //cobotQueue queue(cobotIds,NCobots);

    for(int nThread = 0; nThread < NThreads; nThread++){
        ros::ServiceClient planning_client=n.serviceClient<robot_negotiation::DeSerializeEnvironmentPlan>("/csp_solver/deserialize_environment");
        robot_negotiation::DeSerializeEnvironmentPlan srv_planning;

        //gather first task of each cobot
        for(int i = 0; i < NCobots; i++)
        {
        	robot_negotiation::Action a;
        	a.cobot_id = i;
            a.cobot_type = queue.cobots[i].m_str_type;
        	a.object_type = queue.cobots[i].tasks.task_list[0].object_id;
        	a.deadline = queue.cobots[i].tasks.task_list[0].deadline - queue.cobots[i].tasks.task_list[0].est_time;
        	srv_planning.request.plan.push_back(a);
        }
        srv_planning.request.baxter_location=baxter_location;
        srv_planning.request.start_time=ros::Time::now().toSec()+10.0f;

        if (planning_client.call(srv_planning))
        {
            
            std::vector<std::vector<DeliveryOrderSeq>> plans; 
            std::vector<DeliveryOrderSeq> plan;
            ROS_INFO("Makespan of greedy approach is %f",srv_planning.response.greedy_makespan);

            for(int j = 0; j < srv_planning.response.plans.size(); j++)
            {
                ROS_INFO("Makespan of schedule %d is %f",j,srv_planning.response.plans[j].plan[srv_planning.response.plans[j].plan.size()-1].expected_completion_time);
                plan.clear();
                for(int k = 0; k < srv_planning.response.plans[j].plan.size(); k++)
                {
                    DeliveryOrderSeq a(srv_planning.response.plans[j].plan[k].cobot_id, srv_planning.response.plans[j].plan[k].location, srv_planning.response.plans[j].plan[k].expected_completion_time, 0.0f , "INVALID");
                    plan.push_back(a);
                }       
                plans.push_back(plan);
            }
            ROS_INFO("Starting the voting process");
            
            std::vector<std::vector<double>> voteResult = queue.collect_votes(plans);

            std::vector< std::vector<double> > voteResult_t(voteResult[0].size());

            for (int i = 0; i < voteResult[0].size(); i++)
            {
                std::vector<double> temp(voteResult.size());
                voteResult_t[i] = temp;
            }

            for (int i = 0; i < voteResult.size(); i++)
            {

                for (int j = 0; j < voteResult[0].size(); j++)
                {
                    voteResult_t[j][i] = voteResult[i][j];
                }
            }
            int best_plan=queue.send_best_plan(voteResult_t, voteResult_t.size());
            ROS_INFO("Makespan of negotiated schedule is %f",srv_planning.response.plans[best_plan].plan[srv_planning.response.plans[best_plan].plan.size()-1].expected_completion_time);
            ROS_INFO("Vote Result-Best plan: %d",best_plan);

        }
        else
        {
            ROS_ERROR("Failed to call service /csp_solver/deserialize_environment. No plan generated");
        }   
    }




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
