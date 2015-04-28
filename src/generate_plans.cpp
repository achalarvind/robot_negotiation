#include "CSPSolver.h"
#include "robot_negotiation/DeSerializeEnvironmentPlan.h"
#include "robot_negotiation/ResultTask.h"
#include "robot_negotiation/Action.h"
#include "robot_negotiation/Plan.h"
#include "robot_negotiation/TaskVector.h"

bool deserializeEnvironment(robot_negotiation::DeSerializeEnvironmentPlan::Request  &req,
	robot_negotiation::DeSerializeEnvironmentPlan::Response &res,
	std::string *file_names){
	std::vector<Environment*> pEnv;

	Environment obTable1 , obTable2 , obTable3;
	for(int i=0;i<3;i++){

		std::ifstream ifs(file_names[i].c_str());
		boost::archive::text_iarchive ia(ifs);
		
		if(0 == i)
		{
			ia >> obTable1;
			pEnv.push_back(&obTable1);
		}
		else if(1 == i)
		{
			ia >> obTable2;
			pEnv.push_back(&obTable2);
		}
		else
		{
			ia >> obTable3;
			pEnv.push_back(&obTable3);
		}

		ifs.close();
	}

	EnvironmentGeometry obGeometry(20, 50, 30);

	std::vector<TaskInfo> vecTasks;

	for(int iCount = 0; iCount < req.plan.size() ; iCount++)
	{
		Block obBlock(req.plan[iCount].object_type, 10, rand()%1000);
		vecTasks.push_back(TaskInfo(req.plan[iCount].deadline, obBlock, req.plan[iCount].cobot_id));
	}	

	time_t now=time(0);
	CSPSolver obSolver(req.baxter_location, 0, (double)now+30.0f, pEnv, obGeometry);
	obSolver.m_d_WorldStartTime=(double)now;

	std::unordered_map<double, std::vector<DeliveryOrderSeq>> umapResults= obSolver.GenerateCobotOrder(vecTasks);
	res.greedy_makespan=obSolver.m_d_Greedy_MakeSpan;

	robot_negotiation::Plan plan;

	for(std::unordered_map<double, std::vector<DeliveryOrderSeq>> ::iterator it = umapResults.begin() ; it != umapResults.end() ; it++ )
	{
	    std::vector<DeliveryOrderSeq> vecPlan = it->second;
	    plan.plan.clear();
	    // ROS_INFO("plan size %d",<<vecPlan.size());
	    for(int iCount = 0; iCount < vecPlan.size(); iCount++)
	    {
			robot_negotiation::ResultTask task;
			task.cobot_id = vecPlan[iCount].m_iCobotNum;
			task.location = vecPlan[iCount].m_strLoc;
			task.expected_completion_time = vecPlan[iCount].m_dExpectedTime;
			plan.plan.push_back(task);
		}
		res.plans.push_back(plan);
	}

	// for(int iCount = 0; iCount < pEnv.size() ; iCount++)
	// {
	// 	delete (pEnv.at(iCount));
	// }
	ROS_INFO("Plans generated. Terminating");
	return true;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "csp_solver");
	ros::NodeHandle n("~");
	ROS_INFO("CSP Planner ready");


	std::string filenames[]={"/home/kim/Desktop/Grad_AI/src/robot_negotiation/data_files/table0.env","/home/kim/Desktop/Grad_AI/src/robot_negotiation/data_files/table1.env","/home/kim/Desktop/Grad_AI/src/robot_negotiation/data_files/table2.env"};
	//std::string filenames[]={"/usr0/home/aarvind/catkin_ws/src/robot_negotiation/data_files/table0.env","/usr0/home/aarvind/catkin_ws/src/robot_negotiation/data_files/table1.env","/usr0/home/aarvind/catkin_ws/src/robot_negotiation/data_files/table2.env"};

	ros::ServiceServer deserialize_environment_plan = n.advertiseService<robot_negotiation::DeSerializeEnvironmentPlan::Request, robot_negotiation::DeSerializeEnvironmentPlan::Response>("deserialize_environment", boost::bind(deserializeEnvironment, _1, _2, filenames));
	
	ros::spin();
}