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

	for(int i=0;i<pEnv.size();i++){
		Environment *obTable = new Environment();
		std::ifstream ifs(file_names[i].c_str());
		boost::archive::text_iarchive ia(ifs);
		ia >> *obTable;
		pEnv.push_back(obTable);
		ifs.close();
	}
	EnvironmentGeometry obGeometry(20, 50, 30);
	CSPSolver obSolver(req.baxter_location, req.start_time, req.start_time+30, pEnv, obGeometry);
	std::vector<TaskInfo> vecTasks;

	for(int iCount = 0; iCount < req.plan.size() ; iCount++)
	{
		Block obBlock(req.plan[iCount].object_type, 10, rand()%1000);
		vecTasks.push_back(TaskInfo(req.plan[iCount].deadline, obBlock, req.plan[iCount].cobot_id));
	}	
	std::unordered_map<double, std::vector<DeliveryOrderSeq>> umapResults= obSolver.GenerateCobotOrder(vecTasks);

	robot_negotiation::Plan plan;

	for(std::unordered_map<double, std::vector<DeliveryOrderSeq>> ::iterator it = umapResults.begin() ; it != umapResults.end() ; it++ )
	{
	    std::vector<DeliveryOrderSeq> vecPlan = it->second;
	    plan.plan.clear();

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

	for(int iCount = 0; iCount < pEnv.size() ; iCount++)
	{
		delete (pEnv.at(iCount));
	}

	return true;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "csp_solver");
	ros::NodeHandle n("~");
	ROS_INFO("Environment ready");


	std::string filenames[]={"table0.env","table1.env","table2.env"};

	ros::ServiceServer deserialize_environment_plan = n.advertiseService<robot_negotiation::DeSerializeEnvironmentPlan::Request, robot_negotiation::DeSerializeEnvironmentPlan::Response>("deserialize_environment", boost::bind(deserializeEnvironment, _1, _2, filenames));
	
	ros::spin();
}