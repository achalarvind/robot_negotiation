#include "CSPSolver.h"

bool deserializeEnvironment(robot_negotiation::DeSerializeEnvironment::Request  &req,
	robot_negotiation::DeSerializeEnvironment::Response &res,
	std::vector<Environment> *pEnv, std::string *file_names){
	for(int i=0;i<pEnv->size();i++){
		Environment obTable;
		std::ifstream ifs(file_names[i].c_str());
		boost::archive::text_iarchive ia(ifs);
		ia >> obTable;
		pEnv->push_back(obTable);
		ifs.close();
	}
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "csp_solver");
	ros::NodeHandle n("~");
	ROS_INFO("Environment ready");

	std::vector<Environment> pEnv;
	std::string filenames[]={"table0.env","table1.env","table2.env"};

	ros::ServiceServer deserialize_environment = n.advertiseService<robot_negotiation::DeSerializeEnvironment::Request, robot_negotiation::DeSerializeEnvironment::Response>("deserialize_environment", boost::bind(deserializeEnvironment, _1, _2, &pEnv, filenames));

	ros::spin();
}