#include "Environment.h"

bool serializeEnvironment(robot_negotiation::SerializeEnvironment::Request  &req,
	robot_negotiation::SerializeEnvironment::Response &res,
	std::vector<Environment*> pEnv, std::string *file_names){
	for(int i=0;i<pEnv.size();i++){
		std::ofstream ofs(file_names[i].c_str(), std::ofstream::out);
		boost::archive::text_oarchive oa(ofs);
		oa & *(pEnv[i]);
		ofs.close();
	}
	return true;
}

bool addObject(robot_negotiation::AddObject::Request  &req,
	robot_negotiation::AddObject::Response &res,
	std::vector<Environment*> pEnv)
{
	Block b(req.block.object_type, req.block.height, req.block.object_id);
	Location l(req.location.x, req.location.y);
	pEnv[req.table]->Add_Element(b, l);
	return true;
}

bool removeObject(robot_negotiation::RemoveObject::Request  &req,
	robot_negotiation::RemoveObject::Response &res,
	std::vector<Environment*> pEnv)
{
	Block b(req.block.object_type, req.block.height, req.block.object_id);
	Location l;
	pEnv[req.table]->GetNearestObjectLocation(b, &l);
	pEnv[req.table]->Remove_Element(b, l, true);
	return true;
}

void PopulateEnvironmentObjects(Environment &obTable1, Environment &obTable2, Environment &obTable3);

int main(int argc, char** argv){

	Environment obTable1(10, 100);
	Environment obTable2(10, 100);
	Environment obTable3(10, 100);

	PopulateEnvironmentObjects(obTable1, obTable2, obTable3);

	std::vector<Environment*> pEnv;

	pEnv.push_back(&obTable1);
	pEnv.push_back(&obTable2);
	pEnv.push_back(&obTable3);

	EnvironmentGeometry obGeometry(20, 50, 30);

	ros::init(argc, argv, "environment");
	ros::NodeHandle n("~");
	ROS_INFO("Environment ready");

	std::string filenames[]={"table0.env","table1.env","table2.env"};

	ros::ServiceServer add_object = n.advertiseService<robot_negotiation::AddObject::Request, robot_negotiation::AddObject::Response>("add_object", boost::bind(addObject, _1, _2, pEnv));
	ros::ServiceServer remove_object = n.advertiseService<robot_negotiation::RemoveObject::Request, robot_negotiation::RemoveObject::Response>("remove_object", boost::bind(removeObject, _1, _2, pEnv));
	ros::ServiceServer serialize_environment = n.advertiseService<robot_negotiation::SerializeEnvironment::Request, robot_negotiation::SerializeEnvironment::Response>("serialize_environment", boost::bind(serializeEnvironment, _1, _2, pEnv, filenames));

	ros::spin();


	return 0;
}

void PopulateEnvironmentObjects(Environment &obTable1, Environment &obTable2, Environment &obTable3)
{
	for (int iCount = 0; iCount < 20; iCount++)
	{
		if (iCount % 4 == 0)
		{
			Block obBlock("A", 10, Environment::GenerateID());
			obTable1.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 1)
		{
			Block obBlock("B", 10, Environment::GenerateID());
			obTable1.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 2)
		{
			Block obBlock("C", 10, Environment::GenerateID());
			obTable1.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else
		{
			Block obBlock("D", 10, Environment::GenerateID());
			obTable1.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
	}

	for (int iCount = 0; iCount < 20; iCount++)
	{
		if (iCount % 4 == 0)
		{
			Block obBlock("B", 10, Environment::GenerateID());
			obTable2.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 1)
		{
			Block obBlock("C", 10, Environment::GenerateID());
			obTable2.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 2)
		{
			Block obBlock("D", 10, Environment::GenerateID());
			obTable2.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else
		{
			Block obBlock("A", 10, Environment::GenerateID());
			obTable2.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
	}

	for (int iCount = 0; iCount < 20; iCount++)
	{
		if (iCount % 4 == 0)
		{
			Block obBlock("C", 10, Environment::GenerateID());
			obTable3.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 1)
		{
			Block obBlock("D", 10, Environment::GenerateID());
			obTable3.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else if (iCount % 4 == 2)
		{
			Block obBlock("A", 10, Environment::GenerateID());
			obTable3.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
		else
		{
			Block obBlock("B", 10, Environment::GenerateID());
			obTable3.Add_Element(obBlock, Location((iCount % 10), (int)(iCount / 10)));
		}
	}
}