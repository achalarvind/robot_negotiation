#include "Environment.h"
#include "Constants.h"
#include <chrono>

int EnvironmentGeometry::g_iTotalLocations = 3;

Block::Block(std::string str, int iHeight , int ID)
{
	m_str = str;
	m_iHeight = iHeight;
	m_str_ID = str + std::to_string(ID);
}

Block::Block()
{
	m_str = NAMELESS_BLOCK;
	m_iHeight = -1;
	m_str_ID = NAMELESS_BLOCK;
}

std::string Block::GetName()
{
	return m_str;
}

std::string Block::GetID()
{
	return m_str_ID;
}

int Block::GetHeight()
{
	return m_iHeight;
}

Block& Block::operator = (const Block &cSource)
{
	m_iHeight = cSource.m_iHeight;
	m_str = cSource.m_str;
	m_str_ID = cSource.m_str_ID;

	return *this;
}

Location::Location(int iX, int iY)
{
	m_iX = iX;
	m_iY = iY;
}

Location::Location()
{
	m_iX = -1;
	m_iY = -1;
}

double Location::GetDistanceToLocation(Location *pclLoc)
{
	return sqrtf (1.0 * (((m_iX - pclLoc->GetX()) * (m_iX - pclLoc->GetX())) + ((m_iY - pclLoc->GetY()) * (m_iY - pclLoc->GetY()))) );
}

int Location::GetX()
{
	return m_iX;
}

int Location::GetY()
{
	return m_iY;
}

void Location::SetLocation(int iX, int iY)
{
	m_iX = iX;
	m_iY = iY;
}

BlockStack::BlockStack()
{

}

bool BlockStack::AddToStack(Block obBlock, int iCapacity)
{
	if (iCapacity == m_vecStackBlocks.size())
	{
		return false;
	}

	m_vecStackBlocks.push_back(obBlock);
	return true;
}

int BlockStack::GetStackSize()
{
	return m_vecStackBlocks.size();
}

bool BlockStack::Get_Object_Locations(Block obBlock, Location loc, std::unordered_map<std::string , std::pair<Location, double>> *pvecLocations)
{
	bool bExists = false;

	const int c_iStackSize = m_vecStackBlocks.size();
	double dHeight = 0 , dCOM;

	for (int iCount = 0; iCount < c_iStackSize; iCount++)
	{
		dHeight = dHeight + m_vecStackBlocks[iCount].GetHeight();

		if (m_vecStackBlocks[iCount].GetName() == obBlock.GetName())
		{
			dCOM = dHeight - (m_vecStackBlocks[iCount].GetHeight() / 2.0);
			//pvecLocations->push_back(std::make_pair(loc, dCOM));
			pvecLocations->insert(std::make_pair(m_vecStackBlocks[iCount].GetID() , std::make_pair(loc, dCOM)));
			bExists = bExists | true;
		}
	}

	return bExists;
}

bool BlockStack::RemoveBlock(Block obBlock)
{
	for (std::vector<Block>::iterator it = m_vecStackBlocks.begin(); it != m_vecStackBlocks.end(); it++)
	{
		if (it->GetName() == obBlock.GetName())
		{
			m_vecStackBlocks.erase(it);
			return true;
		}
	}

	return false;
}

BlockStack& BlockStack::operator = (const BlockStack &cSource)
{
	m_vecStackBlocks = cSource.m_vecStackBlocks;
	return *this;
}

Environment::Environment(FILE *pFile)
{
	;
}

Environment::Environment(int iTableSize, int iCapacity_Per_Location)
{
	std::vector<BlockStack> vecTemp(iTableSize , BlockStack());
	
	for (int iCount = 0; iCount < iTableSize; iCount++)
	{
		m_vecTable.push_back(vecTemp);
	}

	m_iTableSize = iTableSize;
	m_iCapacity_Per_Location = iCapacity_Per_Location;
	m_Table_In_Use = false;

	pclPivotPoint = new Location((int)(iTableSize / 2), 0);
}

Environment::~Environment()
{
	//delete pclPivotPoint;
	//pclPivotPoint = NULL;
}

TableState Environment::Is_Loc_In_Bounds(Location obLoc)
{
	int iX = obLoc.GetX();
	int iY = obLoc.GetY();

	if ((iX < 0) || (iX >= m_iTableSize) || (iY < 0) || (iY >= m_iTableSize))
	{
		return TableState::LOC_NOT_IN_BOUNDS;
	}

	return TableState::LOC_IN_BOUNDS;
}

TableState Environment::Is_Table_Full_At_Loc(Location obLoc)
{
	BlockStack *pvecStck = &(m_vecTable.at(obLoc.GetY()).at(obLoc.GetX()));

	if (pvecStck->GetStackSize() >= m_iCapacity_Per_Location)
	{
		return TableState::FULL_AT_LOCATION;
	}

	return TableState::NOT_FULL_AT_LOCATION;
}

TableState Environment::Add_Element(Block obBlock, Location obLoc)
{
	//boost::lock_guard<boost::mutex> guard(m_bmutex);

	if (TableState::FULL_AT_LOCATION == Is_Table_Full_At_Loc(obLoc))
	{
		return TableState::FULL_AT_LOCATION;
	}

	if (TableState::LOC_NOT_IN_BOUNDS == Is_Loc_In_Bounds(obLoc))
	{
		return TableState::LOC_NOT_IN_BOUNDS;
	}

	BlockStack *pvecStck = &(m_vecTable[obLoc.GetY()][obLoc.GetX()]);
	
	if (NULL == pvecStck)
	{
		pvecStck = new BlockStack();
	}

	bool bAdded = pvecStck->AddToStack(obBlock , m_iCapacity_Per_Location);

	if (!bAdded)
	{
		return TableState::FULL_AT_LOCATION;
	}

	return TableState::OBJECT_ADDED;
}

TableState Environment::Remove_Element(Block obBlock, Location obLoc, bool bTimeDelay)
{
	//boost::lock_guard<boost::mutex> guard(m_bmutex);

	bool bRemoved = m_vecTable[obLoc.GetY()][obLoc.GetX()].RemoveBlock(obBlock);

	if (!bRemoved)
	{
		return TableState::OBJECT_NOT_REMOVED;
	}

	if (bTimeDelay)
	{
		time_t t;
		srand((unsigned)time(&t));

		double dDist = sqrtf(1.0 *( (obLoc.GetX()) ^ 2 + (obLoc.GetY()) ^ 2 ) );
		double dDelayFactor = ( rand() % ((int)(100 * MAX_DELAY_TIME_PROP_DISTANCE)) )/100.0;
		double dTime = ( dDist * (1 + dDelayFactor) )/(BAXTER_PICK_UP_SPEED);

		//Insert Time delay here - Uses dTime

	}

	return TableState::OBJECT_REMOVED;
}

bool Environment::Get_Object_Locations(Block obBlock, std::unordered_map<std::string , std::pair<Location, double>> *pvecObjLocation)
{
	bool bObjectExists = false;

	for (int iY = 0; iY < m_iTableSize; iY++)
	{
		for (int iX = 0; iX < m_iTableSize; iX++)
		{
			bObjectExists = bObjectExists | (m_vecTable[iY][iX]).Get_Object_Locations(obBlock, Location(iX, iY) , pvecObjLocation );
		}
	}

	return bObjectExists;
}

double Environment::GetNearestObjectLocation(Block obBlock , Location *pclLoc)
{
	typedef std::unordered_map<std::string, std::pair<Location, double>> LocationsDistance;
	LocationsDistance umapLocations;

	bool bExists = Get_Object_Locations(obBlock, &(umapLocations));

	if (!bExists)
	{
		return MAX_DIST_VALUE;
	}

	double dNearestDistance = MAX_DIST_VALUE;
	double dDist;

	for (LocationsDistance::iterator it = umapLocations.begin(); it != umapLocations.end(); it++)
	{
		dDist = it->second.first.GetDistanceToLocation(pclPivotPoint);

		if (dDist < dNearestDistance)
		{
			dNearestDistance = dDist;
			pclLoc->SetLocation(it->second.first.GetX() , it->second.first.GetY());
		}
	}

	return dNearestDistance;
}

bool Environment::Is_Table_In_Use()
{
	return m_Table_In_Use;
}

int Environment::GetStackHeight(Location obLoc)
{
	return (m_vecTable[obLoc.GetY()][obLoc.GetX()]).GetStackSize();
}

int Environment::GenerateID()
{
	static int iObjectID = 0;
	return iObjectID++;
}

Environment& Environment::operator= (const Environment &cSource)
{
	m_vecTable = cSource.m_vecTable;
	return *this;
}

EnvironmentGeometry::EnvironmentGeometry(double dDist01, double dDist02, double dDist12)
{
	m_dDist01 = dDist01;
	m_dDist02 = dDist02;
	m_dDist12 = dDist12;
}

EnvironmentGeometry::EnvironmentGeometry()
{
	m_dDist01 = -1.0;
	m_dDist02 = -1.0;
	m_dDist12 = -1.0;
}

double EnvironmentGeometry::ReturnTravDistance(int iLoc1 , int iLoc2)
{
	if (iLoc1 == iLoc2)
	{
		return 0;
	}

	if (iLoc2 < iLoc1)
	{
		std::swap(iLoc1 , iLoc2);
	}

	if ((0 == iLoc1) && (1 == iLoc2))
	{
		return m_dDist01;
	}
	else if ((0 == iLoc1) && (2 == iLoc2))
	{
		return m_dDist02;
	}

	return m_dDist12;
}

EnvironmentGeometry& EnvironmentGeometry::operator= (const EnvironmentGeometry &cSource)
{
	m_dDist01 = cSource.m_dDist01;
	m_dDist02 = cSource.m_dDist02;
	m_dDist12 = cSource.m_dDist12;

	return *this;
}

bool addObject(robot_negotiation::AddObject::Request  &req,
         robot_negotiation::AddObject::Response &res,
         std::vector<Environment*> pEnv)
{
	Block b(req.block.object_type,req.block.height,req.block.object_id);
	Location l(req.location.x,req.location.y);
	pEnv[req.table]->Add_Element(b,l);
	return true;
}

bool removeObject(robot_negotiation::RemoveObject::Request  &req,
         robot_negotiation::RemoveObject::Response &res,
         std::vector<Environment*> pEnv)
{
	Block b(req.block.object_type,req.block.height,req.block.object_id);
	Location l;
	pEnv[req.table]->GetNearestObjectLocation( b , &l);
	pEnv[req.table]->Remove_Element(b,l,true);
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

	ros::init(argc, argv, "~environment");	
	ros::NodeHandle n("~");
	ROS_INFO("Environment ready");

	ros::ServiceServer add_object =n.advertiseService<robot_negotiation::AddObject::Request, robot_negotiation::AddObject::Response>("add_object", boost::bind(addObject, _1, _2, pEnv));
	ros::ServiceServer remove_object =n.advertiseService<robot_negotiation::RemoveObject::Request, robot_negotiation::RemoveObject::Response>("remove_object", boost::bind(removeObject, _1, _2, pEnv));

	ros::spin();


	return 0;
}

void PopulateEnvironmentObjects(Environment &obTable1, Environment &obTable2, Environment &obTable3)
{
	for (int iCount = 0; iCount < 20; iCount++)
	{
		if (iCount % 4 == 0)
		{
			Block obBlock("A" , 10 , Environment::GenerateID());
			obTable1.Add_Element(obBlock , Location((iCount % 10) , (int)(iCount/10)));
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