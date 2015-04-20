#include "Environment.h"

Block::Block(std::string str, int iHeight , int ID)
{
	m_str = str;
	m_iHeight = iHeight;
	m_str_ID = str + std::to_string(ID);
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

Location::Location(int iX, int iY)
{
	m_iX = iX;
	m_iY = iY;
}

int Location::GetX()
{
	return m_iX;
}

int Location::GetY()
{
	return m_iY;
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

Environment::Environment(FILE *pFile)
{
	;
}

Environment::Environment(int iTableSize, int iCapacity_Per_Location)
{
	std::vector<std::vector<BlockStack*>> m_vecTable(iTableSize, std::vector<BlockStack*>(iTableSize , NULL));

	m_iTableSize = iTableSize;
	m_iCapacity_Per_Location = iCapacity_Per_Location;
	m_Table_In_Use = false;
}

Environment::~Environment()
{
	for (int iY = 0; iY < m_iTableSize; iY++)
	{
		for (int iX = 0; iX < m_iTableSize; iX++)
		{
			delete m_vecTable[iY][iX];
		}
	}
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
	BlockStack *pvecStck = m_vecTable[obLoc.GetY()][obLoc.GetX()];

	if (pvecStck->GetStackSize() >= m_iCapacity_Per_Location)
	{
		return TableState::FULL_AT_LOCATION;
	}

	return TableState::NOT_FULL_AT_LOCATION;
}

TableState Environment::Add_Element(Block obBlock, Location obLoc, double dHeight)
{
	boost::lock_guard<boost::mutex> guard(m_bmutex);

	if (TableState::FULL_AT_LOCATION == Is_Table_Full_At_Loc(obLoc))
	{
		return TableState::FULL_AT_LOCATION;
	}

	if (TableState::LOC_NOT_IN_BOUNDS == Is_Loc_In_Bounds(obLoc))
	{
		return TableState::LOC_NOT_IN_BOUNDS;
	}

	BlockStack *pvecStck = m_vecTable[obLoc.GetY()][obLoc.GetX()];
	
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

TableState cBaxter::Add_Element(Environment* pclEnv, Block obBlock, Location obLoc, double dHeight)
{
	return pclEnv->Add_Element(obBlock, obLoc, dHeight);
}

TableState cHuman::Add_Element(Environment* pclEnv, Block obBlock, Location obLoc, double dHeight)
{
	return pclEnv->Add_Element(obBlock, obLoc, dHeight);
}


TableState Environment::Remove_Element(Block obBlock, Location obLoc)
{
	boost::lock_guard<boost::mutex> guard(m_bmutex);

	bool bRemoved = m_vecTable[obLoc.GetY()][obLoc.GetX()]->RemoveBlock(obBlock);

	if (!bRemoved)
	{
		return TableState::OBJECT_NOT_REMOVED;
	}

	return TableState::OBJECT_REMOVED;
}

TableState cBaxter::Remove_Element(Environment* pclEnv, Block obBlock, Location obLoc)
{
	return pclEnv->Remove_Element(obBlock, obLoc);
}

TableState cHuman::Remove_Element(Environment* pclEnv, Block obBlock, Location obLoc)
{
	return pclEnv->Remove_Element(obBlock, obLoc);
}

bool Environment::Get_Object_Locations(Block obBlock, std::unordered_map<std::string , std::pair<Location, double>> *pvecObjLocation)
{
	bool bObjectExists = false;

	for (int iY = 0; iY < m_iTableSize; iY++)
	{
		for (int iX = 0; iX < m_iTableSize; iX++)
		{
			bObjectExists = bObjectExists | m_vecTable[iY][iX]->Get_Object_Locations(obBlock, Location(iX, iY) , pvecObjLocation );
		}
	}

	return bObjectExists;
}

bool Environment::Is_Table_In_Use()
{
	return m_Table_In_Use;
}

int Environment::GetStackHeight(Location obLoc)
{
	return m_vecTable[obLoc.GetY()][obLoc.GetX()]->GetStackSize();
}

int Environment::GenerateID()
{
	static int iObjectID = 0;
	return iObjectID++;
}