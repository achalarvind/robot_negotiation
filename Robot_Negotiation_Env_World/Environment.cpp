#include "Environment.h"

Block::Block(std::string str)
{
	m_str = str;
}

std::string Block::GetName()
{
	return m_str;
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

bool BlockStack::Get_Object_Locations(Block obBlock, Location loc, std::vector<std::pair<Location, int>> *pvecLocations)
{
	bool bExists = false;

	const int c_iStackSize = m_vecStackBlocks.size();

	for (int iCount = 0; iCount < c_iStackSize; iCount++)
	{
		if (m_vecStackBlocks[iCount].GetName() == obBlock.GetName())
		{
			pvecLocations->push_back(std::make_pair( loc , c_iStackSize - iCount - 1) );
			bExists = bExists | true;
		}
	}

	return bExists;
}

bool BlockStack::RemoveBlock(Block obBlock)
{
	if (m_vecStackBlocks[m_vecStackBlocks.size() - 1].GetName() != obBlock.GetName())
	{
		return false;
	}

	m_vecStackBlocks.pop_back();
	return true;
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

TableState Environment::Add_Element(Block obBlock, Location obLoc)
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

TableState cBaxter::Add_Element(Environment* pclEnv, Block obBlock, Location obLoc)
{
	return pclEnv->Add_Element(obBlock , obLoc);
}

TableState cHuman::Add_Element(Environment* pclEnv, Block obBlock, Location obLoc)
{
	return pclEnv->Add_Element(obBlock, obLoc);
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

bool Environment::Get_Object_Locations(Block obBlock, std::vector<std::pair<Location, int>> *pvecObjLocation)
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