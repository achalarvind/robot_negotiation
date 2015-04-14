#ifndef ENVIRONMENT_H	   
#define ENVIRONMENT_H	

#include <string>
#include <vector>
#include <algorithm>
#include <boost/thread.hpp>

class cBaxter;
class cHuman;

class Block
{
	private:
		std::string m_str;
	public:
		Block(std::string str);
		std::string GetName();
};

class Location
{
private:
	int m_iX, m_iY;
public:
	Location(int iX, int iY);
	int GetX();
	int GetY();
};

class BlockStack
{
	private:
		std::vector<Block> m_vecStackBlocks;
	public:
		bool AddToStack(Block obBlock , int iCapacity);
		int GetStackSize();
		bool Get_Object_Locations(Block obBlock, Location loc, std::vector<std::pair<Location, int>> *pvecLocations);
		bool RemoveBlock(Block block);
};

enum class Action
{ 
	PICK_UP, DROP_OFF	 
};

enum class TableState
{
	NOT_FULL_AT_LOCATION , FULL_AT_LOCATION, LOC_IN_BOUNDS, LOC_NOT_IN_BOUNDS, TABLE_FULL, OBJECT_ADDED , OBJECT_REMOVED , OBJECT_NOT_REMOVED
};

class Plan
{
	public:
		typedef std::tuple<Block, Location, Action> ActionSeq;
		std::vector<ActionSeq> m_vecPlan;	
};



class Environment
{
	private:
		int m_iTableSize;         // m_iTableSize * m_iTableSize
		std::vector<std::vector<BlockStack*>> m_vecTable;
		int m_iCapacity_Per_Location;		
		bool m_Table_In_Use;
		TableState Is_Loc_In_Bounds(Location obLoc);
		TableState Is_Table_Full_At_Loc(Location obLoc);	
		boost::mutex m_bmutex;;

	public:
		Environment(FILE *pFile);
		Environment(int iTableSize, int m_iCapacity_Per_Location);
		~Environment();

		TableState Add_Element(Block obBlock, Location obLoc);
		TableState Remove_Element(Block obBlock, Location obLoc);
		bool Get_Object_Locations(Block obBlock, std::vector<std::pair<Location , int>> * );
		bool Is_Table_In_Use();
		int GetStackHeight(Location obLoc);

		friend TableState cBaxter::Add_Element(Environment*, Block obBlock, Location obLoc);
		friend TableState cBaxter::Remove_Element(Environment*, Block obBlock, Location obLoc);

		friend TableState cHuman::Add_Element(Environment*, Block obBlock, Location obLoc);
		friend TableState cHuman::Remove_Element(Environment*, Block obBlock, Location obLoc);
};

class cBaxter
{
public:
	TableState Add_Element(Environment*, Block obBlock, Location obLoc);
	TableState Remove_Element(Environment*, Block obBlock, Location obLoc);
};

class cHuman
{
public:
	TableState Add_Element(Environment*, Block obBlock, Location obLoc);
	TableState Remove_Element(Environment*, Block obBlock, Location obLoc);
};


#endif