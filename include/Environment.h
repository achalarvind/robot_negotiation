#ifndef ENVIRONMENT_H	   
#define ENVIRONMENT_H	

#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include "ros/ros.h"
#include "robot_negotiation/AddObject.h"
#include "robot_negotiation/RemoveObject.h"
#include "robot_negotiation/Block.h"
#include "robot_negotiation/Location.h"
//#include <boost/thread.hpp>

class Block
{
	private:
		std::string m_str;
		std::string m_str_ID;
		int m_iHeight;
	public:
		Block(std::string str , int m_iHeight , int iD);
		Block(); // Avoid using this constructor

		std::string GetName();
		std::string GetID();
		int GetHeight();
		Block& operator = (const Block &cSource);
};

class Location
{
private:
	int m_iX, m_iY;
public:
	Location(int iX, int iY);
	Location();
	int GetX();
	int GetY();
	double GetDistanceToLocation(Location *pclLoc);
	void SetLocation(int iX, int iY);
};

class BlockStack
{
	private:
		std::vector<Block> m_vecStackBlocks;
	public:
		BlockStack();
		bool AddToStack(Block obBlock , int iCapacity);
		int GetStackSize();
		bool Get_Object_Locations(Block obBlock, Location loc, std::unordered_map<std::string , std::pair<Location, double>> *pvecLocations);
		bool RemoveBlock(Block block);
		BlockStack& operator= (const BlockStack &cSource);
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
		std::vector<std::vector<BlockStack>> m_vecTable;
		int m_iCapacity_Per_Location;		
		bool m_Table_In_Use;
		TableState Is_Loc_In_Bounds(Location obLoc);
		TableState Is_Table_Full_At_Loc(Location obLoc);	
//		boost::mutex m_bmutex;;
		Location *pclPivotPoint;

	public:
		Environment(FILE *pFile);
		Environment(int iTableSize, int m_iCapacity_Per_Location);
		~Environment();

		TableState Add_Element(Block obBlock, Location obLoc);
		TableState Remove_Element(Block obBlock, Location obLoc , bool bTimeDelay = false);
		bool Get_Object_Locations(Block obBlock, std::unordered_map<std::string , std::pair<Location, double>> *);
		double GetNearestObjectLocation(Block obBlock, Location *pclLoc);
		bool Is_Table_In_Use();
		int GetStackHeight(Location obLoc);
        static int GenerateID();
		Environment& operator= (const Environment &cSource);
};

class EnvironmentGeometry
{
	private:
		double m_dDist01, m_dDist02, m_dDist12;
	
	public:
		EnvironmentGeometry(double , double , double );
		EnvironmentGeometry();
		EnvironmentGeometry& operator= (const EnvironmentGeometry &cSource);
		double ReturnTravDistance(int, int);
		static int g_iTotalLocations;
};

#endif