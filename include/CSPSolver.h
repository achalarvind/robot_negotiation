#ifndef CSPSOLVER_H	   
#define CSPSOLVER_H

#include <Environment.h>
#include "ros/ros.h"
#include "robot_negotiation/DeSerializeEnvironment.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <sstream>

#define ROS_CODE

class TaskInfo
{
	public:
		double m_dTaskDeadline;
		Block m_obBlock;
		int m_iCobotNum;

		TaskInfo(double, Block, int);
};

enum class SOLUTION
{
	TIME_IN, TIME_OUT , INFEASIBLE
};

class DeliveryOrderSeq
{
	public:
		int m_iCobotNum;
		std::string m_strLoc;
		double m_dExpectedTime;
		double m_dDeadLine;

		DeliveryOrderSeq(int , std::string , double , double);
		void set(DeliveryOrderSeq);
};

class PickUpOrderSeqInfo
{
	public:
		int m_iTableNum;
		Location m_obLoc;
		std::string m_strObjectName;
		Block m_obBlock;
		double m_dPickUpTime;

		PickUpOrderSeqInfo(int, Location, std::string, double, Block);
};

class CompleteSeqInfo
{
	public:
		int m_iCobotNum;
		int m_iObjectPickUpLocation;
		int m_iDropOffLocation;
		double m_dPickUpTime;
		double m_dDeadline;
		Block m_obBlock;

		CompleteSeqInfo(int iCobotNum, int iPickUp, int iDropOff, double dDeadLine, double dPickUpTime, Block obBlock);
};

class CSPSolver
{
	private:

		const int m_c_iStartLocation;
		const double m_c_dStartTime, m_c_Solver_Time_Out_Feasible;
		bool m_bFeasibility;

		std::vector<DeliveryOrderSeq> m_vecCobotOrder;
		std::vector<PickUpOrderSeqInfo> m_vecPickUpObjectOrder;
		std::unordered_map<int, CompleteSeqInfo> m_umapCompleteSeqInfo;
		std::unordered_map<double, std::vector<DeliveryOrderSeq>> m_umap_Candidates;

		std::vector<bool> m_Assigned_Cobots;
		std::vector<TaskInfo> m_vecTasks;
		std::vector<Environment> c_m_vecEnvironments; // Use it for const
		std::vector<Environment> m_vecEnvironments;
		EnvironmentGeometry m_obGeometry;

		std::string ReturnDropOfLocation(int iLoc);
		int ReturnDropOfLocation(std::string stLoc);

		void Perform3Swap();

		std::pair<bool, SOLUTION> TraverseGraph(int iCobotCount, int iCurrLoc, double dCurrentTime);
		std::pair<bool, SOLUTION> SelectPickUpLocation(Block obBlock, int iCobotCount, int iCurrLoc, double dCurrentTime);
		std::pair<bool, SOLUTION> SelectDeliveryLocation(int iCobotCount, int iCurrLoc, Location obLoc, double dCurrentTime); //Location here is the location at which object was picked up
		std::pair<bool, SOLUTION> SelectNode(int iCurrLoc, double dCurrentTime);

		void CheckForLocalImprovement(std::vector<int> vecRandom, std::vector<int> vecShuffle , std::vector<DeliveryOrderSeq>* pvecDeliverySequence);
		void CheckForLocalImprovementGreedyStartegy(std::vector<DeliveryOrderSeq>* pvecDeliverySequence);

		void PopulateSequenceInfo();
		void InsertSequenceIntoCandidatePool(double, std::vector<DeliveryOrderSeq>);
		double ReturnKeyOfBestCandidate();
		int Return_MCV_Cobot(std::vector<int>* pvecVals);
		bool CheckIfAllVarsInitialized();
		std::tuple<double, int, Location> GetPairWiseShortestCosts(std::vector<Environment>* pvecEnvVars, Block obBlock, int iCurr, int iDest , double dCurrTime);
	
public:
	
	    CSPSolver(std::string strStartLoc, double dStartTime, double dTime_Out_1, std::vector<Environment*> , EnvironmentGeometry obGeometry);

#ifdef ROS_CODE	
		std::unordered_map<double, std::vector<DeliveryOrderSeq>> GenerateCobotOrder(std::vector<TaskInfo>);
#else
		std::unordered_map<double, std::vector<DeliveryOrderSeq>> GenerateCobotOrder(std::vector<TaskInfo>, bool);
#endif

		typedef std::pair<std::pair<int , double>, std::pair<double, Location >> PickUpTime;
		typedef std::pair<int, std::pair<double, int >> DeliveryTime;
		
		//Needs to be changed with ROS time
		double ReturnCurrentTime();
};

#endif