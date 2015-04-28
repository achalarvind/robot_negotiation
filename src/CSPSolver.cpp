#include "CSPSolver.h"
#include <random>       
#include <chrono>
#include "Constants.h"
#include <iostream>

#define LARGE_TREE   


using namespace std;

void GenerateRandomNumbers(int iMaxValue, int iNumOfValues, std::vector<int>* pvecRandom);
void GenerateSwapCombination(int iCombination, std::vector<int>* pvecRandomIntegers, int iNumOfCobots, std::vector<int>* pvecSwapIndices);

bool Time_To_Pick_At_Location_Sort(CSPSolver::PickUpTime obNode1, CSPSolver::PickUpTime obNode2)
{
	return (obNode1.second.first < obNode2.second.first);
}

bool Time_To_Deliver_At_Location_Sort(CSPSolver::DeliveryTime obNode1, CSPSolver::DeliveryTime obNode2)
{
	return (obNode1.second.first < obNode2.second.first);
}

bool MCV_Deadline_Heurisitc(TaskInfo obTask1, TaskInfo obTask2)
{
	return (obTask1.m_dTaskDeadline < obTask2.m_dTaskDeadline);
}

TaskInfo::TaskInfo(double dEndTime, Block obBlock, int iCobotNum)
{
	m_dTaskDeadline = dEndTime;
	m_obBlock =obBlock;
	m_iCobotNum = iCobotNum;
}

DeliveryOrderSeq::DeliveryOrderSeq(int iCobotNum, std::string strLoc, double dTime, double dDeadLine)
{
	m_iCobotNum = iCobotNum;
	m_strLoc = strLoc;
	m_dExpectedTime = dTime;
	m_dDeadLine = dDeadLine;
}

void DeliveryOrderSeq::set(DeliveryOrderSeq a)
{
	m_iCobotNum = a.m_iCobotNum;
	m_strLoc = a.m_strLoc;
	m_dExpectedTime = a.m_dExpectedTime ;
	m_dDeadLine = a.m_dDeadLine ;	
}


CompleteSeqInfo::CompleteSeqInfo(int iCobotNum, int iPickUp, int iDropOff, double dDeadLine, double dPickUpTime, Block obBlock)
{
	m_iCobotNum = iCobotNum;
	m_iObjectPickUpLocation = iPickUp;
	m_iDropOffLocation = iDropOff;
	m_dDeadline = dDeadLine;
	m_dPickUpTime = dPickUpTime;
	m_obBlock = obBlock;
}

PickUpOrderSeqInfo::PickUpOrderSeqInfo(int iTableNum, Location stLoc, std::string strObjName, double dTime, Block obBlock)
{
	m_iTableNum = iTableNum;
	m_obLoc.SetLocation(stLoc.GetX() , stLoc.GetY());
	m_strObjectName = strObjName;
	m_dPickUpTime = dTime;
	m_obBlock = obBlock;
}

std::string CSPSolver::ReturnDropOfLocation(int iLoc)
{
	if (0 == iLoc){return "A";}
	else if (1 == iLoc){return "B";}
	else{return "C";}
}

int CSPSolver::ReturnDropOfLocation(std::string stLoc)
{
	if ("A" == stLoc){return 0;}
	else if ("B" == stLoc){return 1;}
	else{return 2;}
}

double CSPSolver::ReturnTimeToPlaceObject(std::string stCobotType, int iTableNum)
{
	if (0 == iTableNum)
	{
		if ("SMALL" == stCobotType) { return  TIME_TO_PLACE_OBJECT_TABLE_0_SMALL;}
		else if ("MEDIUM" == stCobotType) {	return TIME_TO_PLACE_OBJECT_TABLE_0_MEDIUM;}
		else { return TIME_TO_PLACE_OBJECT_TABLE_0_LARGE; }
	}
	else if (1 == iTableNum)
	{
		if ("SMALL" == stCobotType) { return  TIME_TO_PLACE_OBJECT_TABLE_1_SMALL; }
		else if ("MEDIUM" == stCobotType) { return TIME_TO_PLACE_OBJECT_TABLE_1_MEDIUM; }
		else { return TIME_TO_PLACE_OBJECT_TABLE_1_LARGE; }
	}
	else
	{
		if ("SMALL" == stCobotType) { return  TIME_TO_PLACE_OBJECT_TABLE_2_SMALL; }
		else if ("MEDIUM" == stCobotType) { return TIME_TO_PLACE_OBJECT_TABLE_2_MEDIUM; }
		else { return TIME_TO_PLACE_OBJECT_TABLE_2_LARGE; }
	}
}

#ifdef ROS_CODE
std::unordered_map<double, std::vector<DeliveryOrderSeq>> CSPSolver::GenerateCobotOrder(std::vector<TaskInfo> vecCobotTasks)
#else
std::unordered_map<double, std::vector<DeliveryOrderSeq>> CSPSolver::GenerateCobotOrder(std::vector<TaskInfo> vecCobotTasks, bool bShuffle)
#endif
{
	m_bFeasibility = true;

	m_umap_Candidates.clear();
	m_vecCobotOrder.clear();
	m_vecTasks.clear();
	m_vecEnvironments.clear();

	m_d_Greedy_MakeSpan = -1;
	m_vecTasks = vecCobotTasks;
	
	m_vecEnvironments = c_m_vecEnvironments;

	//Initialize variables
	for (unsigned int iCount = 0; iCount < m_vecTasks.size(); iCount++)
	{
		m_Assigned_Cobots.push_back(false);
	}
	
	//Sort by MCV heurisitc - deadlines are used here

#ifdef ROS_CODE

#else
	std::sort(m_vecTasks.begin(), m_vecTasks.end(), MCV_Deadline_Heurisitc);
	if (bShuffle)
	{
		std::random_shuffle(m_vecTasks.begin(), m_vecTasks.end());
	}
#endif

	
#ifdef LARGE_TREE
	std::pair<bool, SOLUTION> stSol = SelectNode(m_c_iStartLocation, m_c_dStartTime);
#else
	std::pair<bool, SOLUTION> stSol = TraverseGraph(0 , m_c_iStartLocation, m_c_dStartTime);
#endif

	if (false == stSol.first)
	{
		m_bFeasibility = false;

		//Initialize variables
		for (unsigned int iCount = 0; iCount < m_vecTasks.size(); iCount++)
		{
			m_Assigned_Cobots[iCount] = false;
		}

#ifdef LARGE_TREE
		stSol = SelectNode(m_c_iStartLocation, m_c_dStartTime);
#else
		stSol = TraverseGraph(0, m_c_iStartLocation, m_c_dStartTime);
#endif
	}

	m_d_Greedy_MakeSpan = m_vecCobotOrder[m_vecCobotOrder.size() - 1].m_dExpectedTime;

	PopulateSequenceInfo();

	if (true == stSol.first)
	{
		Perform3Swap();
	}

	return m_umap_Candidates;
}

CSPSolver::CSPSolver(std::string strStartLocation, double dStartTime, double dTime_Out_1, std::vector<Environment*> vecptrEnvironments, EnvironmentGeometry obGeometry) : m_c_iStartLocation(ReturnDropOfLocation(strStartLocation)), m_c_dStartTime(dStartTime), m_c_Solver_Time_Out_Feasible(dTime_Out_1)
{
	m_obGeometry = obGeometry;
	m_bFeasibility = true;

	//Deep copy of environment variables
	c_m_vecEnvironments.reserve(vecptrEnvironments.size());
	for (unsigned int iCount = 0; iCount < vecptrEnvironments.size(); iCount++)
	{
		c_m_vecEnvironments.push_back(*(vecptrEnvironments[iCount]));
	}
}

int CSPSolver::Return_MCV_Cobot(std::vector<int>* pvecVals)
{
	std::vector<int>::iterator it;

	for (unsigned int iCount = 0; iCount < m_Assigned_Cobots.size(); iCount++)
	{
		if (true == m_Assigned_Cobots[iCount]) { continue; }

		it = std::find(pvecVals->begin(), pvecVals->end(), iCount);

		if (it != pvecVals->end()) { continue; }

		return iCount;
	}

	return -1;
}


std::pair<bool, SOLUTION> CSPSolver::SelectNode(int iCurrLoc, double dCurrentTime)
{
	int iCobotIndex;
	std::pair<bool, SOLUTION> stSol;
	std::vector<int> vecPrevtriedValues;

	for (unsigned int iCount = 0; iCount < m_vecTasks.size(); iCount++)
	{
		if (true == CheckIfAllVarsInitialized())
		{
			return std::make_pair(true, SOLUTION::TIME_IN);
		}

		iCobotIndex = Return_MCV_Cobot(&(vecPrevtriedValues));

		if (-1 == iCobotIndex)
		{
			return std::make_pair(false, SOLUTION::TIME_IN);
		}

		if ( (dCurrentTime >= m_vecTasks[iCobotIndex].m_dTaskDeadline) && m_bFeasibility )
		{
			return std::make_pair(false, SOLUTION::TIME_IN);
		}

		//cout << iCobotIndex << "\n";		
		
		vecPrevtriedValues.push_back(iCobotIndex);

		m_Assigned_Cobots[iCobotIndex] = true;

		stSol = TraverseGraph(iCobotIndex, iCurrLoc, dCurrentTime);
		
		if ((true == stSol.first) || (SOLUTION::TIME_OUT == stSol.second) || (SOLUTION::INFEASIBLE == stSol.second))
		{
			return stSol;
		}

		m_Assigned_Cobots[iCobotIndex] = false;
	}

	return std::make_pair(false, SOLUTION::TIME_IN);
}

std::pair<bool, SOLUTION> CSPSolver::TraverseGraph(int iCobotIndex, int iCurrLoc, double dCurrentTime)
{
#ifndef LARGE_TREE
	if(iCobotIndex == m_vecTasks.size())
	{
		return std::make_pair(true, SOLUTION::TIME_IN);
	}
#endif

	std::pair<bool, SOLUTION> stSol;

	if ((ReturnCurrentTime() > m_c_Solver_Time_Out_Feasible) && m_bFeasibility)
	{
		return std::make_pair(false, SOLUTION::TIME_OUT);
	}

	stSol = SelectPickUpLocation(m_vecTasks[iCobotIndex].m_obBlock, iCobotIndex, iCurrLoc, dCurrentTime);
		
	return stSol;
}

std::pair<bool, SOLUTION> CSPSolver::SelectPickUpLocation(Block obBlock, int iCobotIndex, int iCurrLoc, double dCurrentTime)
{
	std::vector<PickUpTime> vecLocations;
	
	double dDistance_At_Table, dTime_At_Table;
	double dDistance_To_Table, dTime_To_Table;
	
	bool bItemAvailable = false;

	for (int iLocation = 0; iLocation < EnvironmentGeometry::g_iTotalLocations; iLocation++)
	{
		Location stLoc;  //Dummy initialization

		dDistance_At_Table = m_vecEnvironments[iLocation].GetNearestObjectLocation(obBlock, &stLoc);

		dDistance_At_Table = dDistance_At_Table * (1 + (MAX_DELAY_TIME_PROP_DISTANCE / 2));          // Adds delay for picking
		
		if (MAX_DIST_VALUE == dDistance_At_Table)
		{
			dTime_At_Table = MAX_DIST_VALUE;
		}
		else
		{
			dTime_At_Table = (dDistance_At_Table / BAXTER_PICK_UP_SPEED);
			bItemAvailable = bItemAvailable | true;
		}

		dDistance_To_Table = m_obGeometry.ReturnTravDistance(iCurrLoc, iLocation);
		dTime_To_Table = (dDistance_To_Table / BAXTER_TRAV_SPEED);
		
		vecLocations.push_back(std::make_pair(std::make_pair(iLocation, dTime_At_Table), std::make_pair(dCurrentTime + dTime_At_Table + dDistance_To_Table, stLoc)));
	}

	if (false == bItemAvailable)
	{
		return std::make_pair(false , SOLUTION::INFEASIBLE);
	}

	std::sort(vecLocations.begin(), vecLocations.end(), Time_To_Pick_At_Location_Sort);

	std::pair<bool, SOLUTION> stSol;

	for (int iCount = 0; iCount < EnvironmentGeometry::g_iTotalLocations; iCount++)
	{
		if (MAX_DIST_VALUE == vecLocations[iCount].second.first)
		{
			continue;
		}

		int iEnvNum = vecLocations[iCount].first.first;
		double dPickUpTime = vecLocations[iCount].first.second;
		
		m_vecEnvironments[iEnvNum].Remove_Element(obBlock, vecLocations[iCount].second.second);

		m_vecPickUpObjectOrder.push_back(PickUpOrderSeqInfo(iEnvNum, vecLocations[iCount].second.second, obBlock.GetID(), dPickUpTime, obBlock));

		stSol = SelectDeliveryLocation(iCobotIndex, iEnvNum, vecLocations[iCount].second.second, vecLocations[iCount].second.first);

		if ((true == stSol.first) || (SOLUTION::TIME_OUT == stSol.second) || (SOLUTION::INFEASIBLE == stSol.second) )
		{
			return stSol;
		}
		
		m_vecPickUpObjectOrder.pop_back();
		m_vecEnvironments[iEnvNum].Add_Element(obBlock, vecLocations[iCount].second.second);
	}

	return std::make_pair(false , SOLUTION::TIME_IN);
}

std::pair<bool, SOLUTION> CSPSolver::SelectDeliveryLocation(int iCobotIndex, int iPickUpLocation, Location obLoc, double dCurrentTime)
{
	double dDistance_To_Table, dTime_To_Table;
	std::vector<DeliveryTime> vecDeliveryLocations;

	for (int iDeliveryLocation = 0; iDeliveryLocation < EnvironmentGeometry::g_iTotalLocations; iDeliveryLocation++)
	{
		dDistance_To_Table = m_obGeometry.ReturnTravDistance(iPickUpLocation, iDeliveryLocation);
		dTime_To_Table = dDistance_To_Table / BAXTER_TRAV_SPEED;
		dTime_To_Table = dTime_To_Table + TIME_TO_PLACE_OBJECT;

		vecDeliveryLocations.push_back(std::make_pair(iPickUpLocation, std::make_pair(dCurrentTime + dTime_To_Table, iDeliveryLocation)));
	}

	std::sort(vecDeliveryLocations.begin(), vecDeliveryLocations.end(), Time_To_Deliver_At_Location_Sort);

	std::pair<bool , SOLUTION> stSol;
	for (int iCount = 0; iCount < EnvironmentGeometry::g_iTotalLocations; iCount++)
	{
		double dDeadline = m_vecTasks[iCobotIndex].m_dTaskDeadline;

		if (m_bFeasibility)
		{		
			if (vecDeliveryLocations[iCount].second.first > dDeadline) 
			{
				continue;
			}
		}

		int iDropOffLoc = vecDeliveryLocations[iCount].second.second;
		std::string stDropOffLoc = ReturnDropOfLocation(iDropOffLoc);
		double dTimeDelivery = vecDeliveryLocations[iCount].second.first;		

		m_vecCobotOrder.push_back(DeliveryOrderSeq(m_vecTasks[iCobotIndex].m_iCobotNum, stDropOffLoc, dTimeDelivery, dDeadline));

#ifdef LARGE_TREE
		stSol = SelectNode(iDropOffLoc, dTimeDelivery);
#else
		stSol = TraverseGraph(iCobotIndex + 1, iDropOffLoc, dTimeDelivery);
#endif

		if ((true == stSol.first) || (SOLUTION::TIME_OUT == stSol.second) || (SOLUTION::INFEASIBLE == stSol.second))
		{
			return stSol;
		}

		m_vecCobotOrder.pop_back();
	}

	return std::make_pair(false, SOLUTION::TIME_IN);
}

void CSPSolver::Perform3Swap()
{
	int iNumOfCobots = m_vecCobotOrder.size();
	std::vector<int> vecSwapIndices{ -1, -1, -1 };
	std::vector<int> vecRandomIntegers{-1 ,-1 ,-1};

	if (m_bFeasibility)
	{
		InsertSequenceIntoCandidatePool(m_vecCobotOrder[iNumOfCobots - 1].m_dExpectedTime, &m_vecCobotOrder);
	}
	else
	{
		double dFeasibleCobots = 0;

		for (unsigned int iCount = 0; iCount < m_vecCobotOrder.size(); iCount++)
		{
			if (m_vecCobotOrder[iCount].m_dExpectedTime <= m_vecCobotOrder[iCount].m_dDeadLine) 
			{
				dFeasibleCobots = dFeasibleCobots + 1.0;
			}
		}

		InsertSequenceIntoCandidatePool( -1 * dFeasibleCobots , &m_vecCobotOrder);
	}

	if (1 == iNumOfCobots)
	{
		return ;
	}

	std::vector<DeliveryOrderSeq> vecNewSequence;

	for (unsigned int iCount = 0; iCount < m_vecCobotOrder.size(); iCount++)
	{
		vecNewSequence.push_back(m_vecCobotOrder[iCount]);
	}
	
	std::vector<DeliveryOrderSeq> vecCombinations[6];
		
	unsigned int iMaxIterations;
	double dBestSol;

	if (iNumOfCobots > 15)
	{
		iMaxIterations = 1000;
	}
	else if (iNumOfCobots == 2)
	{
		iMaxIterations = 2;		
	}
	else
	{
		iMaxIterations = (int)((iNumOfCobots * (iNumOfCobots - 1) * (iNumOfCobots - 2)) / 6);
	}

	for (unsigned int iIterations = 0; iIterations < iMaxIterations; iIterations++)
	{
		int iPermutations;

		if (iNumOfCobots >= 3)
		{
			GenerateRandomNumbers(iNumOfCobots, 3, &vecRandomIntegers);
			iPermutations = 5;
		}
		else
		{
			GenerateRandomNumbers(iNumOfCobots, iNumOfCobots, &vecRandomIntegers);
			iPermutations = 2;
		}

		for (int iCombination = 0; iCombination < iPermutations; iCombination++)
		{
			vecCombinations[iCombination].clear();

			for (unsigned int iCount = 0; iCount < vecNewSequence.size(); iCount++)
			{
				vecCombinations[iCombination].push_back(vecNewSequence[iCount]);
			}
			
			GenerateSwapCombination(iCombination, &vecRandomIntegers, iNumOfCobots, &vecSwapIndices);

			CheckForLocalImprovement(vecRandomIntegers, vecSwapIndices, &vecCombinations[iCombination]);
		}

		dBestSol = ReturnKeyOfBestCandidate();
		vecNewSequence = m_umap_Candidates.find(dBestSol)->second;
	}

}

void CSPSolver::PopulateSequenceInfo()
{
	int iCobotNum, iPickUp, iDropOff; 
	double dDeadLine , dPickUpTime;
	Block obBlock;

	for (unsigned int iCount = 0; iCount < m_vecCobotOrder.size(); iCount++)
	{
		iCobotNum = m_vecCobotOrder[iCount].m_iCobotNum;
		iDropOff = ReturnDropOfLocation(m_vecCobotOrder[iCount].m_strLoc);
		dDeadLine = m_vecCobotOrder[iCount].m_dDeadLine;

		iPickUp = m_vecPickUpObjectOrder[iCount].m_iTableNum;
		dPickUpTime = m_vecPickUpObjectOrder[iCount].m_dPickUpTime;
		obBlock = m_vecPickUpObjectOrder[iCount].m_obBlock;

		m_umapCompleteSeqInfo.insert(std::make_pair(iCobotNum, CompleteSeqInfo(iCobotNum, iPickUp, iDropOff, dDeadLine, dPickUpTime, obBlock)));
	}
}

void CSPSolver::CheckForLocalImprovement(std::vector<int> vecRandom, std::vector<int> vecShuffle, std::vector<DeliveryOrderSeq>* pvecDeliverySequence)
{
	int iBaxterStartLocation = m_c_iStartLocation;
	std::vector<DeliveryOrderSeq> vecDelivery = *pvecDeliverySequence;	

	std::vector<DeliveryOrderSeq> vecOb;

	for (unsigned int iCount = 0; iCount < vecRandom.size(); iCount++)
	{
		vecOb.push_back(vecDelivery[vecRandom[iCount]]);
	}

	for (unsigned int iCount = 0; iCount < vecRandom.size(); iCount++)
	{
		vecDelivery[vecShuffle[iCount]] = vecOb[iCount];
	}

	double dStartTime = m_c_dStartTime;

	int iCurrCobot;
	int iDeliveryLocation;
	int iPickUpLocation;

	double dTime = m_c_dStartTime;
	double dFeassibleCobots = 0;

	// Strategy
	for (unsigned int iCount = 0; iCount < vecDelivery.size(); iCount++)
	{
		iCurrCobot = vecDelivery[iCount].m_iCobotNum;
		std::unordered_map<int, CompleteSeqInfo>::iterator it = m_umapCompleteSeqInfo.find(iCurrCobot);

		iPickUpLocation = it->second.m_iObjectPickUpLocation;
		iDeliveryLocation = it->second.m_iDropOffLocation;

		dTime = dTime + (it->second.m_dPickUpTime) + m_obGeometry.ReturnTravDistance(iBaxterStartLocation, iPickUpLocation) + m_obGeometry.ReturnTravDistance(iPickUpLocation, iDeliveryLocation);

		if (m_bFeasibility)
		{
			if (dTime > it->second.m_dDeadline) 
			{ 
				return CheckForLocalImprovementGreedyStartegy(&vecDelivery);
			}
		}
		else
		{
			if (dTime < it->second.m_dDeadline)
			{
				dFeassibleCobots = dFeassibleCobots + 1.0;
			}
		}

		vecDelivery[iCount].m_dExpectedTime = dTime;  // Updates the delivery time of this sequence

		iBaxterStartLocation = iDeliveryLocation;
	}

	if (m_bFeasibility)
	{
		InsertSequenceIntoCandidatePool(dTime, &vecDelivery);
	}
	else
	{
		InsertSequenceIntoCandidatePool(-1 * dFeassibleCobots, &vecDelivery);
	}

	CheckForLocalImprovementGreedyStartegy(&vecDelivery);
}

void CSPSolver::CheckForLocalImprovementGreedyStartegy(std::vector<DeliveryOrderSeq>* pvecDeliverySequence)
{
	//Greedy strategy
	std::vector<Environment> vecEnv(c_m_vecEnvironments);

	std::vector<DeliveryOrderSeq> vecGreedyDelivery = *pvecDeliverySequence;

	std::tuple<double, int, Location> stResult;

	int iBaxterStartLocation = m_c_iStartLocation;
	int iBestDest;
	double dCurrTime;

	double dFeassibleCobots = 0;

	for (unsigned int iCount = 0; iCount < vecGreedyDelivery.size(); iCount++)
	{
		std::unordered_map<int, CompleteSeqInfo>::iterator it = m_umapCompleteSeqInfo.find(vecGreedyDelivery[iCount].m_iCobotNum);
		std::tuple<double, int, Location> stBestResult = std::make_tuple(MAX_DIST_VALUE, 0, Location()); //Dummy initialization  

		if (0 == iCount) { dCurrTime = m_c_dStartTime; }
		else { dCurrTime = vecGreedyDelivery[iCount - 1].m_dExpectedTime; }

		for (int iStart = 0; iStart < 3; iStart++)
		{
			double dTravCost = m_obGeometry.ReturnTravDistance(iBaxterStartLocation, iStart);

			for (int iDest = 0; iDest < 3; iDest++)
			{
				stResult = GetPairWiseShortestCosts(&vecEnv, it->second.m_obBlock, iStart, iDest, dCurrTime);
				std::get<0>(stResult) = std::get<0>(stResult) +dTravCost;

				if (std::get<0>(stBestResult) > std::get<0>(stResult))
				{
					stBestResult = stResult;
					iBestDest = iDest;
				}
			}
		}

		vecEnv.at(std::get<1>(stBestResult)).Remove_Element(it->second.m_obBlock, std::get<2>(stBestResult));
		
		//Greedy updates many fields
		vecGreedyDelivery[iCount].m_dExpectedTime = std::get<0>(stBestResult);
		iBaxterStartLocation = iBestDest;
		vecGreedyDelivery[iCount].m_strLoc = ReturnDropOfLocation(std::get<1>(stBestResult));
		
		if (m_bFeasibility)
		{
			if (vecGreedyDelivery[iCount].m_dExpectedTime > it->second.m_dDeadline) 
			{ return; }
		}
		else
		{
			if (vecGreedyDelivery[iCount].m_dExpectedTime < it->second.m_dDeadline)
			{
				dFeassibleCobots = dFeassibleCobots + 1.0;
			}
		}
	}

	if (m_bFeasibility)
	{
		InsertSequenceIntoCandidatePool(vecGreedyDelivery[vecGreedyDelivery.size()-1].m_dExpectedTime, &vecGreedyDelivery);		
	}
	else
	{
		InsertSequenceIntoCandidatePool(-1 * dFeassibleCobots, &vecGreedyDelivery);		
	}
}

std::tuple<double , int , Location> CSPSolver::GetPairWiseShortestCosts(std::vector<Environment>* pvecEnvVars, Block obBlock , int iCurr , int iDest , double dCurrTime)
{
	Location stLoc;
	std::tuple<double, int , Location> stResult = std::make_tuple(MAX_DIST_VALUE, -1, stLoc);
	double dDistance_At_Table , dTime_At_Table, dTime, dCurrToTable, dTableToDest;

	for (int iTable = 0; iTable < 3; iTable++)
	{
		dCurrToTable = m_obGeometry.ReturnTravDistance(iCurr, iTable);
		dTime = (dCurrToTable / BAXTER_TRAV_SPEED);

		dDistance_At_Table = pvecEnvVars->at(iTable).GetNearestObjectLocation(obBlock, &stLoc);
		dDistance_At_Table = dDistance_At_Table * (1 + (MAX_DELAY_TIME_PROP_DISTANCE / 2));          // Adds delay for picking

		if (MAX_DIST_VALUE == dDistance_At_Table)
		{
			dTime_At_Table = MAX_DIST_VALUE;
		}
		else
		{
			dTime_At_Table = (dDistance_At_Table / BAXTER_PICK_UP_SPEED);
		}

		dTime = dTime + dTime_At_Table;
		dTime = dTime + TIME_TO_PLACE_OBJECT;

		dTableToDest = m_obGeometry.ReturnTravDistance(iTable, iDest);
		dTime = dTime + (dTableToDest / BAXTER_TRAV_SPEED);		

		if (std::get<0>(stResult) > dTime)
		{
			stResult = std::make_tuple(dTime, iTable, stLoc);
		}
	}

	std::get<0>(stResult) = std::get<0>(stResult) +dCurrTime;

	return stResult;
}

double CSPSolver::ReturnMakeSpanForSchedule(std::vector<DeliveryOrderSeq>* pvecDeliverySequence)
{
	std::vector<Environment> vecEnv(c_m_vecEnvironments);

	std::vector<DeliveryOrderSeq> vecGreedyDelivery = *pvecDeliverySequence;

	std::tuple<double, int, Location> stResult;

	int iBaxterStartLocation = m_c_iStartLocation;
	int iBestDest;
	double dCurrTime;

	double dFeassibleCobots = 0;

	for (unsigned int iCount = 0; iCount < vecGreedyDelivery.size(); iCount++)
	{
		std::unordered_map<int, CompleteSeqInfo>::iterator it = m_umapCompleteSeqInfo.find(vecGreedyDelivery[iCount].m_iCobotNum);
		std::tuple<double, int, Location> stBestResult = std::make_tuple(MAX_DIST_VALUE, 0, Location()); //Dummy initialization  

		if (0 == iCount) { dCurrTime = m_c_dStartTime; }
		else { dCurrTime = vecGreedyDelivery[iCount - 1].m_dExpectedTime; }

		for (int iStart = 0; iStart < 3; iStart++)
		{
			double dTravCost = m_obGeometry.ReturnTravDistance(iBaxterStartLocation, iStart);

			for (int iDest = 0; iDest < 3; iDest++)
			{
				stResult = GetPairWiseShortestCosts(&vecEnv, it->second.m_obBlock, iStart, iDest, dCurrTime);
				std::get<0>(stResult) = std::get<0>(stResult) +dTravCost;

				if (std::get<0>(stBestResult) > std::get<0>(stResult))
				{
					stBestResult = stResult;
					iBestDest = iDest;
				}
			}
		}

		vecEnv.at(std::get<1>(stBestResult)).Remove_Element(it->second.m_obBlock, std::get<2>(stBestResult));

		//Greedy updates many fields
		vecGreedyDelivery[iCount].m_dExpectedTime = std::get<0>(stBestResult);
		iBaxterStartLocation = iBestDest;
		vecGreedyDelivery[iCount].m_strLoc = ReturnDropOfLocation(std::get<1>(stBestResult));		
	}

	return vecGreedyDelivery[vecGreedyDelivery.size() - 1].m_dExpectedTime;
}

void CSPSolver::InsertSequenceIntoCandidatePool(double dTime, std::vector<DeliveryOrderSeq> *pvecDelivery)
{
	std::vector<double> vecKeys;

	for (std::unordered_map<double, std::vector<DeliveryOrderSeq>>::iterator it = m_umap_Candidates.begin(); it != m_umap_Candidates.end(); it++)
	{
		vecKeys.push_back(it->first);
	}

	double dLargestKey;

	if (0 == vecKeys.size())
	{
		dLargestKey = MAX_DIST_VALUE;
	}
	else
	{
		dLargestKey = *max_element(vecKeys.begin(), vecKeys.end());
	}

	int iNumOfCandidates = vecKeys.size();

	if (iNumOfCandidates < 5)
	{
		m_umap_Candidates.insert(std::make_pair(dTime + ((double)(rand() % 100) / 10000), *pvecDelivery));
	}
	else if (dTime < dLargestKey)
	{
		m_umap_Candidates.erase(dLargestKey);
		m_umap_Candidates.insert(std::make_pair(dTime + ((double)(rand() % 100) / 10000), *pvecDelivery));
	}
}

double CSPSolver::ReturnKeyOfBestCandidate()
{
	std::vector<double> vecKeys;
	
	for (std::unordered_map<double, std::vector<DeliveryOrderSeq>>::iterator it = m_umap_Candidates.begin(); it != m_umap_Candidates.end(); it++)
	{
		vecKeys.push_back(it->first);
	}

	double dSmallestKey;

	if (0 == vecKeys.size())
	{
		dSmallestKey = MAX_DIST_VALUE;
	}
	else
	{
		dSmallestKey = *(min_element(vecKeys.begin(), vecKeys.end()));
	}

	bool bRandom = (rand() % 2) > 0 ? false : true ;

	if (bRandom)
	{
		int iRandom = rand() % m_umap_Candidates.size();
		std::unordered_map<double, std::vector<DeliveryOrderSeq>>::iterator it = m_umap_Candidates.begin();
		std::advance(it, iRandom);
		return it->first;
	}

	return dSmallestKey;
}

double CSPSolver::ReturnCurrentTime()
{
	time_t  now = time(0);
	
	double dTime = (double)(now) - m_d_WorldStartTime;
	return dTime;
}

bool CSPSolver::CheckIfAllVarsInitialized()
{
	for (unsigned int iCount = 0; iCount < m_Assigned_Cobots.size(); iCount++)
	{
		if (false == m_Assigned_Cobots[iCount]){ return false; }
	}
	return true;
}

void GenerateSwapCombination(int iCombination, std::vector<int>* pvecRandomIntegers, int iNumOfCobots, std::vector<int>* pvecSwapIndices)
{	
	if (2 == iNumOfCobots)
	{
		if (0 == iCombination)
		{
			pvecSwapIndices->at(0) = pvecRandomIntegers->at(0);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(1);
		}
		else if (1 == iCombination)
		{
			pvecSwapIndices->at(0) = pvecRandomIntegers->at(1);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(0);
		}
		return ;
	}

	if (0 == iCombination)
	{
		pvecSwapIndices->at(0) = pvecRandomIntegers->at(0);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(2);  pvecSwapIndices->at(2) = pvecRandomIntegers->at(1);
	}
	else if (1 == iCombination)
	{
		pvecSwapIndices->at(0) = pvecRandomIntegers->at(1);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(0);  pvecSwapIndices->at(2) = pvecRandomIntegers->at(2);
	}
	else if (2 == iCombination)
	{
		pvecSwapIndices->at(0) = pvecRandomIntegers->at(1);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(2);  pvecSwapIndices->at(2) = pvecRandomIntegers->at(0);
	}
	else if (3 == iCombination)
	{
		pvecSwapIndices->at(0) = pvecRandomIntegers->at(2);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(1);  pvecSwapIndices->at(2) = pvecRandomIntegers->at(0);
	}
	else
	{
		pvecSwapIndices->at(0) = pvecRandomIntegers->at(2);   pvecSwapIndices->at(1) = pvecRandomIntegers->at(0);  pvecSwapIndices->at(2) = pvecRandomIntegers->at(1);
	}

	return;
}

void GenerateRandomNumbers(int iMaxValue, int iNumOfValues, std::vector<int>* pvecRandom)
{
	time_t t;
	srand((unsigned)time(&t));
	
	while (1)
	{
		for (int iCount = 0; iCount < iNumOfValues; iCount++)
		{
			pvecRandom->at(iCount) = (rand() % iMaxValue);
		}

		if ((pvecRandom->at(0) == pvecRandom->at(1)) || (pvecRandom->at(0) == pvecRandom->at(2)) || (pvecRandom->at(2) == pvecRandom->at(1)))
		{
			continue;
		}
		else
		{
			break;
		}
	}	
}