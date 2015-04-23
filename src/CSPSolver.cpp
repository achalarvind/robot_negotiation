#include "CSPSolver.h"
#include <random>       
#include <chrono>
#include "Constants.h"
#include <iostream>

using namespace std;

std::vector<int> GenerateRandomNumbers(int iMaxValue, int iNumOfValues);
std::vector<int> GenerateSwapCombination(int iCombination, std::vector<int> vecRandomIntegers, int iNumOfCobots);

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

DeliveryOrderSeq::DeliveryOrderSeq(int iCobotNum, std::string strLoc, double dTime , double dDeadLine)
{
	m_iCobotNum = iCobotNum;
	m_strLoc = strLoc;
	m_dExpectedTime = dTime;
	m_dDeadLine = dDeadLine;
}

CompleteSeqInfo::CompleteSeqInfo(int iCobotNum, int iPickUp, int iDropOff, double dDeadLine, double dPickUpTime)
{
	m_iCobotNum = iCobotNum;
	m_iObjectPickUpLocation = iPickUp;
	m_iDropOffLocation = iDropOff;
	m_dDeadline = dDeadLine;
	m_dPickUpTime = dPickUpTime;
}

PickUpOrderSeqInfo::PickUpOrderSeqInfo(int iTableNum, Location stLoc, std::string strObjName, double dTime)
{
	m_iTableNum = iTableNum;
	m_obLoc.SetLocation(stLoc.GetX() , stLoc.GetY());
	m_strObjectName = strObjName;
	m_dPickUpTime = dTime;
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

std::unordered_map<double, std::vector<DeliveryOrderSeq>> CSPSolver::GenerateCobotOrder(std::vector<TaskInfo> vecCobotTasks, std::vector<Environment*> vecptrEnvironments , bool bShuffle)
{
	m_bFeasibility = true;

	m_umap_Candidates.clear();
	m_vecCobotOrder.clear();
	m_vecTasks.clear();
	m_vecEnvironments.clear();

	m_vecTasks = vecCobotTasks;
	
	//Sort by MCV heurisitc - deadlines are used here
	std::sort(m_vecTasks.begin(), m_vecTasks.end(), MCV_Deadline_Heurisitc);

	if (bShuffle)
	{
		std::random_shuffle(m_vecTasks.begin(), m_vecTasks.end());
	}

	//Deep copy of environment variables
	m_vecEnvironments.reserve(vecptrEnvironments.size());
	for (unsigned int iCount = 0; iCount < vecptrEnvironments.size(); iCount++)
	{
		m_vecEnvironments.push_back(*(vecptrEnvironments[iCount]) );
	}

	std::pair<bool, SOLUTION> stSol = TraverseGraph(0, m_c_iStartLocation , m_c_dStartTime);

	if (false == stSol.first)
	{
		m_bFeasibility = false;
		stSol = TraverseGraph(0, m_c_iStartLocation , m_c_dStartTime);
	}

	PopulateSequenceInfo();

	if (true == stSol.first)
	{
		Perform3Swap();
	}

	return m_umap_Candidates;
}

CSPSolver::CSPSolver(std::string strStartLocation, double dStartTime, double dTime_Out_1, double dTime_Out_2, EnvironmentGeometry obGeometry) : m_c_iStartLocation(ReturnDropOfLocation(strStartLocation)), m_c_dStartTime(dStartTime), m_c_Solver_Time_Out_Feasible(dTime_Out_1), m_c_Solver_Time_Out_In_Feasible(dTime_Out_2)
{
	m_obGeometry = obGeometry;
	m_bFeasibility = true;
}

std::pair<bool, SOLUTION> CSPSolver::TraverseGraph(int iCobotIndex, int iCurrLoc, double dCurrentTime)
{
	if (iCobotIndex == m_vecTasks.size())
	{
		return std::make_pair(true, SOLUTION::TIME_IN);
	}

	cout << iCobotIndex << "\n";

	std::pair<bool, SOLUTION> stSol;

	if ((ReturnCurrentTime() > m_c_Solver_Time_Out_Feasible) && m_bFeasibility)
	{
		return std::make_pair(false, SOLUTION::TIME_OUT);
	}

	if ((ReturnCurrentTime() > m_c_Solver_Time_Out_In_Feasible) && !m_bFeasibility)
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

		stSol = SelectDeliveryLocation(iCobotIndex, iEnvNum, vecLocations[iCount].second.second, vecLocations[iCount].second.first);

		if (true == stSol.first)
		{
			m_vecPickUpObjectOrder.push_back(PickUpOrderSeqInfo(iEnvNum, vecLocations[iCount].second.second, obBlock.GetID(), dPickUpTime));
			return stSol;
		}
		else if ((SOLUTION::TIME_OUT == stSol.second) || (SOLUTION::INFEASIBLE == stSol.second))
		{
			return stSol;
		}

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
		
		stSol = TraverseGraph(iCobotIndex + 1, iDropOffLoc, dTimeDelivery);

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
	InsertSequenceIntoCandidatePool(m_vecCobotOrder[iNumOfCobots - 1].m_dExpectedTime, m_vecCobotOrder);

	if (1 == iNumOfCobots)
	{
		return ;
	}

	std::vector<DeliveryOrderSeq> vecNewSequence;

	for (int iCount = 0; iCount < m_vecCobotOrder.size(); iCount++)
	{
		vecNewSequence.push_back(m_vecCobotOrder[iCount]);
	}
	
	std::vector<DeliveryOrderSeq> vecCombinations[6];
		
	unsigned int iMaxIterations;

	if (iNumOfCobots > 15)
	{
		iMaxIterations = 500;
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
		std::vector<int> vecRandomIntegers;
		int iPermutations;

		if (iNumOfCobots >= 3)
		{
			vecRandomIntegers = GenerateRandomNumbers(iNumOfCobots, 3);
			iPermutations = 5;
		}
		else
		{
			vecRandomIntegers = GenerateRandomNumbers(iNumOfCobots, iNumOfCobots);
			iPermutations = 2;
		}

		
		for (int iCombination = 0; iCombination < iPermutations; iCombination++)
		{
			vecCombinations[iCombination].clear();

			for (int iCount = 0; iCount < vecNewSequence.size(); iCount++)
			{
				vecCombinations[iCombination].push_back(vecNewSequence[iCount]);
			}
			
			std::vector<int> vecSwapIndices = GenerateSwapCombination(iCombination, vecRandomIntegers , iNumOfCobots);

			CheckForLocalImprovement(vecRandomIntegers, vecSwapIndices, &vecCombinations[iCombination]);
		}
	}

}

void CSPSolver::PopulateSequenceInfo()
{
	int iCobotNum, iPickUp, iDropOff; 
	double dDeadLine , dPickUpTime;

	for (unsigned int iCount = 0; iCount < m_vecCobotOrder.size(); iCount++)
	{
		iCobotNum = m_vecCobotOrder[iCount].m_iCobotNum;
		iDropOff = ReturnDropOfLocation(m_vecCobotOrder[iCount].m_strLoc);
		dDeadLine = m_vecCobotOrder[iCount].m_dDeadLine;

		iPickUp = m_vecPickUpObjectOrder[iCount].m_iTableNum;
		dPickUpTime = m_vecPickUpObjectOrder[iCount].m_dPickUpTime;

		m_umapCompleteSeqInfo.insert(std::make_pair(iCobotNum, CompleteSeqInfo(iCobotNum, iPickUp, iDropOff, dDeadLine, dPickUpTime)));
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

	for (unsigned int iCount = 0; iCount < vecDelivery.size(); iCount++)
	{
		iCurrCobot = vecDelivery[iCount].m_iCobotNum;
		std::unordered_map<int, CompleteSeqInfo>::iterator it = m_umapCompleteSeqInfo.find(iCurrCobot);

		iPickUpLocation = it->second.m_iObjectPickUpLocation;
		iDeliveryLocation = it->second.m_iDropOffLocation;

		dTime = dTime + (it->second.m_dPickUpTime) + m_obGeometry.ReturnTravDistance(iBaxterStartLocation, iPickUpLocation) + m_obGeometry.ReturnTravDistance(iPickUpLocation, iDeliveryLocation);

		if ((dTime > it->second.m_dDeadline) && m_bFeasibility)
		{
			return;
		}

		vecDelivery[iCount].m_dExpectedTime = dTime;  // Updates the delivery time of this sequence

		iBaxterStartLocation = iDeliveryLocation;
	}

	InsertSequenceIntoCandidatePool(dTime, vecDelivery);
}

void CSPSolver::InsertSequenceIntoCandidatePool(double dTime, std::vector<DeliveryOrderSeq> vecDelivery)
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
		m_umap_Candidates.insert(std::make_pair(dTime , vecDelivery));
	}
	else if (dTime < dLargestKey)
	{
		m_umap_Candidates.erase(dLargestKey);
		m_umap_Candidates.insert(std::make_pair(dTime, vecDelivery));
	}
}

double CSPSolver::ReturnCurrentTime()
{
	return 0;
}

std::vector<int> GenerateSwapCombination(int iCombination, std::vector<int> vecRandomIntegers, int iNumOfCobots)
{
	std::vector<int> vecSwapIndices;

	for (unsigned int iCount = 0; iCount < vecRandomIntegers.size(); iCount++)
	{
		vecSwapIndices.push_back(-1);
	}

	if (2 == iNumOfCobots)
	{
		if (0 == iCombination)
		{
			vecSwapIndices[0] = vecRandomIntegers[0];   vecSwapIndices[1] = vecRandomIntegers[1];
		}
		else if (1 == iCombination)
		{
			vecSwapIndices[0] = vecRandomIntegers[1];   vecSwapIndices[1] = vecRandomIntegers[0];
		}

		return vecSwapIndices;
	}

	if (0 == iCombination)
	{
		vecSwapIndices[0] = vecRandomIntegers[0];   vecSwapIndices[1] = vecRandomIntegers[2];  vecSwapIndices[2] = vecRandomIntegers[1];
	}
	else if (1 == iCombination)
	{
		vecSwapIndices[0] = vecRandomIntegers[1];   vecSwapIndices[1] = vecRandomIntegers[0];  vecSwapIndices[2] = vecRandomIntegers[2];
	}
	else if (2 == iCombination)
	{
		vecSwapIndices[0] = vecRandomIntegers[1];   vecSwapIndices[1] = vecRandomIntegers[2];  vecSwapIndices[2] = vecRandomIntegers[0];
	}
	else if (3 == iCombination)
	{
		vecSwapIndices[0] = vecRandomIntegers[2];   vecSwapIndices[1] = vecRandomIntegers[1];  vecSwapIndices[2] = vecRandomIntegers[0];
	}
	else
	{
		vecSwapIndices[0] = vecRandomIntegers[2];   vecSwapIndices[1] = vecRandomIntegers[0];  vecSwapIndices[2] = vecRandomIntegers[1];
	}

	return vecSwapIndices;
}

std::vector<int> GenerateRandomNumbers(int iMaxValue, int iNumOfValues)
{
	std::vector<int> vecRandom;

	while (1)
	{
		vecRandom.clear();

		for (int iCount = 0; iCount < iNumOfValues; iCount++)
		{
			vecRandom.push_back(rand() % iMaxValue);
		}

		if (vecRandom.end() == std::unique(vecRandom.begin(), vecRandom.end()))
		{
			break;
		}
	}

	return vecRandom;
}