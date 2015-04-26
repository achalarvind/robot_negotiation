#include "ros/ros.h"
#include "../include/CobotQueueHandler.h"
#include <algorithm>


cobotQueue::cobotQueue(std::vector<uint> ids, uint maxN)
{
	cobotIds = ids;
	maxNDecisionMakers = maxN;
}

uint cobotQueue::get_queue_size()
{ 
	return cobotIds.size();
}
		
bool cobotQueue::add_cobot(uint id)
{
	cobotIds.push_back(id);
	return true;
}
		
bool cobotQueue::remove_cobot(uint id)
{
	for (std::vector<uint>::iterator it = cobotIds.begin(); it != cobotIds.end(); it++)
	{
		if (*it == id)
		{
			cobotIds.erase(it);
			return true;
		}
	}

	return false;
}

bool cobotQueue::initiate_vote() // Broadcasts a message to all CoBots in Queue to submit vote
{
	return false;
}

std::vector< std::vector<uint> > cobotQueue::collect_votes(uint nPlans) // Listens to incoming messages and collects votes for all Cobots present in Queue (returns matrix of votes (rows are ranks; cols are voter IDs))
{
    std::vector< std::vector<uint> > votesTable;
	return votesTable;
}

bool cobotQueue::send_best_plan(std::vector< std::vector<uint> > votesTable, uint nPlans) // Evaluates votes and send final plan to Baxter
{
	//create an array of scores for every plan 

	////
	double* row_sum;
	row_sum = new double[nPlans];
 	for(int plan = 0; plan < votesTable.size(); plan++)
 	{
		row_sum[plan] = 0;
 		for(int voter = 0; voter < votesTable[plan].size(); voter++)
 		{
 			row_sum[plan] = row_sum[plan] + votesTable[plan][voter];
 		}
		row_sum[plan] = row_sum[plan]/votesTable[plan].size();
		std::cout<<(boost::lexical_cast<std::string>(row_sum[plan]))<<std::endl;
 	} 
	
	return false;
}

int main(){
 	std::vector< std::vector<uint> > votesTable {{10,9,8,7,6},{10,9,8,7,6},{1,2,3,4}};
 	//votesTable[0] = {10,9,8,7,6};
	//votesTable[1] = {9,10,7,8,6};
	//votesTable[2] = {1,2,3,4};  
    cobotQueue c({},3);
    c.send_best_plan(votesTable,3);
	return 0;
}
