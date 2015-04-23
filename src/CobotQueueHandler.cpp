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

std::vector<std::vector<uint*>> cobotQueue::collect_votes(uint nPlans) // Listens to incoming messages and collects votes for all Cobots present in Queue (returns matrix of votes (rows are ranks; cols are voter IDs))
{

	return false;
}

bool cobotQueue::send_best_plan(std::vector<std::vector<uint*>> votesTable, uint nPlans) // Evaluates votes and send final plan to Baxter
{
	//create an array of scores for every plan 

	////

 	for(int rank = 0; rank < votesTable.size(); rank++)
 	{
 		for(int voter = 0; voter < votesTable[rank].size(); voter++)
 		{
 			if()
 		}

 	} 
	

	return false;
}

int main(){

    send_best_plan
	return 0;
}
