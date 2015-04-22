#include "../include/CobotQueueHandler.h"

cobotQueue::cobotQueue(std::vector<uint> ids)
{
	cobotIds = ids;
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

bool cobotQueue::collect_votes() // Listens to incoming messages and collects votes for all Cobots present in Queue (returns matrix of votes by reference)
{
	return false;
}

bool cobotQueue::send_best_plan() // Sends final plan to Baxter
{
	return false;
}

int main(){

	return 0;
}
