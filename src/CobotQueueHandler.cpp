#include <Environment>
#include "ros/ros.h"


cobotQueue::cobotQueue(std::vector<uint> ids)
{
	cobotIds = ids;
}

uint cobotQueue::getQueueSize()
{
	return cobotIds.size();
}
		
bool cobotQueue::addCobot(uint id)
{
	cobotIds.push_back(id);
	return true;
}
		
bool cobotQueue::removeCobot(uint id)
{
	for (std::vector<uint>::iterator it = cobotIds.begin(); it != cobotIds.end(); it++)
	{
		if (it == id)
		{
			m_vecStackBlocks.erase(it);
			return true;
		}
	}

	return false;
}

bool cobotQueue::initiateVote()
{
	return false;
}

bool cobotQueue::collectVotes()
{
	return false;
}

bool cobotQueue::sendBestPlan()
{
	return false;
}