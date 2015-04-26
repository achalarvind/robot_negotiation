#include "ros/ros.h"
#include "../include/CobotQueueHandler.h"
#include <algorithm>

double variance(std::vector<uint> x,double mean)
{
	double variance = 0;
	for(int i =0; i < x.size(); i++)
	{
		variance += (x[i]-mean)*(x[i]-mean);
	}
	return variance/x.size();
}

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
	DeliveryOrderSeq a(1,s,0.0,0.0);
	return false;
}

std::vector< std::vector<uint> > cobotQueue::collect_votes(uint nPlans) // Listens to incoming messages and collects votes for all Cobots present in Queue (returns matrix of votes (rows are ranks; cols are voter IDs))
{
    std::vector< std::vector<uint> > votesTable;
	return votesTable;
}

int cobotQueue::send_best_plan(std::vector< std::vector<uint> > votesTable, uint nPlans) // Evaluates votes and send final plan id to Baxter
{
	//create an array of costs for every plan; return plan with lower average cost and highest variance 

	double* row_av;
	double* row_var;
	double min_cost = std::numeric_limits<double>::infinity();
	double min_cost_std = 0;
 	int best_plan = -1;
	row_av = new double[nPlans];
	row_var = new double[nPlans];
 	for(int plan = 0; plan < votesTable.size(); plan++)
 	{
		row_av[plan] = 0;
		row_var[plan] = 0;
 		for(int voter = 0; voter < votesTable[plan].size(); voter++)
 		{
 			row_av[plan] = row_av[plan] + votesTable[plan][voter];
 		}
		row_av[plan] = row_av[plan]/votesTable[plan].size();
		row_var[plan] = variance(votesTable[plan],row_av[plan]);
		if(row_av[plan]<min_cost){min_cost = row_av[plan]; best_plan = plan; min_cost_std=row_var[plan];}
		if(row_av[plan] == min_cost && min_cost_std < row_var[plan]){best_plan = plan; min_cost_std=row_var[plan];}
		//std::cout<<(boost::lexical_cast<std::string>(row_av[plan]))<<std::endl;
 	} 
	
	return best_plan;
}

int main(){
 	//std::vector< std::vector<uint> > votesTable {{10,9,8,7,6},{1,2,3,4},{10,0,0,0}};
    //std::cout<<variance({1,2,3},2)<<std::endl;
 	//votesTable[0] = {10,9,8,7,6};
	//votesTable[1] = {9,10,7,8,6};
	//votesTable[2] = {1,2,3,4};  
    //cobotQueue c({},3);
    //std::cout<<c.send_best_plan(votesTable,3)<<std::endl;
	return 0;
}
