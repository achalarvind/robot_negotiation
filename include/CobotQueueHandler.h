#include "ros/ros.h"
#include "cobot.h"
#include <CSPSolver.h>

class cobotQueue{	
	private:
		//ros::NodeHandle nh;
		// ros::Publisher cobotQueuePub; //publishes number of Cobots in queue, among possibly other things
		std::vector<uint> cobotIds; //a list of the ID's of Cobot in queue
		std::vector<ccobot> cobots; //hack
		uint maxNDecisionMakers; //maximum number of Cobots which can participate in plan decision-making

	public:

        //Constructor
		cobotQueue(std::vector<uint> ,uint);

        //Queue Manager
        
		uint get_queue_size();
		
		bool add_cobot(uint id);
		
		bool remove_cobot(uint id);
        

        //Voting methods

		bool initiate_vote();

		std::vector< std::vector<uint> > collect_votes(std::vector<std::vector<DeliveryOrderSeq>>);

		int send_best_plan(std::vector< std::vector<uint> > votesTable, uint nPlans);

};	
	
