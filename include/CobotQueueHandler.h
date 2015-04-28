#include "ros/ros.h"
#include "cobot.h"
#include <CSPSolver.h>
#include "robot_negotiation/ResultTask.h"

class cobotQueue{	
	private:
		//ros::NodeHandle nh;
		// ros::Publisher cobotQueuePub; //publishes number of Cobots in queue, among possibly other things
		std::vector<uint> cobotIds; //a list of the ID's of Cobot in queue
		uint maxNDecisionMakers; //maximum number of Cobots which can participate in plan decision-making

	public:

		std::vector<ccobot> cobots; //hack
        //Constructor
        cobotQueue();
		cobotQueue(std::vector<uint> ,uint);

        //Queue Manager
        
		uint get_queue_size();
		
		bool add_cobot(uint id);
		
		bool remove_cobot(uint id);
        

        //Voting methods

		bool initiate_vote();

		std::vector< std::vector<double> > collect_votes(std::vector<std::vector<DeliveryOrderSeq>>);

		std::vector< std::vector<double> > collect_votes(std::vector<std::vector<robot_negotiation::ResultTask>> );

		int send_best_plan(std::vector< std::vector<double> > votesTable, uint nPlans);

};	
	
