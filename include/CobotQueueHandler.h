#include "ros/ros.h"

class cobotQueue{	
	private:
		//ros::NodeHandle nh;
		//ros::Publisher cobotQueuePub; //publishes number of Cobots in queue, among possibly other things
		std::vector<uint> cobotIds; //a list of the ID's of Cobot in queue
		uint maxNDecisionMakers = 10; //maximum number of Cobots which can participate in plan decision-making

	public:

        //Constructor
		cobotQueue(std::vector<uint> ids,uint maxN);

        //Queue Manager
        
		uint get_queue_size();
		
		bool add_cobot(uint id);
		
		bool remove_cobot(uint id);
        

        //Voting methods

		bool initiate_vote();

		bool collect_votes();

		bool send_best_plan();

};	
	