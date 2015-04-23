#include "ros/ros.h"

class cobotQueue{	
	private:
		ros::NodeHandle nh;
		// ros::Publisher cobotQueuePub; //publishes number of Cobots in queue, among possibly other things
		std::vector<uint> cobotIds; //a list of the ID's of Cobot in queue

	public:

		cobotQueue(std::vector<uint> ids);

		uint get_queue_size();
		
		bool add_cobot(uint id);
		
		bool remove_cobot(uint id);

		bool initiate_vote();

		bool collect_votes();

		bool send_best_plan();

};	
