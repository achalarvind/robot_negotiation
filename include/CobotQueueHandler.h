#include <Environment>
#include "ros/ros.h"

class cobotQueue{	
	private:
		ros::NodeHandle nh;
		ros::Publisher cobotQueuePub; //publishes number of Cobots in queue, among possibly other things
		std::vector<uint> cobotIds; //a list of the ID's of Cobot in queue

	public:

		cobotQueue(std::vector<uint> ids);

		uint getQueueSize();
		
		bool addCobot();
		
		bool removeCobot();

		bool initiateVote();

		bool collectVotes();

		bool sendBestPlan();

};	