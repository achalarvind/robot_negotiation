#include <Environment>
#include "ros/ros.h"

class cbaxter{	//publishes messages that are subscribed to by the drone_relay node.
	private:
		ros::NodeHandle nh;
		ros::Publisher negotiation_pub;
		int state;


	public:
	
	void start_negotiation() //send message to all robots to submit their "bids" and the object that they want.
	TableState Add_Element(Environment*, Block obBlock, Location obLoc, double height); //insert object into the environment
	TableState Remove_Element(Environment*, Block obBlock, Location obLoc); //remove object from the environment
};	