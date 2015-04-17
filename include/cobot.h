#include <Environment>
#include "ros/ros.h"

class ccobot{	//publishes messages that are subscribed to by the drone_relay node.
	private:
		ros::NodeHandle nh;
		ros::Publisher negotiation_pub;
		int state;

	public:
	void start_negotiation()

};	