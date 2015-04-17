#include <Environment>

class ccobot{	//publishes messages that are subscribed to by the drone_relay node.
	private:
		ros::NodeHandle nh;
		ros::Publisher negotiation_pub;

	public:
	cbaxter()	//Constructor for initialising the variable values.
	{		
		negotiation_pub=nh.advertise<std_msgs::Empty>("/start_negotiation",1);		//Publish cmd_velocity message.
	}
	void start_negotiation()
	{	
		reset_pub.publish(std_msgs::Empty());
	}

	TableState Add_Element(Environment*, Block obBlock, Location obLoc);
	TableState Remove_Element(Environment*, Block obBlock, Location obLoc);
};	