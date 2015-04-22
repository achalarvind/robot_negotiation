#include <Environment>

	cbaxter::cbaxter()	//Constructor for initialising the variable values.
	{		
		negotiation_pub=nh.advertise<std_msgs::Empty>("/start_negotiation",1);		//Publish cmd_velocity message.
	}

	void cbaxter::start_negotiation()
	{	
		reset_pub.publish(std_msgs::Empty());
	}

	TableState cbaxter::Add_Element(Environment*, Block obBlock, Location obLoc, double height);
	TableState cbaxter::Remove_Element(Environment*, Block obBlock, Location obLoc);
};	