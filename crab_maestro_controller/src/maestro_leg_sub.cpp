
#include <ros/ros.h>
#include "crab_msgs/LegJointsState.h"
#include "PolstroSerialInterface.h"

typedef boost::shared_ptr<crab_msgs::LegJointsState const> LegStateConstPtr;
Polstro::SerialInterface* maestro;

void chatterLegState (const LegStateConstPtr& state)
{
	float target_value;

//	for (char i=0; i<3; i++){
		target_value = 6016 - (state->joint[0] * 2486.624);
		maestro->setTarget(9, (int) target_value);
		ROS_INFO("Servo %d: [%d]", 0, (int) target_value);
		target_value = 6016 + (state->joint[1] * 2486.624);
		maestro->setTarget(10, (int) target_value);
		ROS_INFO("Servo %d: [%d]", 1, (int) target_value);
		target_value = 6016 - (state->joint[2] * 2486.624);
		maestro->setTarget(11, (int) target_value);
		ROS_INFO("Servo %d: [%d]", 2, (int) target_value);
//	}
}

int main(int argc, char **argv)
{
	std::string portName = "/dev/ttyACM0";
	unsigned int baudRate = 9600;
	maestro = Polstro::SerialInterface::createSerialInterface(portName, baudRate);
	ros::init(argc, argv, "maestro_leg_sub");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("leg_states", 1, chatterLegState);
	ros::spin();

	return 0;
}
