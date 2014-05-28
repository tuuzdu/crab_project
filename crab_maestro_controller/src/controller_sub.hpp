
#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_


#include <ros/ros.h>
#include <crab_msgs/LegsJointsState.h>
#include "PolstroSerialInterface.h"

class Controller {
	public:
		Controller();

	private:

		ros::NodeHandle node;
		std::string port_name;
		Polstro::SerialInterface* maestro;
		double joint_lower_limit, joint_upper_limit, limit_coef;
		const static unsigned int num_joints = 3;
		const static unsigned int num_legs = 6;

		ros::Subscriber sub;

		void chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts);
};

#endif /* CONTROLLER_SUB_HPP_ */
