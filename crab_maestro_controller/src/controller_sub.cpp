#include "controller_sub.hpp"

	const int rotation_direction[18] = 	{1,-1, 1,
								 	 	 1,-1, 1,
								 	 	 1,-1, 1,
								 	 	 1,	1,-1,
								 	 	 1,	1,-1,
								 	 	 1,	1,-1};


Controller::Controller(){

  // Get Min and Max joints limits
	node.param("joint_lower_limit", joint_lower_limit, -1.570796327);
	node.param("joint_upper_limit", joint_upper_limit, 1.570796327);
	limit_coef = 127 / ((joint_upper_limit - joint_lower_limit) / 2);

	node.param("port_name", port_name, std::string("/dev/ttyACM0"));

	maestro = Polstro::SerialInterface::createSerialInterface(port_name, 9600);
	sub = node.subscribe("joints_to_controller", 1, &Controller::chatterLegsState, this);
	ROS_INFO("Maestro servo controller is ready...");
}

void Controller::chatterLegsState (const crab_msgs::LegsJointsStateConstPtr &legs_jnts){

	float target_value;
	int s_num;

	for (int i=0; i<num_legs; i++){
		for (int j=0; j<num_joints; j++){
			s_num = i*3+j;
			target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef;
//			target_value = (neutral_ms + rotation_direction[s_num] * (legs_jnts->joints_state[i].joint[j] * coef_ms)) * 4;
			maestro->setTargetMSS(s_num, (unsigned char) target_value);
//			ROS_INFO("Servo %d: [%d]", s_num, (int) target_value);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller_sub");
	Controller c;
    ros::spin();
}


