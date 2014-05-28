

#ifndef GAIT_TRIPOD_HPP_
#define GAIT_TRIPOD_HPP_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
//#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <crab_msgs/GetLegIKSolver.h>
#include <crab_msgs/LegsJointsState.h>
#include <crab_msgs/GaitCommand.h>
#include "gait.hpp"

#define NUM_LEGS 6
#define NUM_JOINTS 3

class GaitKinematics {
	public:
		GaitKinematics();
		bool init();
		void gaitGenerator();

	private:
		ros::NodeHandle node;
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
		double fi;
		crab_msgs::LegsJointsState legs;
		crab_msgs::GetLegIKSolver srv;
		crab_msgs::GaitCommand gait_command;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double trap_low_r, trap_high_r, trap_h, trap_z;
		double d_ripple, d_tripod;

		ros::ServiceClient client;
		ros::Publisher joints_pub;
		ros::Subscriber gait_control_sub;

		bool loadModel(const std::string xml);
		bool callService (KDL::Vector* vector);
		void teleopGaitCtrl (const crab_msgs::GaitCommandConstPtr &gait_cmd);

};

GaitKinematics::GaitKinematics(){}



#endif /* GAIT_TRIPOD_HPP_ */
