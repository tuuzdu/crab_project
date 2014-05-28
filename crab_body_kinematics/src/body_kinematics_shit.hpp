
#ifndef BODY_KINEMATICS_HPP_
#define BODY_KINEMATICS_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
//#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <crab_msgs/GetLegIKSolver.h>
#include <crab_msgs/LegsJointsState.h>
#include <crab_msgs/BodyState.h>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class BodyKinematics {
	public:
		BodyKinematics();
		bool init();

	private:
		ros::NodeHandle node;
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
		crab_msgs::LegsJointsState legs;
		crab_msgs::GetLegIKSolver srv;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;

		KDL::Rotation rotation;
		KDL::Frame tibia_foot_frame, femur_frame;
		KDL::Vector offset_vector, rotate_correction, final_vector [num_legs];

		tf::TransformListener tf_listener;

		ros::ServiceClient client;
		ros::Publisher joints_pub;
		ros::Subscriber teleop_sub;

		bool loadModel(const std::string xml);
		bool calculateKinematics (const crab_msgs::BodyStateConstPtr body_ptr);
		bool callService (KDL::Vector* vector);
		void teleopCallback (const crab_msgs::BodyStateConstPtr &body_state);
};

BodyKinematics::BodyKinematics(){}


#endif /* BODY_KINEMATICS_HPP_ */
