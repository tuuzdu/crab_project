
#ifndef LEG_IK_SERVICE_HPP_
#define LEG_IK_SERVICE_HPP_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include "chainiksolvervel_pinv.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include "hp_chainiksolverpos_nr_jl.hpp"
#include <crab_msgs/GetLegIKSolver.h>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class LegKinematics {
	public:
		LegKinematics();
		bool init();
	private:
		ros::NodeHandle node, node_private;
		std::string root_name, tip_name;
		double joint_lower_limit, joint_upper_limit;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;

		KDL::Chain* chains_ptr[6];
		KDL::JntArray joint_min, joint_max;
		KDL::ChainFkSolverPos_recursive* fk_solver[6];
		KDL::HP_ChainIkSolverPos_NR_JL* ik_solver_pos[6];
		KDL::ChainIkSolverVel_pinv* ik_solver_vel[6];

		ros::ServiceServer ik_service;

		bool loadModel(const std::string xml);
		bool getLegIKSolver (	crab_msgs::GetLegIKSolver::Request &request,
								crab_msgs::GetLegIKSolver::Response &response);


};



#endif /* LEG_IK_SERVICE_HPP_ */
