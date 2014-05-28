#include "leg_ik_service.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

LegKinematics::LegKinematics():	node_private("~"){}

bool LegKinematics::init() {
	 std::string robot_desc_string;
    // Get URDF XML
    if (!node.getParam("robot_description", robot_desc_string)) {
           ROS_FATAL("Could not load the xml from parameter: robot_description");
           return false;
    }

    // Get Root and Tip From Parameter Server
    node_private.param("root_name", root_name, std::string("thorax"));
    node_private.param("tip_name", tip_name, std::string("tibia_foot"));

    // Load and Read Models
    if (!loadModel(robot_desc_string)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Min and Max joints limits
    node.param("joint_lower_limit", joint_lower_limit, -(KDL::PI/2));
    node.param("joint_upper_limit", joint_upper_limit, KDL::PI/2);
    joint_min.resize(num_joints);
    joint_max.resize(num_joints);
    for (unsigned int i=0; i<num_joints; i++){
    	joint_min(i) = joint_lower_limit;
    	joint_max(i) = joint_upper_limit;
    }
    // Get Solver Parameters
	int maxIterations;
	double epsilon;

	node_private.param("maxIterations", maxIterations, 100);
	node_private.param("epsilon", epsilon, 1e-3);

	// Build Solvers
	for (unsigned int i=0; i<num_legs; i++){
		  fk_solver[i] = new KDL::ChainFkSolverPos_recursive(*chains_ptr[i]);
		  ik_solver_vel[i] = new KDL::ChainIkSolverVel_pinv(*chains_ptr[i]);
		  ik_solver_pos[i] = new KDL::HP_ChainIkSolverPos_NR_JL(*chains_ptr[i], joint_min, joint_max,
				  *fk_solver[i], *ik_solver_vel[i], maxIterations, epsilon);
	}

	ROS_INFO("Advertising service");
	ik_service = node_private.advertiseService("get_ik",&LegKinematics::getLegIKSolver,this);
	ROS_INFO("Ready to client's request...");
	return true;
}

bool LegKinematics::loadModel(const std::string xml) {
    KDL::Tree tree;
    KDL::Chain chain;
    std::string tip_name_result;

    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    ROS_INFO("Construct tree");

    for (int i=0; i<num_legs; i++){
    	tip_name_result = tip_name + suffixes[i];
		if (!tree.getChain(root_name, tip_name_result, chain)) {
			ROS_ERROR("Could not initialize chain_%s object", suffixes[i].c_str());
			return false;
		}
		chains_ptr[i] = new KDL::Chain(chain);
    }
    ROS_INFO("Construct chains");

    return true;
}

bool LegKinematics::getLegIKSolver (	crab_msgs::GetLegIKSolver::Request &request,
										crab_msgs::GetLegIKSolver::Response &response){

	crab_msgs::LegPositionState leg_dest_pos;
	response.target_joints.clear();

	for (int i = 0; i < request.leg_number.size(); i++){
		leg_dest_pos = request.target_position[i];
		KDL::JntArray jnt_pos_in(num_joints);
		KDL::JntArray jnt_pos_out(num_joints);

		//Get initial joints and frame
		for (unsigned int j=0; j < num_joints; j++) {
			jnt_pos_in(j) = request.current_joints[i].joint[j];
		}
		KDL::Frame F_dest (KDL::Vector(leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z));

		//IK solver
		int ik_valid = ik_solver_pos[request.leg_number[i]]->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

		if (ik_valid >= 0) {
			crab_msgs::LegJointsState jnt_buf;
			for (unsigned int j=0; j<num_joints; j++) {
					jnt_buf.joint[j] = jnt_pos_out(j);
				}
			response.target_joints.push_back(jnt_buf);
			response.error_codes = response.IK_FOUND;
			ROS_DEBUG("IK Solution for leg%s found", suffixes[request.leg_number[i]].c_str());
		}
		else {
			response.error_codes = response.IK_NOT_FOUND;
			ROS_ERROR("An IK solution could not be found");
			return true;
		}
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_ik_service");
	LegKinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }

    ros::spin();
    return 0;
}
