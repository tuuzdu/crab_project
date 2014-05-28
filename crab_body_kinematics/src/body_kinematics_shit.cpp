#include "body_kinematics.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};


bool BodyKinematics::init() {
	client = node.serviceClient<crab_msgs::GetLegIKSolver>("/crab_leg_kinematics/get_ik");
	joints_pub = node.advertise<crab_msgs::LegsJointsState>("joints_to_controller", 1);
	teleop_sub = node.subscribe<crab_msgs::BodyState>("/teleop/move_body", 1, &BodyKinematics::teleopCallback, this);

	for (int i=0; i<num_legs; i++){
		for (int j = 0; j < num_joints; j++) {
			legs.joints_state[i].joint[j] = 0;
		}
	}
//	joints_pub.publish(legs);

	std::string robot_desc_string;
	// Get URDF XML
	if (!node.getParam("robot_description", robot_desc_string)) {
		   ROS_FATAL("Could not load the xml from parameter: robot_description");
		   return false;
	   }

	// Get Root and Tip From Parameter Server
	node.param("root_name_body", root_name, std::string("thorax"));
	node.param("tip_name_body", tip_name, std::string("coxa"));

	// Load and Read Models
	if (!loadModel(robot_desc_string)) {
		ROS_FATAL("Could not load models!");
		return false;
	}


	ROS_INFO("Ready to receive teleop messages... ");

	return true;
}

bool BodyKinematics::loadModel(const std::string xml){
	//Construct tree with kdl_parser
	KDL::Tree tree;

	if (!kdl_parser::treeFromString(xml, tree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}
	ROS_INFO("Construct tree");

//	robot_state_publisher::RobotStatePublisher(tree);
//	std::map<std::string, double> joint_positions;
//	std::map<std::string, double>::const_iterator iter;
//	iter = joint_positions.begin();
//	(*iter).first

	std::string link_name_result;
	tf::StampedTransform transform;
	frames.resize(num_legs);
//	joints_pub.publish(legs);
	for (int i=0; i<num_legs; i++){
		link_name_result = tip_name + suffixes[i];
		try{
			tf_listener.waitForTransform(root_name, link_name_result, ros::Time(0), ros::Duration(5.0) );
			tf_listener.lookupTransform(root_name, link_name_result, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		tf::TransformTFToKDL(transform, frames[i]);
	}

	return true;
}

bool BodyKinematics::calculateKinematics (const crab_msgs::BodyStateConstPtr body_ptr){

	//Body rotation
	rotation = KDL::Rotation::RPY(body_ptr->roll,body_ptr->pitch,body_ptr->yaw);

	//Distance from body center to leg tip
	femur_frame = KDL::Frame (KDL::Vector (body_ptr->leg_radius,0,0));

	//Offset from center
	offset_vector = KDL::Vector (body_ptr->x,body_ptr->y,body_ptr->z);
	rotate_correction = KDL::Vector (body_ptr->z * tan(body_ptr->pitch), -(body_ptr->z * tan(body_ptr->roll)), 0);

	for (int i=0; i<num_legs; i++){
		//Get tip frames
		tibia_foot_frame = frames[i] * femur_frame;
		//Get tip vectors with body position
		final_vector[i] = (rotation * tibia_foot_frame.p) + offset_vector + rotate_correction;
		ROS_DEBUG("Position vector leg%s\tx: %f\ty: %f\tz: %f", suffixes[i].c_str(),
							final_vector[i](0),final_vector[i](1),final_vector[i](2));
	}

	ROS_DEBUG("Call service: /leg_ik_service/get_ik");
	if (!callService(final_vector)){
		return 0;
	}

	return true;
}

bool BodyKinematics::callService (KDL::Vector* vector){
	crab_msgs::LegPositionState leg_pos_buf;
	srv.request.leg_number.clear();
	srv.request.target_position.clear();
	srv.request.current_joints.clear();

	//Creating message to request
	for (int i=0; i<num_legs; i++){
		srv.request.leg_number.push_back(i);
		leg_pos_buf.x = vector[i].x();
		leg_pos_buf.y = vector[i].y();
		leg_pos_buf.z = vector[i].z();
		srv.request.target_position.push_back(leg_pos_buf);
		srv.request.current_joints.push_back(legs.joints_state[i]);
	}
	//Call service and parsing response
	if (client.call(srv)){
		if (srv.response.error_codes==srv.response.IK_FOUND){
			for (int i=0; i<num_legs; i++){
				for (int j = 0; j < num_joints; j++) {
						legs.joints_state[i].joint[j] = srv.response.target_joints[i].joint[j];
				}
				ROS_DEBUG("Joints received leg%s\t1: %f\t2: %f\t3: %f", suffixes[i].c_str(),
							legs.joints_state[i].joint[0],
							legs.joints_state[i].joint[1],
							legs.joints_state[i].joint[2]);
			}
		}
		else {
			ROS_ERROR("An IK solution could not be found");
			return 0;
		}
	}
	else {
		ROS_ERROR("Failed to call service");
		return 0;
	}
	return true;
}


void BodyKinematics::teleopCallback(const crab_msgs::BodyStateConstPtr &body_state){
	if (calculateKinematics(body_state)){
		joints_pub.publish(legs);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "body_kinematics");
	BodyKinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }

    ros::spin();
    return 0;
}
