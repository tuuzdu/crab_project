#include "ros/ros.h"
#include <kdl_parser/kdl_parser.hpp>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "rpy_kinematics");
	ros::NodeHandle node;

	KDL::Tree tree;
	std::string robot_desc_string;
	node.param("robot_description", robot_desc_string, std::string("robot_description"));
	if (!kdl_parser::treeFromString(robot_desc_string, tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
		}

	std::vector<KDL::Frame> frames;
	std::map<std::string,KDL::TreeElement>::const_iterator segments_iter;
	std::string name[2] = {"leg_center_", "coxa_"}, side[2] = {"r", "l"}, num[3] = {"1", "2", "3"}, link_name;
	for (int name_i=0; name_i<2; name_i++){
		for (int side_i=0; side_i<2; side_i++){
			for (int num_i=0; num_i<3; num_i++){
				link_name = name [name_i] + side[side_i] + num[num_i];
				segments_iter = tree.getSegment(link_name);
				frames.push_back((*segments_iter).second.segment.getFrameToTip());
			}
		}
	}

	std::vector<KDL::Frame>::const_iterator leg_center_frame_iter, coxa_frame_iter;
	leg_center_frame_iter = frames.begin();
	coxa_frame_iter = frames.begin() + 6;
	KDL::Rotation rotation = KDL::Rotation::RPY(0,0,0);
	KDL::Frame tibia_tip;
	KDL::Frame femur_frame = KDL::Frame (KDL::Vector (0.0294,0,0));
	for (int i=0; i<6; i++){
		tibia_tip = (*(leg_center_frame_iter + i)) * (*(coxa_frame_iter + i)) * femur_frame;
		KDL::Vector v1 = rotation * tibia_tip.p;
		ROS_INFO("\nv1 1: %f\nv1 2: %f\nv1 3: %f\n", v1(0),v1(1),v1(2));
	}


	return 0;
}
