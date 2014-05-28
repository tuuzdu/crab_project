#include "gait_test.hpp"



int main(int argc, char **argv)
{
	ros::init(argc, argv, "gait_test");


	KDL::RotationalInterpolation_SingleAxis rot;
	KDL::Path_RoundedComposite* rounded_path;
	double radius;
	double eqradius;
//	KDL::Path_Line *path;

//	rot = new KDL::RotationalInterpolation_SingleAxis;
	rot.SetStartEnd(KDL::Rotation::Identity(), KDL::Rotation::Identity());

	radius = 0.001;
	eqradius = 0.001;
	rounded_path = new KDL::Path_RoundedComposite(radius,eqradius,&rot);
	rounded_path -> Add(KDL::Frame (KDL::Vector (0,0,0)));
	rounded_path -> Add(KDL::Frame (KDL::Vector (0,0,0.05)));
	rounded_path -> Add(KDL::Frame (KDL::Vector (0.1,0,0.05)));
	rounded_path -> Add(KDL::Frame (KDL::Vector (0.1,0,0)));
	rounded_path -> Add(KDL::Frame (KDL::Vector (0,0,0)));
	rounded_path -> Finish();

	double l;
	l = rounded_path -> PathLength();
	ROS_INFO("l = %f ", l);

	KDL::Frame frame;
	for (double i=0; i<rounded_path -> PathLength(); i=i+0.005){
		frame = rounded_path -> Pos(i);
		ROS_INFO("x = %f y = %f z = %f ", frame.p.x(), frame.p.y(), frame.p.z());
	}
//	l = rounded_path -> LengthToS(l);
//	l = path_comp.LengthToS(l);

//    ros::spin();
    return 0;
}
