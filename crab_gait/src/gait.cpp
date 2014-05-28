#include "gait.hpp"

Gait::Gait(){
	legs_queue.push(5);
	legs_queue.push(0);
	legs_queue.push(4);
	legs_queue.push(2);
	legs_queue.push(3);
	legs_queue.push(1);
}


void Gait::setTrapezoid(double low_r, double high_r, double h, double z){
	low_rad = low_r;
	high_rad = high_r;
	height = z - h;
	z_body = z;
}

void Gait::setFi (double fi){
	//Set vectors like x = r * cos(fi), y = r * sin(fi) in 3d space
	//     B--high_rad---C
	//    /               \  <- h
	//   A-----low_rad-----D <- z

	a.p = KDL::Vector (-low_rad * cos(fi), -low_rad * sin(fi), -z_body);
	b.p = KDL::Vector (-high_rad * cos(fi), -high_rad * sin(fi), -height);
	c.p = KDL::Vector (high_rad * cos(fi), high_rad * sin(fi), -height);
	d.p = KDL::Vector (low_rad * cos(fi), low_rad * sin(fi), -z_body);
}

void Gait::setAlpha (double alpha){
	a.M = KDL::Rotation::RotZ(-alpha);
	b.M = KDL::Rotation::RotZ(-alpha/2);
	c.M = KDL::Rotation::RotZ(alpha/2);
	d.M = KDL::Rotation::RotZ(alpha);
}

void Gait::setPath (){
	path_support = new KDL::Path_Line(d, a, &rot, 0.005, true);

	path_transfer = new KDL::Path_RoundedComposite (0.02,0.005,&rot);
	path_transfer -> Add(a);
	path_transfer -> Add(b);
	path_transfer -> Add(c);
	path_transfer -> Add(d);
	path_transfer -> Finish();
}

void Gait::setTrajectory (double sup_path_duration, double tran_path_duration){
	prof_support.SetProfileDuration(0,path_support->PathLength(), sup_path_duration);
	prof_transfer.SetProfileDuration(0,path_transfer->PathLength(), tran_path_duration);
	trajectory_transfer = new KDL::Trajectory_Segment (path_transfer, &prof_transfer);
	trajectory_support = new KDL::Trajectory_Segment (path_support, &prof_support);
}

KDL::Vector* Gait::RunTripod (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration, duration);
	KDL::Frame frame;
	if (run_state == false){
		run_state = true;
		phase = 0;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}
	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;
	for (int i=phase; i<num_legs; i+=2){
		frame = trajectory_transfer -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
		final_vector[i] = frame.M *(*(vector_iter+i)).p + frame.p;
	}
	for (int i=!phase; i<num_legs; i+=2){
		frame = trajectory_support -> Pos(passed_sec);
		frame.p.x(frame.p.data[0]*scale);
		frame.p.y(frame.p.data[1]*scale);
		final_vector[i] = frame.M * (*(vector_iter+i)).p  + frame.p;
	}

	if (passed_sec >= duration-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
		phase = !phase;
	}
	return final_vector;
}

KDL::Vector* Gait::RunRipple (std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration){
	double phase_offset = duration / num_legs;
	setFi(fi);
	setAlpha(alpha);
	setPath();
	setTrajectory(duration*2/3, duration/3);
	KDL::Frame frame;

	if (run_state == false){
		run_state = true;
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;
	}

	if (pause_state == true){
		begin_sec = ros::Time::now().toSec() - passed_sec;
		pause_state = false;
	}

	passed_sec = ros::Time::now().toSec() - begin_sec;

	getTipVector(trajectory_transfer,	phase_offset,	vector_iter, scale);
	getTipVector(trajectory_transfer,	0,				vector_iter, scale);
	getTipVector(trajectory_support,	3*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	2*phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	phase_offset,	vector_iter, scale);
	getTipVector(trajectory_support,	0,				vector_iter, scale);

	if (passed_sec >= phase_offset-0.02 && run_state==true){
		begin_sec = ros::Time::now().toSec();
		passed_sec = 0;

		legs_queue.push(legs_queue.front());
		legs_queue.pop();
	}
	return final_vector;
}

void Gait::getTipVector (KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale){
	KDL::Frame frame;
	frame = trajectory -> Pos(passed_sec + phase_offset);
	frame.p.x(frame.p.data[0]*scale);
	frame.p.y(frame.p.data[1]*scale);
	final_vector[legs_queue.front()] = frame.M * (*(vector_iter + legs_queue.front())).p + frame.p;
	legs_queue.push(legs_queue.front());
	legs_queue.pop();
}


void Gait::Pause (){
	pause_state = true;
}

void Gait::Stop (){
	run_state = false;
}
