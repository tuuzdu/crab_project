#include "teleop_joy.hpp"

TeleopJoy::TeleopJoy(){
	node.param("clearance", z, 0.045);

	body_state.leg_radius = 0.11;
	body_state.z = -z;
	start_flag = false;

	joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);
	move_body_pub = node.advertise<crab_msgs::BodyState>("/teleop/move_body",1);
	body_cmd_pub = node.advertise<crab_msgs::BodyCommand>("/teleop/body_command",1);
	gait_cmd_pub = node.advertise<crab_msgs::GaitCommand>("/teleop/gait_control",1);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

	if (joy->buttons[button_start] && !imu_flag){
		if (!start_flag){
			start_flag = true;
			body_command.cmd = body_command.STAND_UP_CMD;
			body_cmd_pub.publish(body_command);
		}
		else{
			start_flag = false;
			body_command.cmd = body_command.SEAT_DOWN_CMD;
			body_cmd_pub.publish(body_command);
		}
		ros::Duration(1).sleep();
	}

	if (joy->buttons[button_imu] && !start_flag){
		if (!imu_flag){
			imu_flag = true;
			body_command.cmd = body_command.IMU_START_CMD;
			body_cmd_pub.publish(body_command);
		}
		else{
			imu_flag = false;
			body_command.cmd = body_command.IMU_STOP_CMD;
			body_cmd_pub.publish(body_command);
		}
		ros::Duration(1).sleep();
	}

	if (joy->buttons[button_gait_switch]){
		if (!gait_flag){
			gait_flag = true;
			gait_command.cmd = gait_command.STOP;
		}
		else{
			gait_flag = false;
			gait_command.cmd = gait_command.STOP;
		}
		gait_cmd_pub.publish(gait_command);
		ros::Duration(0.5).sleep();
	}

	//START Button pressed
	if (start_flag){
		// RPY Signal
		if (joy->buttons[button_left_shift]){
			body_state.roll = 0.25 * joy->axes[axis_body_roll];
			body_state.pitch = -0.25 * joy->axes[axis_body_pitch];
			body_state.yaw = -0.28 * joy->axes[axis_body_yaw];
			move_body_pub.publish(body_state);
	//		ROS_INFO_STREAM("roll  " << body_state.roll << "  pitch  " << body_state.pitch << " yaw  " << body_state.yaw);
		}
		// Offset Signal
		if (joy->buttons[button_right_shift]){
			body_state.y = -0.05 * joy->axes[axis_body_y_off];
			body_state.x = -0.05 * joy->axes[axis_body_x_off];
			if (joy->axes[axis_body_z_off] < 0){
				body_state.z = -0.03 * joy->axes[axis_body_z_off] - z;
			}
			else{
				body_state.z = -0.1 * joy->axes[axis_body_z_off] - z;
			}
			move_body_pub.publish(body_state);
	//		ROS_INFO_STREAM("x  " << body_state.x << "  y  " << body_state.y << "  z  " << body_state.z);
		}
		// Gait Signals
		if (!joy->buttons[button_left_shift] && !joy->buttons[button_right_shift]){
			if (joy->axes[axis_fi_x] != 0 || joy->axes[axis_fi_y] != 0){
				if (!gait_flag){
					gait_command.cmd = gait_command.RUNRIPPLE;
				}
				else{
					gait_command.cmd = gait_command.RUNTRIPOD;
				}
				// Gait Fi and Velocity Signal
				float a, b;
				a = pow (joy->axes[axis_fi_x], 2);
				b = pow (joy->axes[axis_fi_y], 2);
				gait_command.fi = atan2(joy->axes[axis_fi_x],joy->axes[axis_fi_y]);
				gait_command.scale = pow (a+b, 0.5) > 1 ? 1 : pow (a+b, 0.5);
				gait_command.alpha = 0;
			}
//			else{
//				gait_command.cmd = gait_command.PAUSE;
//			}

			// Gait Alpha and Direction Signal
			if (joy->axes[axis_alpha] != 0 || joy->axes[axis_scale] != 0){
	//			ROS_INFO_STREAM("y  " << joy->axes[axis_direction] << "  x  " << joy->axes[axis_alpha]);
				if (!gait_flag){
					gait_command.cmd = gait_command.RUNRIPPLE;
				}
				else{
					gait_command.cmd = gait_command.RUNTRIPOD;
				}
				gait_command.fi = (joy->axes[axis_scale] > 0) ? 0 :  3.14;
				gait_command.scale = joy->axes[axis_scale];
				if (gait_command.scale < 0) gait_command.scale *=-1;
				gait_command.alpha = ((joy->axes[axis_alpha] > 0) ? 1 :  -1) * 0.06 * (1-gait_command.scale) + 0.11 * joy->axes[axis_alpha];
			}
//			else{
//				gait_command.cmd = gait_command.PAUSE;
//			}

			if (!joy->axes[axis_alpha] && !joy->axes[axis_scale] && !joy->axes[axis_fi_x] && !joy->axes[axis_fi_y]){
				gait_command.cmd = gait_command.PAUSE;
			}
			gait_cmd_pub.publish(gait_command);
		}
	}
	//START Button didn't press
	else{
		if (joy->buttons[button_right_shift_2] && !imu_flag){
			body_state.z = -0.01;
			body_state.leg_radius = 0.06 * joy->axes[axis_body_yaw] + 0.11;
			move_body_pub.publish(body_state);
		}
		gait_command.cmd = gait_command.STOP;
		gait_cmd_pub.publish(gait_command);

	}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_joy");
	ROS_INFO("Starting ps3 teleop converter, take care of your controller now...");
	TeleopJoy teleop_joy;
	ros::spin();
//    return 0;
}
