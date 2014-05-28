
#include "imu_control.hpp"


using namespace std;


// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

ros::Time timer(0);   //general purpuse timer
ros::Time timer_old(0);
//long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

bool imu_on = false;
crab_msgs::BodyState body_state;
ros::Publisher move_body_pub;
ros::Subscriber body_cmd_sub;



void setup_IMU()
{

	ROS_INFO ("Start initialization IMU");
	IMU_Init();
	usleep (20000);
	for(int i=0;i<32;i++)    // We take some readings...
    {
		Read_Gyro();
		Read_Accel();
		for(int y=0; y<6; y++)   // Cumulate values
			AN_OFFSET[y] += AN[y];
		usleep (20000);
    }

	for(int y=0; y<6; y++)
		AN_OFFSET[y] = AN_OFFSET[y]/32;

	AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

	timer=ros::Time::now();
	usleep (20000);
	counter=0;
}

void teleopBodyCmd(const crab_msgs::BodyCommandConstPtr &body_cmd){
	if (body_cmd->cmd == body_cmd->IMU_START_CMD){
		setup_IMU();
		ros::Rate r(25);
		while (body_state.z >= -0.08){
			body_state.z -= 0.0025;
			r.sleep();
			move_body_pub.publish(body_state);
		}
		imu_on = true;
//		ros::Duration(2.5).sleep();

	}
	if (body_cmd->cmd == body_cmd->IMU_STOP_CMD){
		ros::Rate r(25);
		body_state.roll = 0;
		body_state.pitch = 0;
		body_state.yaw = 0;
		body_state.x = 0;
		body_state.y = 0;
		while (body_state.z <= -0.016){
			body_state.z += 0.0025;
			r.sleep();
			move_body_pub.publish(body_state);
		}
		imu_on = false;
//		ros::Duration(2).sleep();
	}
}


int main(int argc, char **argv){

	ros::init(argc, argv, "imu_control");
	ros::Time::init();
	ros::Rate loop_rate(45);
	ros::NodeHandle node;
//	crab_msgs::BodyState body_state;
	body_state.z = -0.08;
	body_state.leg_radius = 0.11;

	body_cmd_sub = node.subscribe<crab_msgs::BodyCommand>("/teleop/body_command", 1, teleopBodyCmd);
	move_body_pub = node.advertise<crab_msgs::BodyState>("/teleop/move_body",1);

//	setup_IMU();

#if CALIBRMODE == 0		// Magnetometer calibration is off

	while (node.ok()){
		if (imu_on){
			counter++;
			timer_old = timer;
			timer=ros::Time::now();
			if (timer>timer_old)
			  G_Dt = (timer.toSec()-timer_old.toSec());    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
			else
			  G_Dt = 0;

			// *** DCM algorithm
			// Data adquisition
			Read_Gyro();   // This read gyro data
			Read_Accel();     // Read I2C accelerometer

			if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
			  {
			  counter=0;
			  Read_Compass();    // Read I2C magnetometer
			  Compass_Heading(); // Calculate magnetic heading
			  }

			// Calculations...
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();
			// ***
	//	    printf ("\033[1Aroll: %.2f \tpitch: %.2f  \tyaw: %.2f\n", roll, pitch, yaw);

			if (roll > 0.015) body_state.roll = body_state.roll + 0.1 * roll;
			if (roll < -0.015) body_state.roll = body_state.roll + 0.1 * roll;
			if (pitch > 0.015) body_state.pitch = body_state.pitch - 0.1 * pitch;
			if (pitch < -0.015) body_state.pitch = body_state.pitch - 0.1 * pitch;

			if (body_state.roll > 0.35) body_state.roll = 0.35;
			if (body_state.roll < -0.35) body_state.roll = -0.35;
			if (body_state.pitch > 0.35) body_state.pitch = 0.35;
			if (body_state.pitch < -0.35) body_state.pitch = -0.35;

			move_body_pub.publish(body_state);
		}
	    ros::spinOnce();
	    loop_rate.sleep();
	}

	// For magnetometer calibration
#else

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

	LSM303::vector running_min = {2047, 2047, 2047}, running_max = {-2048, -2048, -2048};
	while (1){
		Read_Compass();

		running_min.x = min(running_min.x, (float) magnetom_x);
		running_min.y = min(running_min.y, (float) magnetom_y);
		running_min.z = min(running_min.z, (float) magnetom_z);

		running_max.x = max(running_max.x, (float) magnetom_x);
		running_max.y = max(running_max.y, (float) magnetom_y);
		running_max.z = max(running_max.z, (float) magnetom_z);

		printf ("\033[1Amin_x: %04d\tmin_y: %04d\tmin_z: %04d\tmax_x: %04d\tmax_y: %04d\tmax_z: %04d\n",
				(int) running_min.x, (int) running_min.y, (int) running_min.z,
				(int) running_max.x, (int) running_max.y, (int) running_max.z);
		usleep (100000);
	}

#endif
}



