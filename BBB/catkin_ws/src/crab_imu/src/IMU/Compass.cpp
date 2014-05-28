/*
 * Compass.cpp
 *
 *  Created on: 04.10.2013
 *      Author: tuuzdu
 */
#include "IMU.h"
#include "Compass.h"
#include <math.h>
//#include <stdio.h>

void Compass_Heading() {
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	c_magnetom_x = (float) (magnetom_x - SENSOR_SIGN[6] * M_X_MIN)
			/ (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6] * 0.5;
	c_magnetom_y = (float) (magnetom_y - SENSOR_SIGN[7] * M_Y_MIN)
			/ (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7] * 0.5;
	c_magnetom_z = (float) (magnetom_z - SENSOR_SIGN[8] * M_Z_MIN)
			/ (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8] * 0.5;
	//  printf ("\033[1Aroll: %.2f \tpitch: %.2f  \tyaw: %.2f\n", c_magnetom_x, c_magnetom_y, c_magnetom_z);

	// Tilt compensated Magnetic filed X:
	MAG_X = c_magnetom_x * cos_pitch + c_magnetom_y * sin_roll * sin_pitch
			+ c_magnetom_z * cos_roll * sin_pitch;
	// Tilt compensated Magnetic filed Y:
	MAG_Y = c_magnetom_y * cos_roll - c_magnetom_z * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y, MAG_X);
	//  printf ("\033[1Aroll: %.2f\n", MAG_Heading);
}

