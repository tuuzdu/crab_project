/*
 * LSM303.cpp
 *
 *  Created on: 30.09.2013
 *      Author: tuuzdu
 */

#include "LSM303.h"
#include "../sys/I2C.h"
//#include <stdio.h>

#define LSM303_ACC_ADDRESS_SA0_A_HIGH (0x32 >> 1)
#define LSM303_MAG_ADDRESS            (0x3C >> 1)
#define BUS	1

// Construct I2C class
I2C Wire_Acc = I2C(BUS, LSM303_ACC_ADDRESS_SA0_A_HIGH);
I2C Wire_Mag = I2C(BUS, LSM303_MAG_ADDRESS);

// Turns on the LSM303's accelerometer and magnetometers and places them in normal mode.
void LSM303::enableDefault(void) {
	writeAccReg(LSM303_CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
	writeAccReg(LSM303_CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10 on DLHC; high resolution output mode
	writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode, 15 Hz default
}

// Writes an accelerometer register
void LSM303::writeAccReg(char reg, char value) {
	Wire_Acc.writeI2CDeviceByte(reg, value);
}

// Writes a magnetometer register
void LSM303::writeMagReg(char reg, char value) {
	Wire_Mag.writeI2CDeviceByte(reg, value);
}

// Reads an accelerometer register
char LSM303::readAccReg(char reg) {
	return Wire_Acc.readI2CDeviceByte(reg);
}

// Reads a magnetometer register
char LSM303::readMagReg(char reg) {
	return Wire_Mag.readI2CDeviceByte(reg);
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM303::readAcc(void) {
	// assert the MSB of the address to get the accelerometer
	// to do slave-transmit subaddress updating.
	Wire_Acc.readI2CDeviceMultipleByte(LSM303_OUT_X_L_A | (1 << 7), 6);

	char xla = Wire_Acc.dataBuffer[0];
	char xha = Wire_Acc.dataBuffer[1];
	char yla = Wire_Acc.dataBuffer[2];
	char yha = Wire_Acc.dataBuffer[3];
	char zla = Wire_Acc.dataBuffer[4];
	char zha = Wire_Acc.dataBuffer[5];

	// combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
	// GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
	// if you port it to a compiler that does a logical right shift instead.

	a.x = (float) convertMsbLsb(xha, xla, 4);
	a.y = (float) convertMsbLsb(yha, yla, 4);
	a.z = (float) convertMsbLsb(zha, zla, 4);
//  printf ("aroll: %.2f \tapitch: %.2f  \tayaw: %.2f\n", a.x, a.y, a.z);
}

// Reads the 3 magnetometer channels and stores them in vector m
void LSM303::readMag(void) {
	Wire_Mag.readI2CDeviceMultipleByte(LSM303_OUT_X_H_M, 6);

	char xhm = Wire_Mag.dataBuffer[0];
	char xlm = Wire_Mag.dataBuffer[1];
	char zhm = Wire_Mag.dataBuffer[2];
	char zlm = Wire_Mag.dataBuffer[3];
	char yhm = Wire_Mag.dataBuffer[4];
	char ylm = Wire_Mag.dataBuffer[5];
	// combine high and low bytes
	m.x = (float) convertMsbLsb(xhm, xlm, 0);
	m.y = (float) convertMsbLsb(yhm, ylm, 0);
	m.z = (float) convertMsbLsb(zhm, zlm, 0);
//  printf ("mroll: %.2f \tmpitch: %.2f \tmyaw: %.2f\n", m.x, m.y, m.z);
}

short LSM303::convertMsbLsb(char msb, char lsb, char discard) {
	short temp;
	temp = (msb << 8 | lsb);
	temp = temp >> discard;
	return temp;
}
