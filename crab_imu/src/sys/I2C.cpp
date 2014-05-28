/*
 * I2C.cpp
 *
 *  Created on: 23.08.2013
 *      Author: tuuzdu
 */
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
//#include <stropts.h>
#include <stdio.h>
#include "I2C.h"
#include <iostream>
//#include <math.h>
using namespace std;
#define MAX_BUS 64

I2C::I2C(int bus, int address) {
	I2CBus = bus;
	I2CAddress = address;
	//readFullSensorState();
}

int I2C::writeI2CDeviceByte(char address, char value) {

//    cout << "Starting I2C sensor state write" << endl;
	char namebuf[MAX_BUS];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	int file;
	if ((file = open(namebuf, O_RDWR)) < 0) {
		cout << "Failed to open Sensor on " << namebuf << " I2C Bus" << endl;
		return (1);
	}
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0) {
		cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
		return (2);
	}

	// need to set the ctrl_reg0 ee_w bit. With that set the image registers change properly.
	// need to do this or can't write to 20H ... 3Bh
	// Very Important... wrote a 0x10 to 0x0D and it worked!!!
	//   char buf[2];
	//     buf[0] = BANDWIDTH;
	//     buf[1] = 0x28;
	//     buf[2] = 0x65;
	//  if ( write(file,buf,2) != 2) {
	//	  cout << "Failure to write values to I2C Device " << endl;
	//  }

	char buffer[2];
	buffer[0] = address;
	buffer[1] = value;
	if (write(file, buffer, 2) != 2) {
		cout << "Failure to write values to I2C Device address." << endl;
		return (3);
	}
	close(file);
//   cout << "Finished I2C sensor state write" << endl;
	return 0;
}

int I2C::readI2CDeviceMultipleByte(char address, int quantity) {

	char namebuf[MAX_BUS];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	int file;
	if ((file = open(namebuf, O_RDWR)) < 0) {
		cout << "Failed to open BMA180 Sensor on " << namebuf << " I2C Bus"
				<< endl;
		return (1);
	}
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0) {
		cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
		return (2);
	}

	char buf[1];
	buf[0] = address;
	if (write(file, buf, 1) != 1) {
		cout << "Failed to Reset Address in readFullSensorState() " << endl;
	}

	//int numberBytes = BUFFER_SIZE;
	int bytesRead = read(file, this->dataBuffer, quantity);
	if (bytesRead == -1) {
		cout << "Failure to read Byte Stream in readFullSensorState()" << endl;
	}
	close(file);

//	    if (this->dataBuffer[0]!=0x03){
//	    	cout << "MAJOR FAILURE: DATA WITH BMA180 HAS LOST SYNC!" << endl;
//	    }
//
//	    cout << "Number of bytes read was " << bytesRead << endl;
//	    for (int i=0; i<quantity; i++){
//	    	printf("Byte %02d is 0x%02x\n", i, dataBuffer[i]);
//	    }
	//cout << "Closing BMA180 I2C sensor state read" << endl;
	return 0;
}

char I2C::readI2CDeviceByte(char address) {

	//  cout << "Starting BMA180 I2C sensor state byte read" << endl;
	char namebuf[MAX_BUS];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	int file;
	if ((file = open(namebuf, O_RDWR)) < 0) {
		cout << "Failed to open Sensor on " << namebuf << " I2C Bus" << endl;
		return (1);
	}
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0) {
		cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
		return (2);
	}

	char buf[1];
	buf[0] = address;
	if (write(file, buf, 1) != 1) {
		cout << "Failed to Reset Address in readFullSensorState() " << endl;
	}

	unsigned char buffer[1];
	buffer[0] = address;
	if (read(file, buffer, 2) != 2) {
		cout << "Failure to read value from I2C Device address." << endl;
	}
	close(file);
	//cout << (int) buffer [0] << endl;
	return buffer[0];
}
