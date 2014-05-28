#include "L3G.h"
#include "../sys/I2C.h"

#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)
#define BUS	1

// Construct I2C class
I2C Wire = I2C(BUS, L3GD20_ADDRESS_SA0_HIGH);

// Turns on the L3G's gyro and places it in normal mode.
void L3G::enableDefault(void) {
	// 0x0F = 0b00001111
	// Normal power mode, all axes enabled
	writeReg(L3G_CTRL_REG1, 0x0F);
	writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
}

// Writes a gyro register
void L3G::writeReg(char reg, char value) {
	Wire.writeI2CDeviceByte(reg, value);
}

// Reads a gyro register
char L3G::readReg(char reg) {
	return Wire.readI2CDeviceByte(reg);
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read() {
	// assert the MSB of the address to get the gyro
	// to do slave-transmit subaddress updating.
	Wire.readI2CDeviceMultipleByte(L3G_OUT_X_L | (1 << 7), 6);

	char xlg = Wire.dataBuffer[0];
	char xhg = Wire.dataBuffer[1];
	char ylg = Wire.dataBuffer[2];
	char yhg = Wire.dataBuffer[3];
	char zlg = Wire.dataBuffer[4];
	char zhg = Wire.dataBuffer[5];

//   combine high and low bytes
	g.x = (float) convertMsbLsb(xhg, xlg);
	g.y = (float) convertMsbLsb(yhg, ylg);
	g.z = (float) convertMsbLsb(zhg, zlg);
}

short L3G::convertMsbLsb(char msb, char lsb) {
	short temp;
	temp = (msb << 8 | lsb);
	return temp;
}
