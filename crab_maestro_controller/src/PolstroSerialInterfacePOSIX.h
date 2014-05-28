/*
 * PolstroSerialInterfacePOSIX.h
 *
 *  Created on: 10.10.2013
 *      Author: tuuzdu
 */

#ifndef POLSTROSERIALINTERFACEPOSIX_H_
#define POLSTROSERIALINTERFACEPOSIX_H_

#pragma once

#include "PolstroSerialInterface.h"

namespace Polstro {

class SerialInterfacePOSIX: public SerialInterface {
public:
	SerialInterfacePOSIX(const std::string& portName);
	virtual ~SerialInterfacePOSIX();

	virtual bool isOpen() const;

private:
	int openPort(const std::string& portName);

	virtual bool writeBytes(const unsigned char* data,
			unsigned int dataSizeInBytes);
	virtual bool readBytes(unsigned char* data, unsigned int dataSizeInBytes);

	int mFileDescriptor;
};

}

#endif /* POLSTROSERIALINTERFACEPOSIX_H_ */
