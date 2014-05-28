/*
 * PolstroSerialInterface.h
 *
 *  Created on: 10.10.2013
 *      Author: tuuzdu
 */

#ifndef POLSTROSERIALINTERFACE_H_
#define POLSTROSERIALINTERFACE_H_

#pragma once

#include <string>

namespace Polstro {

/*
 The SerialInterface is the object used to communicate with one or more Maestro devices.
 At creation time, the interface opens the specified port at the given baud rate.
 If succesfull, it is possible after that to issue commands to the device(s) on that port.
 Devices can be chained one to the other. The chain is connected to a single port.

 For detailled explanations about the commands, see the Pololu documentation
 http://www.pololu.com/docs/0J40/5.e
 */
class SerialInterface {
public:
	SerialInterface(const std::string& portName);
	virtual ~SerialInterface();

	virtual bool isOpen() const = 0;

	static unsigned int getMinChannelValue() {
		return mMinChannelValue;
	}
	static unsigned int getMaxChannelValue() {
		return mMaxChannelValue;
	}

	// The target is given in units of 0.25µs
	bool setTarget(unsigned char channelNumber, unsigned short target);

	// The normalizedTarget is between 0 and 255 and the actual value it represents in µs depends
	// on the neutral and range configured for the channel of the device (and stored on the device).
	// This allow calibration to be done externally once (using the Maestro Control Center for example)
	// and have the application manipulate values that are "good".
	// The miniSCC channel number allows access to channels of chained devices (see doc)
	bool setTargetMSS(unsigned char miniSCCChannelNumber,
			unsigned char normalizedTarget);

	// On Mini Maestro 12, 18 and 24 only, so not supported here
	// bool setMultipleTargets(...)

	// The speed limit is given in units of (0.25µs)/(10ms)
	bool setSpeed(unsigned char channelNumber, unsigned short speed);

	// The acceleration limit is a value from 0 to 255 in units of (0.25µs)/(10ms)/(80ms)
	bool setAcceleration(unsigned char channelNumber,
			unsigned char acceleration);

	// For a servo channel, the position is the current pulse width in units of 0.25µs
	// For a digital output channel, a position less than 6000 means the line is low, and above 6000 it's high
	// For an input channel, the position represents the voltage measure on the channel (see doc)
	bool getPosition(unsigned char channelNumber, unsigned short& position);

	bool getMovingState(bool& servosAreMoving);

	bool getErrors(unsigned short& error);

	// The "go home" action set the channels to their startup/error state.
	// This state is defined on a per-channel. It can either be:
	// - ignore: the value is unchanged when we do a "go home". PWM signal is continues to be generated
	// - go to: the channel is set to the specified value. Again PWM signal is generated
	// - off: the channel is turned off. There's no more PWM signal generated for the channel
	bool goHome();

	static SerialInterface* createSerialInterface(const std::string& portName,
			unsigned int baudRate);

private:
	static const unsigned int mMinChannelValue = 2000;   	// Change tuuzdu
	static const unsigned int mMaxChannelValue = 10000;		// Change tuuzdu

	virtual bool writeBytes(const unsigned char* data,
			unsigned int dataSizeInBytes) = 0;
	virtual bool readBytes(unsigned char* data,
			unsigned int dataSizeInBytes) = 0;
};

}

#endif /* POLSTROSERIALINTERFACE_H_ */
