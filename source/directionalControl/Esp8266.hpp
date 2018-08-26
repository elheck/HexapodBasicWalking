/*
 * esp8266.hpp
 *
 *  Created on: Jan 11, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_ESP8266_HPP_
#define DIRECTIONALCONTROL_ESP8266_HPP_

#include <stdint.h>
#include "DirectionVector.hpp"

#define TEMPORARY_BUFFER_SIZE 254

class Esp8266 {
public:
	Esp8266(uint32_t baudrate);
	virtual ~ Esp8266(){};
	DirectionVector get_vector();

private:
	WalkingGait get_current_gait(uint8_t& lowByte);
	Mode get_current_mode(uint8_t& highByte);
	DirectionVector safety_check(DirectionVector vec);

	uint8_t data[TEMPORARY_BUFFER_SIZE];
	uint32_t receivedLength;
	DirectionVector bufferVector;
};



#endif /* DIRECTIONALCONTROL_ESP8266_HPP_ */
