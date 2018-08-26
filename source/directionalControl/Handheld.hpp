/*
 * Handheld.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_HANDHELD_HPP_
#define DIRECTIONALCONTROL_HANDHELD_HPP_

#include <stdint.h>
#include "Esp8266.hpp"
#include "RectifiedMove.hpp"

class Handheld {
public:
	Handheld(uint32_t baudRate);
	virtual ~Handheld(){};
	void calibrate_offsets();

	/*!
	 * @brief  gets the raw data and rectifies it to be between 0 and 1
	 *
	 * @return          RectifiedMove with coordinates between 0 and 1
	 */
	RectifiedMove get_rectified_movement();

private:
	Esp8266 esp;
	uint16_t lxOffset;
	uint16_t lyOffset;
	uint16_t rxOffset;
	uint16_t ryOffset;
};



#endif /* DIRECTIONALCONTROL_HANDHELD_HPP_ */
