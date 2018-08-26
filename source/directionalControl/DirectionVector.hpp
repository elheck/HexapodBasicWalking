/*
 * DirectionVector.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_DIRECTIONVECTOR_HPP_
#define DIRECTIONALCONTROL_DIRECTIONVECTOR_HPP_

#include <stdint.h>
#include "Modes.hpp"

class DirectionVector {
public:
	DirectionVector();
	DirectionVector(uint16_t leftX, uint16_t leftY, uint16_t rightX, uint16_t rightY, WalkingGait gait, Mode mode);
	virtual ~DirectionVector(){};
	uint16_t& get_leftX();
	uint16_t& get_leftY();
	uint16_t& get_rightX();
	uint16_t& get_rightY();
	Mode& get_mode();
	WalkingGait& get_walking_gait();
	void set_leftX(uint16_t leftX);
	void set_leftY(uint16_t leftY);
	void set_rightX(uint16_t rightX);
	void set_rightY(uint16_t rightY);
	void set_mode(Mode& mode);
	void set_gait(WalkingGait& gait);

private:
	uint16_t leftX;
	uint16_t leftY;
	uint16_t rightX;
	uint16_t rightY;
	Mode mode;
	WalkingGait gait;
};



#endif /* DIRECTIONALCONTROL_DIRECTIONVECTOR_HPP_ */
