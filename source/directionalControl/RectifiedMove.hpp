/*
 * RectifiedMove.hpp
 *
 *  Created on: Jan 30, 2018
 *      Author: aron
 */

#ifndef DIRECTIONALCONTROL_RECTIFIEDMOVE_HPP_
#define DIRECTIONALCONTROL_RECTIFIEDMOVE_HPP_

#include "Modes.hpp"

class RectifiedMove {
public:
	RectifiedMove();
	RectifiedMove(const RectifiedMove& move);
	RectifiedMove(float leftX, float leftY, float rightX, float rightY, WalkingGait gait, Mode mode);
	virtual ~RectifiedMove(){};
	float& get_leftX();
	float& get_leftY();
	float& get_rightX();
	float& get_rightY();
	Mode& get_mode();
	WalkingGait& get_walking_gait();
	void set_leftX(float leftX);
	void set_leftY(float leftY);
	void set_rightX(float rightX);
	void set_rightY(float rightY);
	void set_mode(Mode& mode);
	void set_gait(WalkingGait& gait);

private:
	float leftX;
	float leftY;
	float rightX;
	float rightY;
	Mode mode;
	WalkingGait gait;
};



#endif /* DIRECTIONALCONTROL_RECTIFIEDMOVE_HPP_ */
