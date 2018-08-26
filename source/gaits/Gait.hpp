/*
 * WalkingGait.hpp
 *
 *  Created on: Mar 14, 2018
 *      Author: aron
 */

#ifndef GAITS_GAIT_HPP_
#define GAITS_GAIT_HPP_

#include <stdint.h>

class Gait {
public:
	Gait(){};
	virtual ~Gait(){};
	virtual float get_duty_cycle()=0;
	virtual uint16_t get_step_size()=0;
	virtual uint16_t get_step_height()=0;
	virtual void cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight)=0;

private:
};



#endif /* GAITS_GAIT_HPP_ */
