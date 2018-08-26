/*
 * TripodGait.hpp
 *
 *  Created on: Mar 14, 2018
 *      Author: aron
 */

#ifndef GAITS_TRIPODGAIT_HPP_
#define GAITS_TRIPODGAIT_HPP_

#include <gaits/Gait.hpp>
#include "robot/Leg.hpp"
#include <stdint.h>

class TripodGait: public Gait{
public:
	TripodGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack);
	~TripodGait(){};
	float get_duty_cycle();
	uint16_t get_step_size();
	uint16_t get_step_height();
	void cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);

private:
	uint8_t numCycles;
	static constexpr float dutyCycle = 0.5;
	static constexpr uint16_t stepSize = 30; //mm
	static constexpr uint16_t stepHeight = 30;//mm

	Leg *leftFront;
	Leg *leftMiddle;
	Leg *leftBack;
	Leg *rightFront;
	Leg *rightMiddle;
	Leg *rightBack;

	void left_advance(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);
	void left_advance_leg_status_update();
	void right_advance(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);
	void right_advance_leg_status_update();
};


#endif /* GAITS_TRIPODGAIT_HPP_ */
