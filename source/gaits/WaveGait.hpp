/*
 * WaveGait.hpp
 *
 *  Created on: May 25, 2018
 *      Author: aron
 */

#ifndef GAITS_WAVEGAIT_HPP_
#define GAITS_WAVEGAIT_HPP_

#include <gaits/Gait.hpp>
#include "robot/Leg.hpp"
#include "robot/LegInterpolation.hpp"
#include <vector>

class WaveGait : public Gait{
public:
	WaveGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack);
	~WaveGait(){};
	float get_duty_cycle();
	uint16_t get_step_size();
	uint16_t get_step_height();
	void cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);

private:
	static constexpr float dutyCycle = 0.833333;
	static constexpr int cycleCount = 6;
	static constexpr uint16_t stepSize = 60; //mm
	static constexpr uint16_t stepHeight = 40;//mm
	std::vector<Leg*> legs;
	void set_all_legs_to_ground_position(int bodyHeight);
	void cycle_one_leg(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight, size_t cycleNumber);
	LegInterpolation generate_mid_via(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight, Leg* leg, size_t cycleNumber);
};



#endif /* GAITS_WAVEGAIT_HPP_ */
