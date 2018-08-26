/*
 * RuleBasedGait.hpp
 *
 *  Created on: May 26, 2018
 *      Author: aron
 */

#ifndef GAITS_RULEBASEDGAIT_HPP_
#define GAITS_RULEBASEDGAIT_HPP_

#include "Gait.hpp"
#include "robot/Leg.hpp"
#include "robot/LegInterpolation.hpp"
#include <vector>

//TODO implement progress increments to end while loop

class RuleBasedGait: public Gait {
public:
	RuleBasedGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack);


	float get_duty_cycle();
	uint16_t get_step_size();
	uint16_t get_step_height();
	void cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);


private:
	bool body_is_at_desired_position(int x, int y);
	void reset_recent_usage();
	void cycle_legs(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);
	LegInterpolation get_leg_interpolation_situation(Leg* leg, int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight);
	Leg* get_anterior_leg(Leg* currentLeg);
	Leg* get_posterior_leg(Leg* currentLeg);
	Leg* get_leg_with_position(LegPosition position);
	float get_anterior_euclidean_distance_restrictedness(Leg* leg);
	float get_anterior_current_x_difference(Leg* leg, Leg* anteriorLeg);
	float get_anterior_current_y_difference(Leg* leg, Leg* anteriorLeg);
	float get_anterior_constant_x_distance(LegPosition position);
	float get_anterior_constant_y_distance(LegPosition position);
	float get_posterior_euclidean_distance_restrictedness(Leg* leg);
	float get_posterior_current_x_difference(Leg* leg, Leg* anteriorLeg);
	float get_posterior_current_y_difference(Leg* leg, Leg* anteriorLeg);
	float get_posterior_constant_x_distance(LegPosition position);
	float get_posterior_constant_y_distance(LegPosition position);
	float solve_parameter_restrictedness(float minValue, float maxValue, float currentValue);


	static constexpr int distanceIncrement = 20;//mm
	static constexpr float incrementPerStep = 0.1666;
	static constexpr int incrementSteps = 6;
	static constexpr int emergencyCycleKill = 6;
	static constexpr float dutyCycle = 0.0;
	static constexpr uint16_t stepSize = 60; //mm
	static constexpr uint16_t stepHeight = 40;//mm
	static constexpr float frontXDistance = 120.0;
	static constexpr float ipsilateralXDistance = 40.0;
	static constexpr float frontYDistance = 0.0;
	static constexpr float ipsilateralYDistance = 120.0;
	static constexpr float smoothingFactor = 0.1;
	static constexpr float minEuclideanDistance = 10;
	static constexpr float maxEuclideanDistance = 140;
	static constexpr float maxEuclideanRestrictednessThreshold = 0.8;


	std::vector<Leg*> legs;
	int initialBodyX;
	int initialBodyY;
	int currentBodyX;
	int currentBodyY;


};



#endif /* GAITS_RULEBASEDGAIT_HPP_ */
