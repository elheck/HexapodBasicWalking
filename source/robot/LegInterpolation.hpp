/*
 * LegInterpolation.hpp
 *
 *  Created on: Mar 31, 2018
 *      Author: aron
 */

#ifndef ROBOT_LEGINTERPOLATION_HPP_
#define ROBOT_LEGINTERPOLATION_HPP_

#include <stdint.h>
#include <vector>
#include "Leg.hpp"


class LegInterpolation {
public:
	LegInterpolation(Leg *leg);
	void generate_mid_via_and_end(int xAdded, int yAdded, int midZAdded, int endZ);
	void generate_trajectory_angles();
	void generate_current_angle(int t0, int tm, int t1, int tc, int16_t *vd);
	void generate_current_angle(float t);
	std::vector<int16_t> generate_trajectory_vector_coxa(int t0, int tm, int t1);
	std::vector<int16_t> generate_trajectory_vector_femur(int t0, int tm, int t1);
	std::vector<int16_t> generate_trajectory_vector_tibia(int t0, int tm, int t1);
	void set_current_angles();
	void set_current_angles(uint16_t coxa, uint16_t femur, uint16_t tibia);
	int viaX;
	int viaY;
	int viaZ;
	int endX;
	int endY;
	int endZ;
	int16_t startCoxaAngle;
	int16_t startFemurAngle;
	int16_t startTibiaAngle;
	int16_t viaCoxaAngle;
	int16_t viaFemurAngle;
	int16_t viaTibiaAngle;
	int16_t endCoxaAngle;
	int16_t endFemurAngle;
	int16_t endTibiaAngle;
	uint16_t currentCoxaAngle;
	uint16_t currentFemurAngle;
	uint16_t currentTibiaAngle;

private:
	float solve_linear_interpol(float q0, float q1, float t); //t is between 0 and 1
	void restrict_x(int& x);
	void restrict_y(int& y);
	Leg *leg;

};




#endif /* ROBOT_LEGINTERPOLATION_HPP_ */
