/*
 * leg.hpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */

#ifndef ROBOT_LEG_HPP_
#define ROBOT_LEG_HPP_

#include <stdint.h>
#include "dynamixel/DynamixelControl.hpp"
#include "Motor.hpp"
#include "arm_math.h"
#include <vector>

class Leg {
public:
	Leg(uint8_t coxaId, uint8_t femurId, uint8_t tibiaId, DynamixelControl &dynamixelControl, LegPosition position);
	virtual ~Leg(){};
	float32_t get_body_to_leg_rotation();
	float32_t get_body_to_leg_x_distance();
	float32_t get_body_to_leg_y_distance();

	void solve_ik();
	/*!
	 * @brief  solves the sixth order polynomial for the leg trajectory one angle at a time
	 *
	 * @param  q0		start angle
	 * @param  qm		middle angle
	 * @param  q1		final angle
	 * @param  t0		start time
	 * @param  tm		middle time
	 * @param  t1		end time
	 * @param  tc		current time
	 * @param  vd		pointer for output of velocity at given time, nullptr if not needed.
	 *
	 * @return          angle at given time
	 */
	int16_t solve_trajectory(int16_t q0, int16_t qm, int16_t q1, float32_t t0, float32_t tm, float32_t t1, float32_t tc, int16_t* vd);
	std::vector<int16_t> solve_trajectory_vec(int16_t q0, int16_t qm, int16_t q1, float32_t t0, float32_t tm, float32_t t1);
	void set_speeds(uint16_t coxaSpeed, uint16_t femurSpeed, uint16_t tibiaSpeed);
	void set_angles(uint16_t coxaAngle, uint16_t femurAngle, uint16_t tibiaAngle);
	void update_endeffector_coords(int16_t x, int16_t y, int16_t z); //new endeffe
	int16_t get_x();
	int16_t get_y();
	int16_t get_z();
	uint16_t get_coxa_angle();
	uint16_t get_femur_angle();
	uint16_t get_tibia_angle();
	LegPosition get_position();
	LegPhase get_leg_phase();
	void update_leg_phase(LegPhase phase);
	void reset_usage();
	void increment_usage();
	int get_usage();

	int currentSwingIncrement;
	int currentStanceIncrement;

private:
	LegPosition position;
	LegPhase phase;
	Motor *coxaMotor;
	Motor *tibiaMotor;
	Motor *femurMotor;
	uint16_t lastCoxaAngle;
	uint16_t lastTibiaAngle;
	uint16_t lastFemurAngle;
	uint16_t coxaAngle;
	uint16_t tibiaAngle;
	uint16_t femurAngle;
	int16_t x; //from coxa to endeffector
	int16_t y; //from coxa to endeffector
	int16_t z; //from coxa to endeffector
	float32_t bodyToLegRotationAngle;
	float32_t bodyToLegXDistance;
	float32_t bodyToLegYDistance;
	float32_t tempCoxaAngle;

	int usage;
	void assign_position_dependend_parameters(LegPosition position);
	void range_restrict(uint16_t& coxaAngle, uint16_t& femurAngle, uint16_t& tibiaAngle);
	void apply_coxa_angle_offset(int16_t& coxaAngle);
	void apply_femur_angle_offset(int16_t& femurAngle);
	void apply_tibia_angle_offset(int16_t& tibiaAngle);
	void check_arm_math_status(arm_status status);
	uint16_t correct_cast(int16_t number);
};


#endif /* ROBOT_LEG_HPP_ */
