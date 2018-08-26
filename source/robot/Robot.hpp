/*
 * Robot.hpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */

#ifndef ROBOT_ROBOT_HPP_
#define ROBOT_ROBOT_HPP_

#include "dynamixel/DynamixelControl.hpp"
#include "gaits/TripodGait.hpp"
#include "gaits/WaveGait.hpp"
#include "gaits/RuleBasedGait.hpp"
#include "directionalControl/Modes.hpp"
#include "Leg.hpp"
#include <utility>


class Robot {
public:
	Robot(uint32_t baudRate);
	~Robot();
	void move_to_start_position();
	void move_single_leg_to(int x, int y, int z, Leg *leg);
	void move_all_legs_to(int x, int y, int z);
	void test();
	/*!
	 * @brief  moves the robot
	 *
	 * @param  translationX		translation in X direction of robot frame. Between -1 and 1
	 * @param  translationY		translation in Y direction of robot frame. Between -1 and 1
	 * @param  rotationZ		rotation in Z direction of robot frame. Between -1 and 1
	 * @param  gait				Walking gait enum to symbolize how to move
	 */
	void move(float translationX, float translationY, float rotationZ, WalkingGait gait, Mode mode);



private:
	int bodyHeight;
	DynamixelControl *dynamixelControl;
	Leg *leftFront;
	Leg *leftMiddle;
	Leg *leftBack;
	Leg *rightFront;
	Leg *rightMiddle;
	Leg *rightBack;
	static constexpr float maxPosTranslation = 1.0;
	static constexpr float maxNegTranslation = -1.0;
	static constexpr float maxPosRotation = 1.0;
	static constexpr float maxNegRotation = -1.0;
	static constexpr float zeroNoiseFilterConstant = 0.1;
	WaveGait *waveGait;
	TripodGait *tripodGait;
	RuleBasedGait *ruleBased;

	void filter_zero_noise(float& translationX, float& translationY, float& rotationZ);
	void initiate_gait_phases();
	void reset_leg_positions();
	void reset_single_leg(Leg *leg);
	void rotate(int& xRight, int& yRight, int& xLeft, int& yLeft, const float rotationZ);
	void cycle_gait(float translationX, float translationY, float rotationZ, WalkingGait gait);
};


#endif /* ROBOT_ROBOT_HPP_ */
