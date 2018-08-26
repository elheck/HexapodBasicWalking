/*
 * TripodGait.cpp
 *
 *  Created on: Mar 14, 2018
 *      Author: aron
 */
#include "TripodGait.hpp"
#include "robot/LegInterpolation.hpp"
#include "robot/RobotConstants.hpp"

using namespace robot::basic;

TripodGait::TripodGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack):Gait(){
	this->numCycles = uint8_t(1.0 / this->dutyCycle);
	this->leftFront = lFront;
	this->leftMiddle = lMiddle;
	this->leftBack = lBack;
	this->rightFront = rFront;
	this->rightMiddle = rMiddle;
	this->rightBack = rBack;
}

float TripodGait::get_duty_cycle(){
	return this->dutyCycle;
}

uint16_t TripodGait::get_step_size(){
	return this->stepSize;
}

uint16_t TripodGait::get_step_height(){
	return this->stepHeight;
}

void TripodGait::cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	if(this->leftFront->get_leg_phase() == LegPhase::SWING &&
			this->leftMiddle->get_leg_phase() == LegPhase::SUPPORT &&
			this->leftBack->get_leg_phase() == LegPhase::SWING){
		this->left_advance(newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
		this->right_advance(newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	}else{
		this->right_advance(newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
		this->left_advance(newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	}
}

void TripodGait::left_advance(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	LegInterpolation leftFrontInterpolation = LegInterpolation(this->leftFront);
	LegInterpolation leftMiddleInterpolation = LegInterpolation(this->leftMiddle);
	LegInterpolation leftBackInterpolation = LegInterpolation(this->leftBack);
	LegInterpolation rightFrontInterpolation = LegInterpolation(this->rightFront);
	LegInterpolation rightMiddleInterpolation = LegInterpolation(this->rightMiddle);
	LegInterpolation rightBackInterpolation = LegInterpolation(this->rightBack);

	leftFrontInterpolation.generate_mid_via_and_end(newXLeft, newYLeft, this->stepHeight, -bodyHeight);
	leftMiddleInterpolation.generate_mid_via_and_end(-newXLeft, -newYLeft, 0, -bodyHeight);
	leftBackInterpolation.generate_mid_via_and_end(newXLeft, newYLeft, this->stepHeight, -bodyHeight);

	rightFrontInterpolation.generate_mid_via_and_end(-newXRight, -newYRight, 0, -bodyHeight);
	rightMiddleInterpolation.generate_mid_via_and_end(newXRight, newYRight, this->stepHeight, -bodyHeight);
	rightBackInterpolation.generate_mid_via_and_end(-newXRight, -newYRight, 0, -bodyHeight);

	leftFrontInterpolation.generate_trajectory_angles();
	leftMiddleInterpolation.generate_trajectory_angles();
	leftBackInterpolation.generate_trajectory_angles();

	rightFrontInterpolation.generate_trajectory_angles();
	rightMiddleInterpolation.generate_trajectory_angles();
	rightBackInterpolation.generate_trajectory_angles();

	float midTime = trajectoryIncrements/2.0;

	for(int i=0; i<trajectoryIncrements; i++){
		leftFrontInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		leftMiddleInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		leftBackInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);

		rightFrontInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		rightMiddleInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		rightBackInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);

		leftFrontInterpolation.set_current_angles();
		leftMiddleInterpolation.set_current_angles();
		leftBackInterpolation.set_current_angles();

		rightFrontInterpolation.set_current_angles();
		rightMiddleInterpolation.set_current_angles();
		rightBackInterpolation.set_current_angles();
	}
	this->left_advance_leg_status_update();
}

void TripodGait::left_advance_leg_status_update(){
	this->leftFront->update_leg_phase(LegPhase::SUPPORT);
	this->leftMiddle->update_leg_phase(LegPhase::SWING);
	this->leftBack->update_leg_phase(LegPhase::SUPPORT);
	this->rightFront->update_leg_phase(LegPhase::SWING);
	this->rightMiddle->update_leg_phase(LegPhase::SUPPORT);
	this->rightBack->update_leg_phase(LegPhase::SWING);
}

void TripodGait::right_advance(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	LegInterpolation leftFrontInterpolation = LegInterpolation(this->leftFront);
	LegInterpolation leftMiddleInterpolation = LegInterpolation(this->leftMiddle);
	LegInterpolation leftBackInterpolation = LegInterpolation(this->leftBack);
	LegInterpolation rightFrontInterpolation = LegInterpolation(this->rightFront);
	LegInterpolation rightMiddleInterpolation = LegInterpolation(this->rightMiddle);
	LegInterpolation rightBackInterpolation = LegInterpolation(this->rightBack);

	rightFrontInterpolation.generate_mid_via_and_end(newXRight, newYRight, this->stepHeight, -bodyHeight);
	rightMiddleInterpolation.generate_mid_via_and_end(-newXRight, -newYRight, 0, -bodyHeight);
	rightBackInterpolation.generate_mid_via_and_end(newXRight, newYRight, this->stepHeight, -bodyHeight);

	leftFrontInterpolation.generate_mid_via_and_end(-newXLeft, -newYLeft, 0, -bodyHeight);
	leftMiddleInterpolation.generate_mid_via_and_end(newXLeft, newYLeft, this->stepHeight, -bodyHeight);
	leftBackInterpolation.generate_mid_via_and_end(-newXLeft, -newYLeft, 0, -bodyHeight);

	leftFrontInterpolation.generate_trajectory_angles();
	leftMiddleInterpolation.generate_trajectory_angles();
	leftBackInterpolation.generate_trajectory_angles();

	rightFrontInterpolation.generate_trajectory_angles();
	rightMiddleInterpolation.generate_trajectory_angles();
	rightBackInterpolation.generate_trajectory_angles();

	float midTime = trajectoryIncrements/2.0;

	for(int i=0; i<trajectoryIncrements; i++){
		leftFrontInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		leftMiddleInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		leftBackInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);

		rightFrontInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		rightMiddleInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);
		rightBackInterpolation.generate_current_angle(0, midTime, trajectoryIncrements, i, nullptr);

		leftFrontInterpolation.set_current_angles();
		leftMiddleInterpolation.set_current_angles();
		leftBackInterpolation.set_current_angles();

		rightFrontInterpolation.set_current_angles();
		rightMiddleInterpolation.set_current_angles();
		rightBackInterpolation.set_current_angles();
	}
	this->right_advance_leg_status_update();
}

void TripodGait::right_advance_leg_status_update(){
	this->leftFront->update_leg_phase(LegPhase::SWING);
	this->leftMiddle->update_leg_phase(LegPhase::SUPPORT);
	this->leftBack->update_leg_phase(LegPhase::SWING);
	this->rightFront->update_leg_phase(LegPhase::SUPPORT);
	this->rightMiddle->update_leg_phase(LegPhase::SWING);
	this->rightBack->update_leg_phase(LegPhase::SUPPORT);
}
