/*
 * WaveGait.cpp
 *
 *  Created on: May 25, 2018
 *      Author: aron
 */

#include "WaveGait.hpp"
#include "robot/RobotConstants.hpp"
#include <stdint.h>

using namespace robot::basic;

WaveGait::WaveGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack):Gait(){
	this->legs.push_back(rFront);
	this->legs.push_back(rMiddle);
	this->legs.push_back(rBack);
	this->legs.push_back(lFront);
	this->legs.push_back(lMiddle);
	this->legs.push_back(lBack);
}

float WaveGait::get_duty_cycle(){
	return this->dutyCycle;
}

uint16_t WaveGait::get_step_size(){
	return this->stepSize;
}

uint16_t WaveGait::get_step_height(){
	return this->stepHeight;
}

void WaveGait::cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	this->set_all_legs_to_ground_position(bodyHeight);
	for (size_t legNumber = 0; legNumber < this->legs.size(); legNumber++){
		this->cycle_one_leg(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legNumber);
	}
}

void WaveGait::set_all_legs_to_ground_position(int bodyHeight){
	for(auto leg : this->legs){
		leg->update_endeffector_coords(leg->get_x(), leg->get_y(), -bodyHeight);
		leg->solve_ik();
		uint16_t coxa = leg->get_coxa_angle();
		uint16_t femur = leg->get_femur_angle();
		uint16_t tibia = leg->get_tibia_angle();
		leg->set_angles(coxa, femur, tibia);
	}
}

void WaveGait::cycle_one_leg(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight, size_t cycleNumber){
	LegInterpolation rightFront = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(0), cycleNumber);
	LegInterpolation rightMiddle = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(1), cycleNumber);
	LegInterpolation rightBack = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(2), cycleNumber);
	LegInterpolation leftFront = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(3), cycleNumber);
	LegInterpolation leftMiddle = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(4), cycleNumber);
	LegInterpolation leftBack = generate_mid_via(newXRight, newYRight, newXLeft, newYLeft, bodyHeight, legs.at(5), cycleNumber);

	int singleCycleTime = trajectoryIncrements/2;
	float midTime = singleCycleTime / 2.0;
	for(int i=0; i<singleCycleTime; i++){
		rightFront.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);
		rightMiddle.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);
		rightBack.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);
		leftFront.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);
		leftMiddle.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);
		leftBack.generate_current_angle(0, midTime, singleCycleTime, i, nullptr);

		rightFront.set_current_angles();
		rightMiddle.set_current_angles();
		rightBack.set_current_angles();
		leftFront.set_current_angles();
		leftMiddle.set_current_angles();
		leftBack.set_current_angles();
	}
}


LegInterpolation WaveGait::generate_mid_via(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight, Leg* leg, size_t cycleNumber){
	bool isLeft = (leg->get_position() == LegPosition::LEFT_BACK ||
			leg->get_position() == LegPosition::LEFT_FRONT ||
			leg->get_position() == LegPosition::LEFT_MIDDLE);
	LegInterpolation legInterpolation = LegInterpolation(leg);
	if(leg == this->legs.at(cycleNumber)){ //chosen leg for forward swing
		if(isLeft){
			legInterpolation.generate_mid_via_and_end(newXLeft, newYLeft, this->stepHeight, -bodyHeight);
		}else{
			legInterpolation.generate_mid_via_and_end(newXRight, newYRight, this->stepHeight, -bodyHeight);
		}
	}else{ //all other non chosen legs
		if(isLeft){
			int leftWaveX = newXLeft / this->cycleCount; //eventuell weg
			int leftWaveY = newYLeft / this->cycleCount;
			legInterpolation.generate_mid_via_and_end(-leftWaveX, -leftWaveY, 0, -bodyHeight);
		}else{
			int rightWaveX = newXRight / this->cycleCount;
			int rightWaveY = newYRight / this->cycleCount;
			legInterpolation.generate_mid_via_and_end(-rightWaveX, -rightWaveY, 0, -bodyHeight);
		}
	}
	legInterpolation.generate_trajectory_angles();
	return legInterpolation;
}
