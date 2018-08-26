/*
 * Handheld.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: aron
 */

#include "Handheld.hpp"
#include <vector>

#define CALIBRATION_ITERATIONS 10

Handheld::Handheld(uint32_t baudRate) : esp(baudRate){
	this->lxOffset = 498;
	this->lyOffset = 514;
	this->rxOffset = 519;
	this->ryOffset = 200;
}

void Handheld::calibrate_offsets(){
	int i = 0;
	this->lxOffset = 0;
	this->lyOffset = 0;
	this->rxOffset = 0;
	this->ryOffset = 0;
	std::vector<DirectionVector> currentVector;
	while(i < CALIBRATION_ITERATIONS){
		DirectionVector currentRawData = this->esp.get_vector();
		currentVector.push_back(currentRawData);
		if(currentVector.at(i).get_walking_gait() != WalkingGait::NONE){
			this->lxOffset += (currentVector.at(i).get_leftX()) / i;
			this->lyOffset += (currentVector.at(i).get_leftY()) / i;
			this->rxOffset += (currentVector.at(i).get_rightX()) / i;
			this->ryOffset += (currentVector.at(i).get_rightY()) / i;
			i++;
		}
	}
}

RectifiedMove Handheld::get_rectified_movement(){
	RectifiedMove move = RectifiedMove();
	DirectionVector currentRawData = this->esp.get_vector();
	if(currentRawData.get_walking_gait() != WalkingGait::NONE){
		move.set_leftX(currentRawData.get_leftX() / float(2 * this->lxOffset));
		move.set_leftY(currentRawData.get_leftY() / float(2 * this->lyOffset));
		move.set_rightX(currentRawData.get_rightX() / float(2 * this->rxOffset));
		move.set_rightY(currentRawData.get_rightY() / float(2 * this->ryOffset));
		move.set_mode(currentRawData.get_mode());
		move.set_gait(currentRawData.get_walking_gait());
	}
	return move;
}

