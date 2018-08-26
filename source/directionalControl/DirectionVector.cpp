/*
 * DirectionVector.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: aron
 */

#include "DirectionVector.hpp"

DirectionVector::DirectionVector(){
	this->leftX = 0;
	this->leftY = 0;
	this->rightX = 0;
	this->rightY = 0;
	this->gait = WalkingGait::TRIPOD;
	this->mode = Mode::WALKING;
}

DirectionVector::DirectionVector(uint16_t leftX, uint16_t leftY, uint16_t rightX, uint16_t rightY, WalkingGait gait, Mode mode){
	this->leftX = leftX;
	this->leftY = leftY;
	this->rightX = rightX;
	this->rightY = rightY;
	this->gait = gait;
	this->mode = mode;
}

uint16_t& DirectionVector::get_leftX(){
	return this->leftX;
}

uint16_t& DirectionVector::get_leftY(){
	return this->leftY;
}

uint16_t& DirectionVector::get_rightX(){
	return this->rightX;
}

uint16_t& DirectionVector::get_rightY(){
	return this->rightY;
}

Mode& DirectionVector::get_mode(){
	return this->mode;
}

WalkingGait& DirectionVector::get_walking_gait(){
	return this->gait;
}

void DirectionVector::set_leftX(uint16_t leftX){
	this->leftX = leftX;
}

void DirectionVector::set_leftY(uint16_t leftY){
	this->leftY = leftY;
}

void DirectionVector::set_rightX(uint16_t rightX){
	this->rightX = rightX;
}

void DirectionVector::set_rightY(uint16_t rightY){
	this->rightY = rightY;
}

void DirectionVector::set_mode(Mode& mode){
	this->mode = mode;
}

void DirectionVector::set_gait(WalkingGait& gait){
	this->gait = gait;
}
