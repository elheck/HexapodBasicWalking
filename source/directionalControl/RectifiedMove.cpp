/*
 * RectifiedMove.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: aron
 */

#include "RectifiedMove.hpp"

RectifiedMove::RectifiedMove(){
	this->leftX = 0.0;
	this->leftY = 0.0;
	this->rightX = 0.0;
	this->rightY = 0.0;
	this->gait = WalkingGait::NONE;
	this->mode = Mode::WALKING;
}

RectifiedMove::RectifiedMove(const RectifiedMove& move){
	this->leftX = move.leftX;
	this->leftY = move.leftY;
	this->rightX = move.rightX;
	this->rightY = move.rightY;
	this->gait = move.gait;
	this->mode = move.mode;
}

RectifiedMove::RectifiedMove(float leftX, float leftY, float rightX, float rightY, WalkingGait gait, Mode mode){
	this->leftX = leftX;
	this->leftY = leftY;
	this->rightX = rightX;
	this->rightY = rightY;
	this->gait = gait;
	this->mode = mode;
}

float& RectifiedMove::get_leftX(){
	return this->leftX;
}

float& RectifiedMove::get_leftY(){
	return this->leftY;
}

float& RectifiedMove::get_rightX(){
	return this->rightX;
}

float& RectifiedMove::get_rightY(){
	return this->rightY;
}

Mode& RectifiedMove::get_mode(){
	return this->mode;
}

WalkingGait& RectifiedMove::get_walking_gait(){
	return this->gait;
}

void RectifiedMove::set_leftX(float leftX){
	this->leftX = leftX;
}

void RectifiedMove::set_leftY(float leftY){
	this->leftY = leftY;
}

void RectifiedMove::set_rightX(float rightX){
	this->rightX = rightX;
}

void RectifiedMove::set_rightY(float rightY){
	this->rightY = rightY;
}

void RectifiedMove::set_mode(Mode& mode){
	this->mode = mode;
}

void RectifiedMove::set_gait(WalkingGait& gait){
	this->gait = gait;
}
