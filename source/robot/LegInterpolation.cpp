/*
 * LegInterpolation.cpp
 *
 *  Created on: Mar 31, 2018
 *      Author: aron
 */
#include <robot/LegInterpolation.hpp>
#include "RobotConstants.hpp"


LegInterpolation::LegInterpolation(Leg *leg){
	this->leg = leg;
	this->viaX = 0;
	this->viaY = 0;
	this->viaZ = 0;
	this->endX = 0;
	this->endY = 0;
	this->endZ = 0;
	this->startCoxaAngle = 0;
	this->startFemurAngle = 0;
	this->startTibiaAngle = 0;
	this->viaCoxaAngle = 0;
	this->viaFemurAngle = 0;
	this->viaTibiaAngle = 0;
	this->endCoxaAngle = 0;
	this->endFemurAngle = 0;
	this->endTibiaAngle = 0;
	this->currentCoxaAngle = 0;
	this->currentFemurAngle = 0;
	this->currentTibiaAngle = 0;
}

void LegInterpolation::generate_mid_via_and_end(int xAdded, int yAdded, int midZAdded, int endZ){
	if(this->leg->get_y() == robot::basic::startYGround - yAdded){
		this->viaY = this->leg->get_y() + yAdded;
		this->endY = this->leg->get_y() + (yAdded * 2);
	}else{
		this->viaY = this->leg->get_y() + (yAdded / 2.0);
		this->endY = this->leg->get_y() + yAdded;
	}
	if(this->leg->get_x() == robot::basic::startXGround - xAdded){
		this->viaX = this->leg->get_x() + xAdded;
		this->endX = this->leg->get_x() + (xAdded * 2);
	}else{
		this->viaX = this->leg->get_x() + (xAdded / 2.0);
		this->endX = this->leg->get_x() + xAdded;
	}
	this->restrict_x(this->viaX);
	this->restrict_x(this->endX);
	this->restrict_y(this->viaY);
	this->restrict_y(this->endY);
	this->viaZ = this->leg->get_z() + midZAdded;
	this->endZ = endZ;
}

void LegInterpolation::restrict_x(int& x){
	if(x < robot::basic::minX) x = robot::basic::minX;
	if(x > robot::basic::maxX) x = robot::basic::maxX;
}

void LegInterpolation::restrict_y(int& y){
	if(y < robot::basic::minY) y = robot::basic::minY;
	if(y > robot::basic::maxY) y = robot::basic::maxY;
}

void LegInterpolation::generate_trajectory_angles(){
	this->leg->solve_ik();
	this->startCoxaAngle = this->leg->get_coxa_angle();
	this->startFemurAngle = this->leg->get_femur_angle();
	this->startTibiaAngle = this->leg->get_tibia_angle();
	this->leg->update_endeffector_coords(this->viaX, this->viaY, this->viaZ);
	this->leg->solve_ik();
	this->viaCoxaAngle = this->leg->get_coxa_angle();
	this->viaFemurAngle = this->leg->get_femur_angle();
	this->viaTibiaAngle = this->leg->get_tibia_angle();
	this->leg->update_endeffector_coords(this->endX, this->endY, this->endZ);
	this->leg->solve_ik();
	this->endCoxaAngle = this->leg->get_coxa_angle();
	this->endFemurAngle = this->leg->get_femur_angle();
	this->endTibiaAngle = this->leg->get_tibia_angle();
}

void LegInterpolation::generate_current_angle(int t0, int tm, int t1, int tc, int16_t *vd){
	this->currentCoxaAngle = this->leg->solve_trajectory(this->startCoxaAngle, this->viaCoxaAngle, this->endCoxaAngle, t0, tm, t1, tc, vd);
	this->currentFemurAngle = this->leg->solve_trajectory(this->startFemurAngle, this->viaFemurAngle, this->endFemurAngle, t0, tm, t1, tc, vd);
	this->currentTibiaAngle = this->leg->solve_trajectory(this->startTibiaAngle, this->viaTibiaAngle, this->endTibiaAngle, t0, tm, t1, tc, vd);
}

void LegInterpolation::generate_current_angle(float t){
	this->currentCoxaAngle = uint16_t(this->solve_linear_interpol(startCoxaAngle, endCoxaAngle, t));
	this->currentFemurAngle = uint16_t(this->solve_linear_interpol(startFemurAngle, endFemurAngle, t));
	this->currentTibiaAngle = uint16_t(this->solve_linear_interpol(startTibiaAngle, endTibiaAngle, t));
}

float LegInterpolation::solve_linear_interpol(float q0, float q1, float t){
	return q0 + t * (q1-q0);
}


std::vector<int16_t> LegInterpolation::generate_trajectory_vector_coxa(int t0, int tm, int t1){
	return this->leg->solve_trajectory_vec(this->startCoxaAngle, this->viaCoxaAngle, this->endCoxaAngle, t0, tm, t1);
}

std::vector<int16_t> LegInterpolation::generate_trajectory_vector_femur(int t0, int tm, int t1){
	return this->leg->solve_trajectory_vec(this->startFemurAngle, this->viaFemurAngle, this->endFemurAngle, t0, tm, t1);
}

std::vector<int16_t> LegInterpolation::generate_trajectory_vector_tibia(int t0, int tm, int t1){
	return this->leg->solve_trajectory_vec(this->startTibiaAngle, this->viaTibiaAngle, this->endTibiaAngle, t0, tm, t1);
}

void LegInterpolation::set_current_angles(){
	this->leg->set_angles(this->currentCoxaAngle, this->currentFemurAngle, this->currentTibiaAngle);
}

void LegInterpolation::set_current_angles(uint16_t coxa, uint16_t femur, uint16_t tibia){
	this->leg->set_angles(coxa, femur, tibia);
}
