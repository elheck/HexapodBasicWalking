/*
 * Leg.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */

#include "Leg.hpp"
#include "RobotConstants.hpp"
#include <math.h>
#include "onboard_led_control.h"

using namespace robot::body;
using namespace robot::motor;
using namespace robot::basic;

Leg::Leg(uint8_t coxaId, uint8_t femurId, uint8_t tibiaId, DynamixelControl &dynamixelControl, LegPosition position){
	this->coxaMotor = new Motor(coxaId, dynamixelControl, LegPart::COXA);
	this->femurMotor = new Motor(femurId, dynamixelControl, LegPart::FEMUR);
	this->tibiaMotor = new Motor(tibiaId, dynamixelControl, LegPart::TIBIA);
	this->lastCoxaAngle = 0;
	this->lastFemurAngle = 0;
	this->lastTibiaAngle = 0;
	this->x = startXGround;
	this->y = startYGround;
	this->z = startZGround;
	this->tempCoxaAngle = 0;
	this->position = position;
	this->phase = LegPhase::UNASSIGNED;
	this->assign_position_dependend_parameters(position);
	this->usage = 0;
	this->currentSwingIncrement = 0;
	this->currentStanceIncrement = 0;
}

void Leg::assign_position_dependend_parameters(LegPosition position){
	switch(position){
	case LegPosition::LEFT_FRONT:
		this->bodyToLegRotationAngle = leftFrontAngleOffset;
		this->bodyToLegXDistance = leftFrontXDistance;
		this->bodyToLegYDistance = leftFrontYDistance;
		break;
	case LegPosition::LEFT_MIDDLE:
		this->bodyToLegRotationAngle = leftMiddleAngleOffset;
		this->bodyToLegXDistance = leftMiddleXDistance;
		this->bodyToLegYDistance = leftMiddleYDistance;
		break;
	case LegPosition::LEFT_BACK:
		this->bodyToLegRotationAngle = leftBackAngleOffset;
		this->bodyToLegXDistance = leftBackXDistance;
		this->bodyToLegYDistance = leftBackYDistance;
		break;
	case LegPosition::RIGHT_FRONT:
		this->bodyToLegRotationAngle = rightFrontAngleOffset;
		this->bodyToLegXDistance = rightFrontXDistance;
		this->bodyToLegYDistance = rightFrontYDistance;
		break;
	case LegPosition::RIGHT_MIDDLE:
		this->bodyToLegRotationAngle = rightMiddleAngleOffset;
		this->bodyToLegXDistance = rightMiddleXDistance;
		this->bodyToLegYDistance = rightMiddleYDistance;
		break;
	case LegPosition::RIGHT_BACK:
		this->bodyToLegRotationAngle = rightBackAngleOffset;
		this->bodyToLegXDistance = rightBackXDistance;
		this->bodyToLegYDistance = rightBackYDistance;
		break;
	}
}

float32_t Leg::get_body_to_leg_rotation(){
	return this->bodyToLegRotationAngle;
}
float32_t Leg::get_body_to_leg_x_distance(){
	return this->bodyToLegXDistance;
}
float32_t Leg::get_body_to_leg_y_distance(){
	return this->bodyToLegYDistance;
}

void Leg::set_angles(uint16_t coxaAngle, uint16_t femurAngle, uint16_t tibiaAngle){
	this->range_restrict(coxaAngle, femurAngle, tibiaAngle);
	this->coxaMotor->set_angle(coxaAngle);
	this->femurMotor->set_angle(femurAngle);
	this->tibiaMotor->set_angle(tibiaAngle);
	this->lastCoxaAngle = coxaAngle;
	this->lastFemurAngle = femurAngle;
	this->lastTibiaAngle = tibiaAngle;
}

void Leg::range_restrict(uint16_t& coxaAngle, uint16_t& femurAngle, uint16_t& tibiaAngle){
	if(position == LegPosition::LEFT_BACK || position == LegPosition::LEFT_FRONT || position == LegPosition::LEFT_MIDDLE){
		if(coxaAngle > leftCoxaMaxAngle) coxaAngle = leftCoxaMaxAngle;
		if(femurAngle > leftFemurMaxAngle) femurAngle = leftFemurMaxAngle;
		if(tibiaAngle > leftTibiaMaxAngle) tibiaAngle = leftTibiaMaxAngle;
		if(coxaAngle < leftCoxaMinAngle) coxaAngle = leftCoxaMinAngle;
		if(femurAngle < leftFemurMinAngle) femurAngle = leftFemurMinAngle;
		if(tibiaAngle < leftTibiaMinAngle) tibiaAngle = leftTibiaMinAngle;
	}else if(position == LegPosition::RIGHT_BACK || position == LegPosition::RIGHT_FRONT || position == LegPosition::RIGHT_MIDDLE){
		if(coxaAngle > rightCoxaMaxAngle) coxaAngle = rightCoxaMaxAngle;
		if(femurAngle > rightFemurMaxAngle) femurAngle = rightFemurMaxAngle;
		if(tibiaAngle > rightTibiaMaxAngle) tibiaAngle = rightTibiaMaxAngle;
		if(coxaAngle < rightCoxaMinAngle) coxaAngle = rightCoxaMinAngle;
		if(femurAngle < rightFemurMinAngle) femurAngle = rightFemurMinAngle;
		if(tibiaAngle < rightTibiaMinAngle) tibiaAngle = rightTibiaMinAngle;
	}
}

void Leg::set_speeds(uint16_t coxaSpeed, uint16_t femurSpeed, uint16_t tibiaSpeed){
	this->coxaMotor->set_speed(coxaSpeed);
	this->femurMotor->set_speed(femurSpeed);
	this->tibiaMotor->set_speed(tibiaSpeed);
}

void Leg::solve_ik(){
	float xf = float(this->x);
	float yf = float(this->y);
	float zf = float(this->z);
	float coxaResult = atan2f(yf, xf) * (180.0/PI);
	int16_t coxa = int16_t(coxaResult);
	this->apply_coxa_angle_offset(coxa);
	this->coxaAngle = this->correct_cast(coxa);

	float strideLength;
	arm_sqrt_f32((powf(xf, 2) + powf(yf, 2)), &strideLength); //coxa to endeffektor
	float femurToEndBase = strideLength - coxaLength;
	float femurToEnd;
	arm_sqrt_f32((powf(zf, 2) + powf(femurToEndBase, 2)), &femurToEnd);
	float theta1Top = powf(tibiaLength, 2) - powf(femurLength, 2) - powf(femurToEnd, 2);
	float theta1Bottom = (-2) * femurLength * femurToEnd;
	float theta1 = acosf((theta1Top / theta1Bottom));
	theta1 *= (180/PI);
	float theta3 = atan2f(zf, femurToEndBase) * (180.0/PI); 
	int16_t femur = int16_t(theta1 + theta3);
	this->apply_femur_angle_offset(femur);
	this->femurAngle = this->correct_cast(femur);

	float theta2Top = powf(femurToEnd, 2) - powf(femurLength, 2) - powf(tibiaLength, 2);
	float theta2Bottom = (-2) * femurLength * tibiaLength;
	float theta2 = acosf(theta2Top / theta2Bottom);
	theta2 *= (180.0 / PI);
	int16_t tibia = int16_t(theta2);
	this->apply_tibia_angle_offset(tibia);
	this->tibiaAngle = this->correct_cast(tibia);
}

int16_t Leg::solve_trajectory(int16_t q0, int16_t qm, int16_t q1, float32_t t0, float32_t tm, float32_t t1, float32_t tc, int16_t* vd){
	arm_status status;
	float32_t qd;
	float32_t v;
	float32_t matrixInverseData[seventhOrderMatrixDimension]= {0};
	float32_t matrixData[seventhOrderMatrixDimension] = {
			1, t0, powf(t0, 2), powf(t0, 3), powf(t0, 4), powf(t0, 5), powf(t0, 6),
			0, 1, 2*t0, 3*powf(t0, 2), 4*powf(t0, 3), 5*powf(t0, 4), 6*powf(t0, 5),
			0, 0, 2, 6*t0, 12*powf(t0, 2), 20*powf(t0, 3), 30*powf(t0, 4),
			1, tm, powf(tm, 2), powf(tm, 3), powf(tm, 4), powf(tm, 5), powf(tm, 6),
			1, t1, powf(t1, 2), powf(t1, 3), powf(t1, 4), powf(t1, 5), powf(t1, 6),
			0, 1, 2*t1, 3*powf(t1, 2), 4*powf(t1, 3), 5*powf(t1, 4), 6*powf(t1, 5),
			0, 0, 2, 6*t1, 12*powf(t1, 2), 20*powf(t1, 3), 30*powf(t1, 4)
	};
	float32_t bData[seventhOrderVectorDimension] = {
			float32_t(q0), 0.0, 0.0, float32_t(qm), float32_t(q1), 0.0, 0.0
	};
	float32_t aData[seventhOrderVectorDimension] = {0};
	arm_matrix_instance_f32 matrix = {seventhOrderM, seventhOrderN, matrixData};
	arm_matrix_instance_f32 b = {seventhOrderVectorDimension, 1, bData};
	arm_matrix_instance_f32 matrixInverse = {seventhOrderM, seventhOrderN, matrixInverseData};
	arm_matrix_instance_f32 resultA = {seventhOrderVectorDimension, 1, aData};
	status = arm_mat_inverse_f32(&matrix, &matrixInverse);
	this->check_arm_math_status(status);
	status = arm_mat_mult_f32(&matrixInverse, &b, &resultA);
	this->check_arm_math_status(status);
	qd = resultA.pData[0] + resultA.pData[1]*tc + resultA.pData[2]*powf(tc,2) + resultA.pData[3]*powf(tc,3) + resultA.pData[4]*powf(tc,4) + resultA.pData[5]*powf(tc,5) + resultA.pData[6]*powf(tc,6);
	if(vd != nullptr){
		v = resultA.pData[1] + 2.0*resultA.pData[2]*tc + 3.0*resultA.pData[3]*powf(tc,2) + 4.0*resultA.pData[4]*powf(tc,3) + 5.0*resultA.pData[5]*powf(tc,4) + 6.0*resultA.pData[6]*powf(tc,5); + resultA.pData[3]*powf(tc,3);
		*vd = int16_t(v);
	}
	return int16_t(qd);
}

std::vector<int16_t> Leg::solve_trajectory_vec(int16_t q0, int16_t qm, int16_t q1, float32_t t0, float32_t tm, float32_t t1){
	std::vector<int16_t> q;
	arm_status status;
	float32_t matrixInverseData[seventhOrderMatrixDimension]= {0};
	float32_t matrixData[seventhOrderMatrixDimension] = {
			1, t0, powf(t0, 2), powf(t0, 3), powf(t0, 4), powf(t0, 5), powf(t0, 6),
			0, 1, 2*t0, 3*powf(t0, 2), 4*powf(t0, 3), 5*powf(t0, 4), 6*powf(t0, 5),
			0, 0, 2, 6*t0, 12*powf(t0, 2), 20*powf(t0, 3), 30*powf(t0, 4),
			1, tm, powf(tm, 2), powf(tm, 3), powf(tm, 4), powf(tm, 5), powf(tm, 6),
			1, t1, powf(t1, 2), powf(t1, 3), powf(t1, 4), powf(t1, 5), powf(t1, 6),
			0, 1, 2*t1, 3*powf(t1, 2), 4*powf(t1, 3), 5*powf(t1, 4), 6*powf(t1, 5),
			0, 0, 2, 6*t1, 12*powf(t1, 2), 20*powf(t1, 3), 30*powf(t1, 4)
	};
	float32_t bData[seventhOrderVectorDimension] = {
			float32_t(q0), 0.0, 0.0, float32_t(qm), float32_t(q1), 0.0, 0.0
	};
	float32_t aData[seventhOrderVectorDimension] = {0};
	arm_matrix_instance_f32 matrix = {seventhOrderM, seventhOrderN, matrixData};
	arm_matrix_instance_f32 b = {seventhOrderVectorDimension, 1, bData};
	arm_matrix_instance_f32 matrixInverse = {seventhOrderM, seventhOrderN, matrixInverseData};
	arm_matrix_instance_f32 resultA = {seventhOrderVectorDimension, 1, aData};
	status = arm_mat_inverse_f32(&matrix, &matrixInverse);
	this->check_arm_math_status(status);
	status = arm_mat_mult_f32(&matrixInverse, &b, &resultA);
	this->check_arm_math_status(status);
	for(int i = 0; i < int(t1); i++){
		float32_t tc = float(i);
		float32_t qd = float32_t( resultA.pData[0] + resultA.pData[1]*tc + resultA.pData[2]*powf(tc,2) + resultA.pData[3]*powf(tc,3) + resultA.pData[4]*powf(tc,4) + resultA.pData[5]*powf(tc,5) + resultA.pData[6]*powf(tc,6));
		q.push_back(int16_t(qd));
	}
	return q;
}

void Leg::check_arm_math_status(arm_status status){
	if(status != ARM_MATH_SUCCESS){
		toggle_red_led();
		while(1){
			__asm volatile("NOP");
		}
	}
}

void Leg::apply_coxa_angle_offset(int16_t& coxaAngle){
	if(position == LegPosition::LEFT_BACK || position == LegPosition::LEFT_FRONT || position == LegPosition::LEFT_MIDDLE){
		coxaAngle = leftCoxaOffsetAngle - coxaAngle;
		if (position == LegPosition::LEFT_FRONT) coxaAngle += 45;
		if (position == LegPosition::LEFT_BACK) coxaAngle -= 45;
	}else if (position == LegPosition::RIGHT_BACK || position == LegPosition::RIGHT_FRONT || position == LegPosition::RIGHT_MIDDLE){
		coxaAngle = rightCoxaOffsetAngle + coxaAngle;
		if (position == LegPosition::RIGHT_FRONT) coxaAngle -= 45;
		if (position == LegPosition::RIGHT_BACK) coxaAngle += 45;
	}
}

void Leg::apply_femur_angle_offset(int16_t& femurAngle){
	if(position == LegPosition::LEFT_BACK || position == LegPosition::LEFT_FRONT || position == LegPosition::LEFT_MIDDLE){
		femurAngle = leftFemurOffsetAngle - femurAngle;
	}else if (position == LegPosition::RIGHT_BACK || position == LegPosition::RIGHT_FRONT || position == LegPosition::RIGHT_MIDDLE){
		femurAngle = rightFemurOffsetAngle + femurAngle;
	}
}

void Leg::apply_tibia_angle_offset(int16_t& tibiaAngle){
	if(position == LegPosition::LEFT_BACK || position == LegPosition::LEFT_FRONT || position == LegPosition::LEFT_MIDDLE){
		tibiaAngle = leftTibiaOffsetAngle - tibiaAngle;
	}else if (position == LegPosition::RIGHT_BACK || position == LegPosition::RIGHT_FRONT || position == LegPosition::RIGHT_MIDDLE){
		tibiaAngle = rightTibiaOffsetAngle + tibiaAngle;
	}
}

void Leg::update_endeffector_coords(int16_t x, int16_t y, int16_t z){
	this->x = x;
	this->y = y;
	this->z = z;
}

int16_t Leg::get_x(){
	return this->x;
}

int16_t Leg::get_y(){
	return this->y;
}

int16_t Leg::get_z(){
	return this->z;
}

uint16_t Leg::get_coxa_angle(){
	return this->coxaAngle;
}

uint16_t Leg::get_femur_angle(){
	return this->femurAngle;
}

uint16_t Leg::get_tibia_angle(){
	return this->tibiaAngle;
}

LegPhase Leg::get_leg_phase(){
	return this->phase;
}

void Leg::update_leg_phase(LegPhase phase){
	this->phase = phase;
}

LegPosition Leg::get_position(){
	return this->position;
}

uint16_t Leg::correct_cast(int16_t number){
	return ((number < 0) ? 0 : uint16_t(number));
}

void Leg::reset_usage(){
	this->usage = 0;
}

void Leg::increment_usage(){
	this->usage++;
}

int Leg::get_usage(){
	return this->usage;
}
