/*
 * Robot.cpp
 *
 *  Created on: Jan 10, 2018
 *      Author: aron
 */

#include <robot/LegInterpolation.hpp>
#include "Robot.hpp"
#include "RobotConstants.hpp"
#include "arm_math.h"

using namespace robot::body;
using namespace robot::basic;

Robot::Robot(uint32_t baudRate){
	this->dynamixelControl = new DynamixelControl(baudRate);
	this->leftFront = new Leg(1, 3, 5, *dynamixelControl, LegPosition::LEFT_FRONT);
	this->leftMiddle = new Leg(13, 15, 17, *dynamixelControl, LegPosition::LEFT_MIDDLE);
	this->leftBack = new Leg(7, 9, 11, *dynamixelControl, LegPosition::LEFT_BACK);
	this->rightFront = new Leg(2, 4, 6, *dynamixelControl, LegPosition::RIGHT_FRONT);
	this->rightMiddle = new Leg(14, 16, 18, *dynamixelControl, LegPosition::RIGHT_MIDDLE);
	this->rightBack = new Leg(8, 10, 12, *dynamixelControl, LegPosition::RIGHT_BACK);
	this->waveGait = new  WaveGait(this->leftFront, this->leftMiddle, this->leftBack, this->rightFront, this->rightMiddle, this->rightBack);
	this->tripodGait = new TripodGait(this->leftFront, this->leftMiddle, this->leftBack, this->rightFront, this->rightMiddle, this->rightBack);
	this->ruleBased = new RuleBasedGait(this->leftFront, this->leftMiddle, this->leftBack, this->rightFront, this->rightMiddle, this->rightBack);
	this->bodyHeight = 100;
}

Robot::~Robot(){
	delete this->leftFront;
	delete this->leftMiddle;
	delete this->leftBack;
	delete this->rightFront;
	delete this->rightMiddle;
	delete this->rightBack;
}

void Robot::test(){
	int16_t x = 150;
	int16_t y = 0;
	int16_t z = -100;

	for(int i = 0; i < 50000000; i++){
			__asm volatile("NOP");
	}
	this->rightMiddle->update_endeffector_coords(x, y, z);
	this->rightMiddle->solve_ik();
	this->rightFront->update_endeffector_coords(x, y, z);
	this->rightFront->solve_ik();
	this->rightMiddle->set_angles(this->rightMiddle->get_coxa_angle(), this->rightMiddle->get_femur_angle(), this->rightMiddle->get_tibia_angle());
	this->rightFront->set_angles(this->rightFront->get_coxa_angle(), this->rightFront->get_femur_angle(), this->rightFront->get_tibia_angle());
	for(int i = 0; i < 50000000; i++){
				__asm volatile("NOP");
	}
}

void Robot::move_to_start_position(){
	for(int i = 0; i < 50000000; i++){
			__asm volatile("NOP");
	}
	this->move_all_legs_to(startXAir, startYAir, startZAir);
	for(int i = 0; i < 50000000; i++){
		__asm volatile("NOP");
	}
	this->move_all_legs_to(startXGround, startYGround, startZGround);
	for(int i = 0; i < 50000000; i++){
			__asm volatile("NOP");
	}
	this->initiate_gait_phases();
}

void Robot::move_all_legs_to(int x, int y, int z){
	this->move_single_leg_to(x, y, z, leftFront);
	this->move_single_leg_to(x, y, z, leftMiddle);
	this->move_single_leg_to(x, y, z, leftBack);
	this->move_single_leg_to(x, y, z, rightFront);
	this->move_single_leg_to(x, y, z, rightMiddle);
	this->move_single_leg_to(x, y, z, rightBack);
}

void Robot::move_single_leg_to(int x, int y, int z, Leg *leg){
	int16_t coxa, femur, tibia;
	leg->update_endeffector_coords(x, y, z);
	leg->solve_ik();
	coxa = leg->get_coxa_angle();
	femur = leg->get_femur_angle();
	tibia = leg->get_tibia_angle();
	leg->set_angles(coxa, femur, tibia);
}

void Robot::move(float translationX, float translationY, float rotationZ, WalkingGait gait, Mode mode){
	this->filter_zero_noise(translationX, translationY, rotationZ);
	if(fabs(translationX) + fabs(translationY) + fabs(rotationZ) != 0.0){
		switch(mode){
		case Mode::WALKING:
			this->cycle_gait(translationX, translationY, rotationZ, gait);
			break;
		default:
		break;
		}
	}else{
		this->reset_leg_positions();
	}
}

void Robot::filter_zero_noise(float& translationX, float& translationY, float& rotationZ){
	if(translationX < (zeroNoiseFilterConstant * maxPosTranslation) && translationX > (zeroNoiseFilterConstant * maxNegTranslation)){
		translationX = 0.0;
	}
	if(translationY < (zeroNoiseFilterConstant * maxPosTranslation) && translationY > (zeroNoiseFilterConstant * maxNegTranslation)){
		translationY = 0.0;
	}
	if(rotationZ < (zeroNoiseFilterConstant * maxPosRotation) && rotationZ > (zeroNoiseFilterConstant * maxNegRotation)){
		rotationZ = 0.0;
	}
}

void Robot::initiate_gait_phases(){
	this->leftFront->update_leg_phase(LegPhase::SUPPORT);
	this->leftMiddle->update_leg_phase(LegPhase::SUPPORT);
	this->leftBack->update_leg_phase(LegPhase::SUPPORT);
	this->rightFront->update_leg_phase(LegPhase::SUPPORT);
	this->rightMiddle->update_leg_phase(LegPhase::SUPPORT);
	this->rightBack->update_leg_phase(LegPhase::SUPPORT);
}

void Robot::reset_leg_positions(){
	this->reset_single_leg(leftFront);
	for(int i = 0; i < 5000000; i++){
		__asm volatile("NOP");
	}
	this->reset_single_leg(leftMiddle);
	for(int i = 0; i < 5000000; i++){
		__asm volatile("NOP");
	}
	this->reset_single_leg(leftBack);
	for(int i = 0; i < 5000000; i++){
		__asm volatile("NOP");
	}
	this->reset_single_leg(rightFront);
	for(int i = 0; i < 5000000; i++){
		__asm volatile("NOP");
	}
	this->reset_single_leg(rightMiddle);
	for(int i = 0; i < 5000000; i++){
		__asm volatile("NOP");
	}
	this->reset_single_leg(rightBack);
}

void Robot::reset_single_leg(Leg *leg){
	if(leg->get_leg_phase() == LegPhase::SWING){
		this->move_single_leg_to(leg->get_x(), leg->get_y(), -this->bodyHeight, leg);
		leg->update_leg_phase(LegPhase::SUPPORT);
		this->move_single_leg_to(leg->get_x(), leg->get_y(), -this->bodyHeight, leg);
	}
}

void Robot::rotate(int& xRight, int& yRight, int& xLeft, int& yLeft,const float rotationZ){
	float32_t thetaRight = float32_t(rotationMaxAngle * (M_PI / 180.0)) * rotationZ;
	float32_t thetaLeft = float32_t((rotationMaxAngle + 180) * (M_PI / 180.0)) * rotationZ;
	float32_t positionDataRight[thirdOrderVectorDimension] = {
		float32_t(xRight), float32_t(yRight), 0.0
	};
	float32_t positionDataLeft[thirdOrderVectorDimension] = {
		float32_t(xLeft), float32_t(yLeft), 0.0
	};
	float32_t rotationDataRight[thirdOrderMatrixDimension] = {
		cosf(thetaRight), -sinf(thetaRight), 0,
		sinf(thetaRight), cosf(thetaRight), 0,
		0, 0, 1
	};
	float32_t rotationDataLeft[thirdOrderMatrixDimension] = {
		cosf(thetaLeft), -sinf(thetaLeft), 0,
		sinf(thetaLeft), cosf(thetaLeft), 0,
		0, 0, 1
	};
	float32_t rotRightResultData[thirdOrderVectorDimension] = {0};
	float32_t rotLeftResultData[thirdOrderVectorDimension] = {0};
	arm_matrix_instance_f32 positionRight = {1, thirdOrderVectorDimension, positionDataRight};
	arm_matrix_instance_f32 positionLeft = {1, thirdOrderVectorDimension, positionDataLeft};
	arm_matrix_instance_f32 rotationRight = {thirdOrderM, thirdOrderN, rotationDataRight};
	arm_matrix_instance_f32 rotationLeft = {thirdOrderM, thirdOrderN, rotationDataLeft};
	arm_matrix_instance_f32 resultRight = {1, thirdOrderVectorDimension, rotRightResultData};
	arm_matrix_instance_f32 resultLeft = {1, thirdOrderVectorDimension, rotLeftResultData};
	arm_mat_mult_f32(&positionRight, &rotationRight, &resultRight);
	arm_mat_mult_f32(&positionLeft, &rotationLeft, &resultLeft);
	xRight = int(resultRight.pData[0]);
	yRight = int(resultRight.pData[1]);
	xLeft = int(resultLeft.pData[0]);
	yLeft = int(resultLeft.pData[1]);
}

void Robot::cycle_gait(float translationX, float translationY, float rotationZ, WalkingGait gait){
	Gait *currentGait;
	switch(gait){
	case WalkingGait::TRIPOD:
		currentGait = this->tripodGait;
		break;
	case WalkingGait::WAVE:
		currentGait = this->waveGait;
		break;
	case WalkingGait::RULE_BASED:
		currentGait = this->ruleBased;
		break;
	default:
		return;
	}
	int xStepSize = int(translationX * currentGait->get_step_size());
	int yStepSize = int(translationY * currentGait->get_step_size());
	int xStepPositionRight = xStepSize;
	int yStepPositionRight = yStepSize;
	int xStepPositionLeft = -xStepSize;
	int yStepPositionLeft = yStepSize;
	if (rotationZ != 0.0){
		this->rotate(xStepPositionRight, yStepPositionRight, xStepPositionLeft, yStepPositionLeft, rotationZ);
	}
	currentGait->cycle(xStepPositionRight, yStepPositionRight, xStepPositionLeft, yStepPositionLeft, this->bodyHeight);
}
